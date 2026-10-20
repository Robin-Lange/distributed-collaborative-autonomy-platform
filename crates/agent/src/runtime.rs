//! Main runtime loop for the onboard agent.
//!
//! The runtime owns the event loop that waits for adapter input, drains each
//! channel (latest-wins for intent/state/status; every-event fold for the
//! keyed constraint set), builds a read-only `DecisionInput`, and publishes
//! the resulting command/authority/veto bundle. It keeps latches for active
//! intent, the source-keyed active-constraint set, active veto state, and the
//! hold-episode anchor so safety transitions are explicit.

use std::collections::BTreeMap;
use std::time::Duration;

use core_msgs::{
    APPROVED_BY_ONBOARD_AGENT, AgentScoped, AgentStateMsg, Behavior, BehaviorPhase,
    BehaviorPrimitive, BehaviorRequest, BehaviorSource, BehaviorStatus, BehaviorStatusReason,
    CapabilityReport, CapabilitySet, CommandStatus, CommandVeto, ControlAction, ControlRequest,
    ControlSource, ManipulationCommand, ManipulationIntent, ObjectState, PlatformCommand,
    PositionTarget, WaypointIntent, behavior_code, catalog::ObjectCatalog, command_source_autonomy,
    command_source_human_operator, command_type_hold, command_type_stop, command_type_velocity,
    constraint_type_robot_fault, constraints::ActiveConstraints, manipulation_command_id,
    manipulation_failure_reason_no_object_detected, manipulation_failure_reason_safety_stop,
    manipulation_failure_reason_unsafe_grip_height, manipulation_phase_failed,
    manipulation_phase_succeeded, manipulation_verb_drop_off, manipulation_verb_pick_up,
    object_lifecycle_free, object_lifecycle_held, veto_reason_collision_stop,
    veto_reason_robot_fault, veto_reason_unauthorized_source,
};
use crossbeam_channel::{Receiver, Select};
use dds_support::{drain_each_with_status, drain_latest_with_status};

use super::{
    authority,
    behavior_gate::BehaviorGate,
    bus_adapter::{AdapterDown, BusAdapter, ManipulationOutput, PlatformInputs, PlatformOutput},
    config::{AgentConfig, AgentManifest},
    decision::{self, DecisionConfig, DecisionOutcome, HoldAnchor},
    manipulation::{self, ActiveManipulationTask},
};

const MANIPULATION_BASE_MAX_LINEAR_SPEED_M_S: f32 = 0.05;
const MANIPULATION_BASE_MAX_YAW_RATE_RAD_S: f32 = 0.10;
const MANIPULATION_STATE_CONFIRM_TIMEOUT_MS: u64 = 2_000;
/// Allows a ten-second SDK phase failure plus the sidecar's bounded recovery
/// before declaring that no terminal executor status will arrive.
const MANIPULATION_EXECUTION_TIMEOUT_MS: u64 = 45_000;
const AUTHORITY_HEARTBEAT_MS: u64 = 1_000;

/// Mutable runtime state carried across decision ticks.
struct RuntimeState {
    /// Latest input messages latched for decision making.
    inbox: RuntimeInbox,

    /// Whether the last published decision had an active veto.
    veto_active: bool,

    /// Last authority heartbeat placed on the bus.
    authority_published_ms: Option<u64>,

    /// Hold target locked for the hold episode in progress, if any.
    ///
    /// Latched from the first hold command of an episode (for a collision
    /// stop, the state at the moment of impact) and reused for every later
    /// hold, so the hold target never chases the platform's post-impact
    /// drift. Released when a velocity command resumes motion.
    hold_anchor: Option<HoldAnchor>,

    /// Manipulation task in flight, if any. Latched when a
    /// `ManipulationCommand` is queued and released when execution fails or a
    /// successful result is confirmed by the object's lifecycle. Enforces
    /// "single active task" the same way `hold_anchor` enforces one hold
    /// episode.
    manipulation_task: Option<ActiveManipulationTask>,

    /// Successful executor result awaiting the corresponding object lifecycle
    /// transition. A behavior is not complete until both agree.
    pending_manipulation_success: Option<core_msgs::ManipulationStatus>,

    /// Operator/autonomy admission state. Operator overrides remain latched
    /// here until an explicit `ResumeAutonomy` command arrives.
    behavior_gate: BehaviorGate,

    /// Typed request currently represented by the executor's active state.
    active_behavior: Option<BehaviorRequest>,

    /// Typed manipulation request converted for the current executor.
    pending_manipulation_intent: Option<ManipulationIntent>,

    /// A navigation request accepted during an Avian wall-stop episode waits
    /// until Avian publishes the matching clear. This prevents the previous
    /// command's collision from consuming the first recovery request.
    navigation_waits_for_environment_wall: bool,

    /// Last liveliness state included in a capability report.
    reported_platform_alive: PublishedPlatformAvailability,
}

impl RuntimeState {
    fn new(autonomy_enabled: bool) -> Self {
        Self {
            inbox: RuntimeInbox::default(),
            veto_active: false,
            authority_published_ms: None,
            hold_anchor: None,
            manipulation_task: None,
            pending_manipulation_success: None,
            behavior_gate: BehaviorGate::new(autonomy_enabled),
            active_behavior: None,
            pending_manipulation_intent: None,
            navigation_waits_for_environment_wall: false,
            reported_platform_alive: PublishedPlatformAvailability::NotPublished,
        }
    }
}

/// Tracks whether the current platform availability has already been reported.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PublishedPlatformAvailability {
    /// No capability report has been sent.
    NotPublished,

    /// The contained availability (`None` means unknown) has been sent.
    Published(Option<bool>),
}

/// Latest message per input stream for the configured agent.
#[derive(Default)]
struct RuntimeInbox {
    /// Newest accepted waypoint intent.
    latest_intent: Option<WaypointIntent>,

    /// Newest accepted state sample from the expected authority source.
    latest_state: Option<AgentStateMsg>,

    /// Active constraint conditions keyed by `(source, constraint_type,
    /// object_id)`. Every drained event is folded in (a latest-wins latch
    /// would drop interleaved events from independent producers); a
    /// `severity_none` event clears only its matching key.
    active_constraints: ActiveConstraints,

    /// Newest command status message.
    latest_status: Option<CommandStatus>,

    /// Latest observed lifecycle/pose per catalog object, keyed by
    /// `object_id`. A snapshot stream (not one-shot events like
    /// `ManipulationIntent`/`ManipulationStatus`), so every drained message
    /// simply overwrites its key — no fold-vs-latest distinction needed.
    object_states: BTreeMap<String, ObjectState>,

    /// Latest platform-bridge liveliness: `Some(false)` = confirmed dead (via
    /// Zenoh liveliness `Delete`), `Some(true)` = alive, `None` = no info yet.
    /// Latest-wins; `None` never triggers a hold.
    latest_platform_alive: Option<bool>,
}

/// Crossbeam select wrapper used to avoid busy-waiting between decision ticks.
struct RuntimeInputWaiter<'a> {
    /// Select set covering every adapter input channel.
    select: Select<'a>,

    /// Maximum wait before the runtime reevaluates stale-state safety.
    watchdog: Duration,
}

/// Identifies which input channel disconnected unexpectedly.
#[derive(Debug, Clone, Copy)]
struct ChannelDisconnected {
    /// Human-readable channel name for the shutdown log.
    channel: &'static str,
}

impl<'a> RuntimeInputWaiter<'a> {
    /// Registers all input receivers in one select set.
    fn new(inputs: &'a PlatformInputs, watchdog: Duration) -> Self {
        let mut select = Select::new();

        select.recv(inputs.intent_rx());
        select.recv(inputs.behavior_request_rx());
        select.recv(inputs.resume_autonomy_rx());
        select.recv(inputs.state_rx());
        select.recv(inputs.constraint_rx());
        select.recv(inputs.status_rx());
        select.recv(inputs.manipulation_intent_rx());
        select.recv(inputs.manipulation_status_rx());
        select.recv(inputs.object_state_rx());
        // Registered so a liveliness `Delete` wakes the loop immediately — the
        // whole point of event-based death detection over staleness polling.
        select.recv(inputs.platform_alive_rx());

        Self { select, watchdog }
    }

    /// Blocks until input is ready or the watchdog timeout expires.
    fn wait(&mut self) {
        let _ = self.select.ready_timeout(self.watchdog);
    }
}

impl RuntimeInbox {
    /// Drains all input channels and updates latest-wins latches.
    ///
    /// Returns a supersede veto when a fresh intent has just replaced a
    /// still-unfinished autonomy intent in the latch, so the caller can emit
    /// the terminal feedback the awaiting planner would otherwise never get.
    fn update_from_channels(
        &mut self,
        inputs: &PlatformInputs,
        agent_id: &str,
        expected_state_authority: u8,
        now_ms: u64,
    ) -> Result<Option<CommandVeto>, ChannelDisconnected> {
        // Capture the previously latched intent before the latch is overwritten:
        // if a genuinely new intent replaces a still-unfinished autonomy intent,
        // the planner awaiting the old command's terminal feedback must be told
        // it was superseded rather than left to hang.
        let previous_intent = self.latest_intent.clone();
        update_latest(
            inputs.intent_rx(),
            &mut self.latest_intent,
            "intent",
            agent_id,
            |_| true,
        )?;
        let supersede_veto = self.latest_intent.as_ref().and_then(|current| {
            decision::superseded_intent_veto(agent_id, previous_intent.as_ref(), current, now_ms)
        });

        // State is accepted only from the authority selected by the active mode;
        // this prevents sim and real state from fighting in hybrid deployments.
        update_latest(
            inputs.state_rx(),
            &mut self.latest_state,
            "state",
            agent_id,
            |state| state.state_authority == expected_state_authority,
        )?;

        // Constraints are independent events from independent producers: fold
        // every drained message into the keyed set instead of keeping only the
        // newest sample.
        let constraints = &mut self.active_constraints;
        let disconnected = drain_each_with_status(
            inputs.constraint_rx(),
            |message| message.agent_id() == agent_id,
            |message| {
                constraints.apply(message, now_ms);
            },
        );
        if disconnected {
            return Err(ChannelDisconnected {
                channel: "constraint",
            });
        }

        update_latest(
            inputs.status_rx(),
            &mut self.latest_status,
            "status",
            agent_id,
            |_| true,
        )?;

        // Object state is a snapshot stream, not agent-scoped (an object
        // belongs to the shared catalog); every drained sample overwrites
        // its own key.
        let object_states = &mut self.object_states;
        let object_state_disconnected = drain_each_with_status(
            inputs.object_state_rx(),
            |_| true,
            |message: ObjectState| {
                object_states.insert(message.object_id.clone(), message);
            },
        );
        if object_state_disconnected {
            return Err(ChannelDisconnected {
                channel: "object_state",
            });
        }

        // Liveliness (`bool`, not agent-scoped: the subscription's key already
        // pins it to this agent's bridge). Latest-wins latch; a `Delete`
        // (`false`) is a confirmed-dead edge the decision gate holds on.
        let alive = drain_latest_with_status(inputs.platform_alive_rx(), |_| true);
        let (latest_alive, alive_disconnected) = alive.into_parts();
        if let Some(value) = latest_alive {
            self.latest_platform_alive = Some(value);
        }
        if alive_disconnected {
            return Err(ChannelDisconnected {
                channel: "platform_alive",
            });
        }

        // Environment constraints are level-published (re-asserted every
        // physics tick); entries that stop being re-asserted expire as a
        // transport backstop. Spot faults are exempt and are released only by
        // the sidecar's own matching clear.
        self.active_constraints.expire(now_ms);

        // A blocked-goal report describes a *previous* command; a fresh intent
        // supersedes it. No other constraint type is cleared by intent recency
        // — in particular, robot faults and wall stops survive new intents.
        if let Some(intent) = self.latest_intent.as_ref() {
            self.active_constraints
                .drop_goal_blocked_older_than(intent.stamp_ms);
        }

        Ok(supersede_veto)
    }

    /// Clears the intent latch when requested by the decision outcome.
    ///
    /// Constraints are deliberately not clearable from decision flow: each
    /// producer owns its conditions' lifecycle (explicit clear or TTL expiry),
    /// so an idle tick or a consumed intent can never erase an active hard stop.
    fn clear_decision_latches(&mut self, outcome: &DecisionOutcome) {
        if outcome.clears_intent() {
            self.latest_intent = None;
        }
    }
}

enum BehaviorProcessingError {
    Channel(ChannelDisconnected),
    Adapter(AdapterDown),
}

enum RoutedBehavior {
    Navigation(WaypointIntent),
    Manipulation(ManipulationIntent),
}

fn process_resume_autonomy(
    inputs: &PlatformInputs,
    gate: &mut BehaviorGate,
    agent_id: &str,
) -> Result<(), ChannelDisconnected> {
    let mut commands = Vec::new();
    let disconnected = drain_each_with_status(
        inputs.resume_autonomy_rx(),
        |command| command.agent_id() == agent_id,
        |command| commands.push(command),
    );

    for command in commands {
        if !command.request_id.trim().is_empty() {
            gate.resume_autonomy();
        }
    }

    if disconnected {
        Err(ChannelDisconnected {
            channel: "resume_autonomy",
        })
    } else {
        Ok(())
    }
}

fn process_behavior_requests(
    inputs: &PlatformInputs,
    bus: &BusAdapter,
    state: &mut RuntimeState,
    capabilities: &CapabilitySet,
    agent_id: &str,
    now_ms: u64,
) -> Result<(), BehaviorProcessingError> {
    let mut requests = Vec::new();
    let disconnected = drain_each_with_status(
        inputs.behavior_request_rx(),
        |request| request.agent_id() == agent_id,
        |request| requests.push(request),
    );

    if disconnected {
        return Err(BehaviorProcessingError::Channel(ChannelDisconnected {
            channel: "behavior_request",
        }));
    }

    for request in requests {
        if let Err(rejection) = state.behavior_gate.admit(
            &request,
            agent_id,
            capabilities,
            state.inbox.latest_platform_alive,
        ) {
            publish_behavior_status(
                bus,
                &request,
                BehaviorPhase::Rejected,
                Some(rejection.reason),
                rejection.detail,
                now_ms,
            )
            .map_err(BehaviorProcessingError::Adapter)?;
            continue;
        }

        if state.manipulation_task.is_some() {
            publish_behavior_status(
                bus,
                &request,
                BehaviorPhase::Rejected,
                Some(BehaviorStatusReason::UnmetPrecondition),
                "a manipulation task is already active".to_owned(),
                now_ms,
            )
            .map_err(BehaviorProcessingError::Adapter)?;
            continue;
        }

        if state
            .active_behavior
            .as_ref()
            .is_some_and(|active| active.request_id == request.request_id)
        {
            continue;
        }

        if let Some(active) = state.active_behavior.take() {
            let operator_preempted = request.source == BehaviorSource::Operator;
            publish_behavior_status(
                bus,
                &active,
                if operator_preempted {
                    BehaviorPhase::Preempted
                } else {
                    BehaviorPhase::Cancelled
                },
                operator_preempted.then_some(BehaviorStatusReason::PreemptedByOperator),
                format!("superseded by request '{}'", request.request_id),
                now_ms,
            )
            .map_err(BehaviorProcessingError::Adapter)?;
        }

        state.inbox.latest_intent = None;
        state.pending_manipulation_intent = None;

        match route_behavior(&request) {
            RoutedBehavior::Navigation(intent) => {
                state.navigation_waits_for_environment_wall =
                    state.inbox.active_constraints.has_environment_wall_stop();
                state.inbox.latest_intent = Some(intent);
            }
            RoutedBehavior::Manipulation(intent) => {
                state.navigation_waits_for_environment_wall = false;
                state.pending_manipulation_intent = Some(intent);
            }
        }
        state.active_behavior = Some(request.clone());

        publish_behavior_status(
            bus,
            &request,
            BehaviorPhase::Accepted,
            None,
            String::new(),
            now_ms,
        )
        .map_err(BehaviorProcessingError::Adapter)?;
        publish_behavior_status(
            bus,
            &request,
            BehaviorPhase::Active,
            None,
            String::new(),
            now_ms,
        )
        .map_err(BehaviorProcessingError::Adapter)?;
    }

    Ok(())
}

fn route_behavior(request: &BehaviorRequest) -> RoutedBehavior {
    let source = match request.source {
        BehaviorSource::Autonomy => command_source_autonomy(),
        BehaviorSource::Operator => command_source_human_operator(),
    };

    match &request.behavior {
        BehaviorPrimitive::NavigateTo { target, .. } => {
            RoutedBehavior::Navigation(WaypointIntent {
                agent_id: request.agent_id.clone(),
                source,
                behavior: behavior_code(Behavior::GoTo),
                point: position_target_array(*target),
                stamp_ms: request.stamp_ms,
                frame_id: request.frame_id.clone(),
            })
        }
        BehaviorPrimitive::MoveAway { from } => RoutedBehavior::Navigation(WaypointIntent {
            agent_id: request.agent_id.clone(),
            source,
            behavior: behavior_code(Behavior::MoveAway),
            point: position_target_array(*from),
            stamp_ms: request.stamp_ms,
            frame_id: request.frame_id.clone(),
        }),
        BehaviorPrimitive::Hold => RoutedBehavior::Navigation(WaypointIntent {
            agent_id: request.agent_id.clone(),
            source,
            behavior: behavior_code(Behavior::Hold),
            point: [0.0; 3],
            stamp_ms: request.stamp_ms,
            frame_id: request.frame_id.clone(),
        }),
        BehaviorPrimitive::Pick { object_id } => RoutedBehavior::Manipulation(ManipulationIntent {
            agent_id: request.agent_id.clone(),
            object_id: object_id.clone(),
            verb: manipulation_verb_pick_up(),
            source,
            stamp_ms: request.stamp_ms,
            frame_id: request.frame_id.clone(),
        }),
        BehaviorPrimitive::Place { object_id } => {
            RoutedBehavior::Manipulation(ManipulationIntent {
                agent_id: request.agent_id.clone(),
                object_id: object_id.clone(),
                verb: manipulation_verb_drop_off(),
                source,
                stamp_ms: request.stamp_ms,
                frame_id: request.frame_id.clone(),
            })
        }
    }
}

const fn position_target_array(target: PositionTarget) -> [f32; 3] {
    match target {
        PositionTarget::Planar([x, y]) => [x, y, 0.0],
        PositionTarget::Spatial(position) => position,
    }
}

fn publish_behavior_status(
    bus: &BusAdapter,
    request: &BehaviorRequest,
    phase: BehaviorPhase,
    reason: Option<BehaviorStatusReason>,
    detail: String,
    stamp_ms: u64,
) -> Result<(), AdapterDown> {
    bus.deliver_behavior_status(BehaviorStatus {
        request_id: request.request_id.clone(),
        agent_id: request.agent_id.clone(),
        phase,
        reason,
        detail,
        stamp_ms,
    })
}

fn publish_capability_report_if_changed(
    bus: &BusAdapter,
    reported_platform_alive: &mut PublishedPlatformAvailability,
    platform_alive: Option<bool>,
    capabilities: &CapabilitySet,
    manifest: &AgentManifest,
    stamp_ms: u64,
) -> Result<(), AdapterDown> {
    if *reported_platform_alive == PublishedPlatformAvailability::Published(platform_alive) {
        return Ok(());
    }

    let unavailable_reason = match platform_alive {
        Some(true) => None,
        Some(false) => Some("platform adapter is disconnected".to_owned()),
        None => Some("platform adapter availability is not known yet".to_owned()),
    };

    bus.deliver_capability_report(CapabilityReport {
        agent_id: manifest.agent_id.clone(),
        platform_profile: manifest.platform.clone(),
        connected: platform_alive == Some(true),
        unavailable_reason,
        capabilities: capabilities.clone(),
        stamp_ms,
    })?;
    *reported_platform_alive = PublishedPlatformAvailability::Published(platform_alive);
    Ok(())
}

fn should_wait_for_environment_wall_clear(waiting: bool, constraints: &ActiveConstraints) -> bool {
    waiting && constraints.has_environment_wall_stop()
}

/// Fixed dependencies shared by every runtime tick.
struct RuntimeDependencies<'a> {
    manifest: &'a AgentManifest,
    capabilities: &'a CapabilitySet,
    catalog: &'a ObjectCatalog,
    bus: &'a BusAdapter,
    inputs: &'a PlatformInputs,
    decision_config: DecisionConfig,
    expected_state_authority: u8,
}

/// Runs the onboard agent forever until an adapter/input channel fails.
pub fn run(
    config: &AgentConfig,
    manifest: &AgentManifest,
    capabilities: &core_msgs::CapabilitySet,
    catalog: &ObjectCatalog,
    bus: &BusAdapter,
    inputs: &PlatformInputs,
) -> ! {
    let watchdog = Duration::from_millis(manifest.state_timeout_ms);
    let dependencies = RuntimeDependencies {
        manifest,
        capabilities,
        catalog,
        bus,
        inputs,
        decision_config: DecisionConfig::from_agent_config(
            config,
            manifest.state_timeout_ms,
            manifest.autonomy_enabled,
            manifest.max_speed,
            capabilities,
        ),
        expected_state_authority: authority::expected_state_authority(manifest),
    };

    let mut runtime_authority = authority::authority_from_manifest(manifest, core_msgs::now_ms());
    let mut runtime_state = RuntimeState::new(manifest.autonomy_enabled);
    let mut input_waiter = RuntimeInputWaiter::new(inputs, watchdog);

    loop {
        input_waiter.wait();
        let now_ms = core_msgs::now_ms();
        let supersede_veto = receive_runtime_inputs(&dependencies, &mut runtime_state, now_ms);
        let drive_active = run_navigation_tick(
            &dependencies,
            &mut runtime_state,
            &mut runtime_authority,
            supersede_veto,
            now_ms,
        );

        if let Err(down) = run_manipulation_tick(
            dependencies.bus,
            dependencies.inputs,
            &mut runtime_state.manipulation_task,
            &mut runtime_state.pending_manipulation_intent,
            &mut runtime_state.active_behavior,
            ManipulationTickInputs {
                catalog: dependencies.catalog,
                agent_id: dependencies.decision_config.agent_id.as_str(),
                runtime_authority: &runtime_authority,
                inbox: &runtime_state.inbox,
                drive_active,
                now_ms,
            },
        ) {
            terminate_for_adapter(&down);
        }
    }
}

fn receive_runtime_inputs(
    dependencies: &RuntimeDependencies<'_>,
    state: &mut RuntimeState,
    now_ms: u64,
) -> Option<CommandVeto> {
    let agent_id = dependencies.decision_config.agent_id.as_str();
    let supersede_veto = channel_or_exit(state.inbox.update_from_channels(
        dependencies.inputs,
        agent_id,
        dependencies.expected_state_authority,
        now_ms,
    ));

    adapter_or_exit(reconcile_manipulation_status(
        dependencies.bus,
        dependencies.inputs,
        agent_id,
        state,
        now_ms,
    ));

    channel_or_exit(process_resume_autonomy(
        dependencies.inputs,
        &mut state.behavior_gate,
        agent_id,
    ));

    behavior_or_exit(process_behavior_requests(
        dependencies.inputs,
        dependencies.bus,
        state,
        dependencies.capabilities,
        agent_id,
        now_ms,
    ));

    adapter_or_exit(publish_capability_report_if_changed(
        dependencies.bus,
        &mut state.reported_platform_alive,
        state.inbox.latest_platform_alive,
        dependencies.capabilities,
        dependencies.manifest,
        now_ms,
    ));

    supersede_veto
}

fn run_navigation_tick(
    dependencies: &RuntimeDependencies<'_>,
    state: &mut RuntimeState,
    runtime_authority: &mut core_msgs::AuthorityState,
    supersede_veto: Option<CommandVeto>,
    now_ms: u64,
) -> bool {
    if let Some(veto) = supersede_veto {
        adapter_or_exit(dependencies.bus.deliver_veto(&veto));
    }

    runtime_authority.stamp_ms = now_ms;
    state.navigation_waits_for_environment_wall = should_wait_for_environment_wall_clear(
        state.navigation_waits_for_environment_wall,
        &state.inbox.active_constraints,
    );
    let decision_intent = if state.navigation_waits_for_environment_wall {
        None
    } else {
        state.inbox.latest_intent.as_ref()
    };
    let outcome = decision::next_decision(
        &dependencies.decision_config,
        runtime_authority,
        now_ms,
        &decision::DecisionInputs {
            intent: decision_intent,
            state: state.inbox.latest_state.as_ref(),
            constraints: &state.inbox.active_constraints,
            status: state.inbox.latest_status.as_ref(),
            hold_anchor: state.hold_anchor.clone(),
            platform_alive: state.inbox.latest_platform_alive,
        },
    );

    if outcome.clears_intent() {
        state.navigation_waits_for_environment_wall = false;
    }
    state.inbox.clear_decision_latches(&outcome);
    state.hold_anchor = outcome.next_hold_anchor(state.hold_anchor.take());

    let typed_request_id = state
        .active_behavior
        .as_ref()
        .filter(|active| !is_manipulation_behavior(&active.behavior))
        .map(|active| active.request_id.as_str());
    let publication = adapter_or_exit(publish_decision_outputs(
        dependencies.bus,
        dependencies.decision_config.agent_id.as_str(),
        outcome,
        &mut state.veto_active,
        &mut state.authority_published_ms,
        typed_request_id,
        now_ms,
    ));

    if let Some(terminal) = publication.terminal
        && let Some(active) = state
            .active_behavior
            .take_if(|active| !is_manipulation_behavior(&active.behavior))
    {
        adapter_or_exit(publish_behavior_status(
            dependencies.bus,
            &active,
            terminal.phase,
            terminal.reason,
            terminal.detail,
            now_ms,
        ));
    }

    publication.drive_active
}

fn terminate_for_channel(disconnected: ChannelDisconnected) -> ! {
    eprintln!(
        "agent: {} channel disconnected (DDS publisher dropped); shutting down",
        disconnected.channel
    );
    std::process::exit(1);
}

fn terminate_for_adapter(down: &AdapterDown) -> ! {
    eprintln!(
        "agent: {} adapter thread disconnected; shutting down",
        down.adapter_name()
    );
    std::process::exit(1);
}

fn terminate_for_behavior_error(error: BehaviorProcessingError) -> ! {
    match error {
        BehaviorProcessingError::Channel(disconnected) => terminate_for_channel(disconnected),
        BehaviorProcessingError::Adapter(down) => terminate_for_adapter(&down),
    }
}

fn channel_or_exit<T>(result: Result<T, ChannelDisconnected>) -> T {
    match result {
        Ok(value) => value,
        Err(disconnected) => terminate_for_channel(disconnected),
    }
}

fn adapter_or_exit<T>(result: Result<T, AdapterDown>) -> T {
    match result {
        Ok(value) => value,
        Err(down) => terminate_for_adapter(&down),
    }
}

fn behavior_or_exit<T>(result: Result<T, BehaviorProcessingError>) -> T {
    match result {
        Ok(value) => value,
        Err(error) => terminate_for_behavior_error(error),
    }
}

/// Inputs for one manipulation tick.
#[derive(Copy, Clone)]
struct ManipulationTickInputs<'a> {
    catalog: &'a ObjectCatalog,
    agent_id: &'a str,
    runtime_authority: &'a core_msgs::AuthorityState,
    inbox: &'a RuntimeInbox,

    /// Whether the base is actively driving (a velocity command) toward a
    /// goal; feeds the manipulation gate's "base must be stopped" precondition.
    drive_active: bool,
    now_ms: u64,
}

fn platform_base_is_moving(state: Option<&AgentStateMsg>) -> bool {
    let Some(state) = state else {
        return false;
    };
    let [x, y, z] = state.velocity;
    if !x.is_finite() || !y.is_finite() || !z.is_finite() || !state.yaw_rate_rad_s.is_finite() {
        return true;
    }

    let linear_speed_squared = x.mul_add(x, y.mul_add(y, z * z));
    linear_speed_squared > MANIPULATION_BASE_MAX_LINEAR_SPEED_M_S.powi(2)
        || state.yaw_rate_rad_s.abs() > MANIPULATION_BASE_MAX_YAW_RATE_RAD_S
}

/// Drains manipulation status/intent input for one tick and, if a new
/// intent arrived, evaluates it and publishes the resulting command/veto.
///
/// Active hard-stop constraints are checked here — before
/// `manipulation::next_manipulation_decision` is ever called — reusing the
/// same `ActiveConstraints`/trust predicate `decision.rs` uses for
/// navigation, exactly mirroring `safety_hold_for_available_state`'s role of
/// running ahead of intent-specific logic.
fn run_manipulation_tick(
    bus: &BusAdapter,
    inputs: &PlatformInputs,
    manipulation_task: &mut Option<ActiveManipulationTask>,
    pending_intent: &mut Option<ManipulationIntent>,
    active_behavior: &mut Option<BehaviorRequest>,
    tick_inputs: ManipulationTickInputs<'_>,
) -> Result<(), AdapterDown> {
    let agent_id = tick_inputs.agent_id;
    let now_ms = tick_inputs.now_ms;

    let intent_drained = drain_latest_with_status(inputs.manipulation_intent_rx(), |message| {
        message.agent_id() == agent_id
    });
    let Some(intent) = pending_intent
        .take()
        .or_else(|| intent_drained.into_latest())
    else {
        return Ok(());
    };
    let typed_request = active_behavior
        .as_ref()
        .filter(|active| is_manipulation_behavior(&active.behavior))
        .map(|active| (active.request_id.clone(), active.source));
    let request_id = typed_request.as_ref().map_or_else(
        || manipulation_command_id(agent_id, intent.stamp_ms),
        |(request_id, _)| request_id.clone(),
    );

    let trusted =
        |source: u8| authority::trusts_constraint_source(tick_inputs.runtime_authority, source);
    let outcome = if let Some(hard_stop) = tick_inputs
        .inbox
        .active_constraints
        .active_hard_stop(trusted)
    {
        let is_fault = hard_stop.constraint_type == constraint_type_robot_fault();
        manipulation::ManipulationOutcome::new(
            None,
            Some(CommandVeto {
                agent_id: agent_id.to_owned(),
                command_id: request_id.clone(),
                source: intent.source,
                reason: if is_fault {
                    veto_reason_robot_fault()
                } else {
                    veto_reason_collision_stop()
                },
                detail: "Active hard stop: manipulation refused".to_owned(),
                stamp_ms: now_ms,
            }),
            None,
        )
    } else {
        manipulation::next_manipulation_decision(
            agent_id,
            tick_inputs.catalog,
            now_ms,
            manipulation::ManipulationInputs {
                intent: Some(&intent),
                request_id: &request_id,
                agent_state: tick_inputs.inbox.latest_state.as_ref(),
                object_states: &tick_inputs.inbox.object_states,
                trajectory_active: tick_inputs.drive_active
                    || platform_base_is_moving(tick_inputs.inbox.latest_state.as_ref()),
                task_active: manipulation_task.is_some(),
            },
        )
        .unwrap_or_else(|| manipulation::ManipulationOutcome::new(None, None, None))
    };

    if let Some(command) = outcome.command.as_ref() {
        let source =
            typed_request.map_or_else(|| behavior_source(intent.source), |(_, source)| source);
        bus.deliver_manipulation_control_request(control_request_from_manipulation_command(
            command, request_id, source,
        ))?;
    }

    if let Some(task) = &outcome.started_task {
        *manipulation_task = Some(task.clone());
    }

    if let Some(veto) = outcome.veto.as_ref()
        && let Some(active) =
            active_behavior.take_if(|active| is_manipulation_behavior(&active.behavior))
    {
        let safety_stop =
            veto.reason == veto_reason_collision_stop() || veto.reason == veto_reason_robot_fault();
        publish_behavior_status(
            bus,
            &active,
            if safety_stop {
                BehaviorPhase::Failed
            } else {
                BehaviorPhase::Rejected
            },
            Some(if safety_stop {
                BehaviorStatusReason::SafetyStop
            } else if veto.reason == veto_reason_unauthorized_source() {
                BehaviorStatusReason::Unauthorized
            } else {
                BehaviorStatusReason::UnmetPrecondition
            }),
            veto.detail.clone(),
            now_ms,
        )?;
    }

    bus.deliver_manipulation(&ManipulationOutput::new(outcome.command, outcome.veto))
}

/// Applies a matching terminal executor status before new behavior requests are
/// admitted in the same runtime tick.
fn reconcile_manipulation_status(
    bus: &BusAdapter,
    inputs: &PlatformInputs,
    agent_id: &str,
    state: &mut RuntimeState,
    now_ms: u64,
) -> Result<(), AdapterDown> {
    let status = drain_latest_with_status(inputs.manipulation_status_rx(), |message| {
        message.agent_id() == agent_id
    })
    .into_latest();
    if let Some(status) = status {
        let terminal = status.phase == manipulation_phase_succeeded()
            || status.phase == manipulation_phase_failed();
        let matches_active_task = state.manipulation_task.as_ref().is_some_and(|task| {
            task.command_id == status.command_id && task.object_id == status.object_id
        });
        if terminal && matches_active_task {
            if status.phase == manipulation_phase_succeeded() {
                state.pending_manipulation_success = Some(status);
            } else {
                state.pending_manipulation_success = None;
                state.manipulation_task = None;
                let (reason, detail) = terminal_manipulation_failure(status.failure_reason);
                return publish_terminal_manipulation_behavior(
                    bus,
                    &mut state.active_behavior,
                    BehaviorPhase::Failed,
                    Some(reason),
                    detail.to_owned(),
                    status.stamp_ms,
                );
            }
        }
    }

    if state.pending_manipulation_success.is_none()
        && state
            .manipulation_task
            .as_ref()
            .is_some_and(|task| manipulation_task_timed_out(task, now_ms))
    {
        state.manipulation_task = None;
        return publish_terminal_manipulation_behavior(
            bus,
            &mut state.active_behavior,
            BehaviorPhase::Failed,
            Some(BehaviorStatusReason::ExecutionFailed),
            "platform did not report manipulation completion before the execution timeout"
                .to_owned(),
            now_ms,
        );
    }

    let (Some(status), Some(task)) = (
        state.pending_manipulation_success.as_ref(),
        state.manipulation_task.as_ref(),
    ) else {
        return Ok(());
    };
    if let Some(confirmed_stamp_ms) =
        manipulation_confirmation_stamp(task, &state.inbox.object_states, agent_id)
    {
        let status_stamp_ms = status.stamp_ms;
        state.pending_manipulation_success = None;
        state.manipulation_task = None;
        return publish_terminal_manipulation_behavior(
            bus,
            &mut state.active_behavior,
            BehaviorPhase::Succeeded,
            None,
            String::new(),
            status_stamp_ms.max(confirmed_stamp_ms),
        );
    }
    if now_ms.saturating_sub(status.stamp_ms) < MANIPULATION_STATE_CONFIRM_TIMEOUT_MS {
        return Ok(());
    }

    state.pending_manipulation_success = None;
    state.manipulation_task = None;
    publish_terminal_manipulation_behavior(
        bus,
        &mut state.active_behavior,
        BehaviorPhase::Failed,
        Some(BehaviorStatusReason::ExecutionFailed),
        "platform reported manipulation success, but object lifecycle did not confirm the result"
            .to_owned(),
        now_ms,
    )
}

const fn manipulation_task_timed_out(task: &ActiveManipulationTask, now_ms: u64) -> bool {
    now_ms.saturating_sub(task.started_ms) >= MANIPULATION_EXECUTION_TIMEOUT_MS
}

const fn terminal_manipulation_failure(failure_reason: u8) -> (BehaviorStatusReason, &'static str) {
    if failure_reason == manipulation_failure_reason_safety_stop() {
        (
            BehaviorStatusReason::SafetyStop,
            "platform manipulation stopped for safety",
        )
    } else if failure_reason == manipulation_failure_reason_no_object_detected() {
        (
            BehaviorStatusReason::ExecutionFailed,
            "gripper reached fully closed without detecting object contact",
        )
    } else if failure_reason == manipulation_failure_reason_unsafe_grip_height() {
        (
            BehaviorStatusReason::ExecutionFailed,
            "requested grip point is below the platform's safe floor clearance",
        )
    } else {
        (
            BehaviorStatusReason::ExecutionFailed,
            "platform manipulation execution failed; inspect the platform runtime log",
        )
    }
}

fn manipulation_confirmation_stamp(
    task: &ActiveManipulationTask,
    object_states: &BTreeMap<String, ObjectState>,
    agent_id: &str,
) -> Option<u64> {
    let state = object_states.get(&task.object_id)?;
    let confirmed = if task.verb == manipulation_verb_pick_up() {
        state.lifecycle == object_lifecycle_held() && state.holder_agent_id == agent_id
    } else {
        debug_assert_eq!(task.verb, manipulation_verb_drop_off());
        state.lifecycle == object_lifecycle_free() && state.holder_agent_id.is_empty()
    };
    confirmed.then_some(state.stamp_ms)
}

fn publish_terminal_manipulation_behavior(
    bus: &BusAdapter,
    active_behavior: &mut Option<BehaviorRequest>,
    phase: BehaviorPhase,
    reason: Option<BehaviorStatusReason>,
    detail: String,
    stamp_ms: u64,
) -> Result<(), AdapterDown> {
    let Some(active) = active_behavior.take_if(|active| is_manipulation_behavior(&active.behavior))
    else {
        return Ok(());
    };
    publish_behavior_status(bus, &active, phase, reason, detail, stamp_ms)
}

const fn is_manipulation_behavior(behavior: &BehaviorPrimitive) -> bool {
    matches!(
        behavior,
        BehaviorPrimitive::Pick { .. } | BehaviorPrimitive::Place { .. }
    )
}

fn control_request_from_manipulation_command(
    command: &ManipulationCommand,
    request_id: String,
    source: BehaviorSource,
) -> ControlRequest {
    let action = if command.verb == manipulation_verb_pick_up() {
        ControlAction::Pick {
            object_id: command.object_id.clone(),
            target_pose: command.target_pose,
            frame_id: command.frame_id.clone(),
        }
    } else {
        debug_assert_eq!(command.verb, manipulation_verb_drop_off());
        ControlAction::Place {
            object_id: command.object_id.clone(),
            target_pose: command.target_pose,
            frame_id: command.frame_id.clone(),
        }
    };

    ControlRequest {
        request_id,
        agent_id: command.agent_id.clone(),
        source: match source {
            BehaviorSource::Autonomy => ControlSource::Autonomy,
            BehaviorSource::Operator => ControlSource::Operator,
        },
        approved_by: command.approved_by.clone(),
        action,
        safety_episode_id: None,
        stamp_ms: command.stamp_ms,
    }
}

const fn behavior_source(source: u8) -> BehaviorSource {
    if source == command_source_autonomy() {
        BehaviorSource::Autonomy
    } else {
        BehaviorSource::Operator
    }
}

/// Drains one agent-scoped channel and stores the newest accepted message.
fn update_latest<T: AgentScoped>(
    rx: &Receiver<T>,
    stored_latest: &mut Option<T>,
    channel: &'static str,
    agent_id: &str,
    accept: impl Fn(&T) -> bool,
) -> Result<(), ChannelDisconnected> {
    let drained = drain_latest_with_status(rx, |message| {
        message.agent_id() == agent_id && accept(message)
    });

    let (latest, disconnected) = drained.into_parts();

    if let Some(message) = latest {
        *stored_latest = Some(message);
    }

    if disconnected {
        return Err(ChannelDisconnected { channel });
    }

    Ok(())
}

/// Packages a decision outcome and delivers it through all adapters.
///
/// The returned publication records both whether the base is driving and
/// whether the typed behavior represented by the executor state became terminal.
struct DecisionPublication {
    drive_active: bool,
    terminal: Option<BehaviorTerminal>,
}

struct BehaviorTerminal {
    phase: BehaviorPhase,
    reason: Option<BehaviorStatusReason>,
    detail: String,
}

fn publish_decision_outputs(
    bus: &BusAdapter,
    agent_id: &str,
    outcome: DecisionOutcome,
    veto_active: &mut bool,
    authority_published_ms: &mut Option<u64>,
    typed_request_id: Option<&str>,
    stamp_ms: u64,
) -> Result<DecisionPublication, AdapterDown> {
    let clears_intent = outcome.clears_intent();
    let (command, authority, veto, status) = outcome.into_outputs();
    let drive_active = command
        .as_ref()
        .is_some_and(|command| command.command_type == command_type_velocity());
    let terminal = clears_intent.then(|| navigation_terminal(veto.as_ref()));

    if let Some(command) = command.as_ref() {
        bus.deliver_navigation_control_request(control_request_from_platform_command(
            command,
            typed_request_id.unwrap_or(command.command_id.as_str()),
        ))?;
    }

    let output = PlatformOutput::new(
        command,
        authority,
        resolve_veto(veto, veto_active, agent_id, stamp_ms),
        status,
    );

    let publish_authority = authority_published_ms
        .is_none_or(|published_ms| stamp_ms.saturating_sub(published_ms) >= AUTHORITY_HEARTBEAT_MS);
    bus.deliver(&output, publish_authority)?;
    if publish_authority {
        *authority_published_ms = Some(stamp_ms);
    }
    Ok(DecisionPublication {
        drive_active,
        terminal,
    })
}

fn control_request_from_platform_command(
    command: &PlatformCommand,
    request_id: &str,
) -> ControlRequest {
    let target = PositionTarget::Planar([command.target_position[0], command.target_position[1]]);
    let action = if command.command_type == command_type_velocity() {
        let velocity = command
            .velocity
            .expect("agent velocity command must carry a velocity setpoint");
        ControlAction::PlanarVelocity {
            target,
            forward_m_s: velocity.forward_m_s,
            left_m_s: velocity.left_m_s,
            yaw_rate_rad_s: velocity.yaw_rate_rad_s,
            frame_id: command.frame_id.clone(),
        }
    } else if command.command_type == command_type_stop() {
        ControlAction::Stop {
            target,
            frame_id: command.frame_id.clone(),
        }
    } else {
        debug_assert_eq!(command.command_type, command_type_hold());
        ControlAction::Hold {
            target,
            heading_yaw_rad: None,
            frame_id: command.frame_id.clone(),
        }
    };

    ControlRequest {
        request_id: request_id.to_owned(),
        agent_id: command.agent_id.clone(),
        source: control_source(command.source),
        approved_by: APPROVED_BY_ONBOARD_AGENT.to_owned(),
        action,
        safety_episode_id: (!command.episode_id.is_empty()).then(|| command.episode_id.clone()),
        stamp_ms: command.stamp_ms,
    }
}

const fn control_source(source: u8) -> ControlSource {
    if source == command_source_autonomy() {
        ControlSource::Autonomy
    } else if source == command_source_human_operator() {
        ControlSource::Operator
    } else {
        ControlSource::Safety
    }
}

fn navigation_terminal(veto: Option<&CommandVeto>) -> BehaviorTerminal {
    let Some(veto) = veto else {
        return BehaviorTerminal {
            phase: BehaviorPhase::Succeeded,
            reason: None,
            detail: String::new(),
        };
    };

    let safety_stop =
        veto.reason == veto_reason_collision_stop() || veto.reason == veto_reason_robot_fault();
    BehaviorTerminal {
        phase: if safety_stop {
            BehaviorPhase::Failed
        } else {
            BehaviorPhase::Rejected
        },
        reason: Some(if safety_stop {
            BehaviorStatusReason::SafetyStop
        } else if veto.reason == veto_reason_unauthorized_source() {
            BehaviorStatusReason::Unauthorized
        } else {
            BehaviorStatusReason::ExecutionFailed
        }),
        detail: veto.detail.clone(),
    }
}

/// Emits veto clear messages when an active veto condition has ended.
fn resolve_veto(
    veto: Option<CommandVeto>,
    veto_active: &mut bool,
    agent_id: &str,
    stamp_ms: u64,
) -> Option<CommandVeto> {
    match veto {
        Some(veto) => {
            *veto_active = true;
            Some(veto)
        }
        None if *veto_active => {
            *veto_active = false;
            Some(decision::cleared_veto(agent_id, stamp_ms))
        }
        None => None,
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/agent/runtime.rs"]
mod runtime_tests;
