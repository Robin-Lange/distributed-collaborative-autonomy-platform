//! Shared operator command language and intent construction.
//!
//! This module is intentionally UI-neutral: it turns plain-text operator
//! commands like `g 1.0 2.0`, `m 3.0 4.0`, or `h` into strongly typed commands,
//! then converts those commands into the `BehaviorRequest` consumed by
//! the autonomy stack. Every operator surface uses this same command
//! vocabulary.
//!
//! Command arguments use canonical `X Y` order in the `map` frame. The map is
//! ENU: `+X` is east/right, `+Y` is north/forward, and `+Z` is up.

use crate::core_msgs::{
    BehaviorPrimitive, BehaviorRequest, BehaviorSource, PositionTarget, catalog::ObjectCatalog,
    frames::MAP_FRAME,
};

/// Object id used by `p`/`d` when the operator omits one — the virtual demo
/// object (the catalog's `cylinder_pink_01`), so a bare `p` always
/// picks up something manipulable with no catalog running (sim/hybrid,
/// before any `object_tracker` session exists). Real, MOCAP-tracked objects
/// are opt-in via an explicit token (see [`resolve_object_token`]).
pub const DEFAULT_OBJECT_ID: &str = "cylinder_pink_01";

/// Commands supported by operator-facing text inputs.
///
/// Operator surfaces accept a compact command language:
/// - `g X Y`: go to the map-frame point `(X, Y)`.
/// - `m X Y`: move away from that map-frame point.
/// - `h`: hold the current position.
/// - `p [OBJECT_ID|MOCAP_ID]`: pick up the named object ([`DEFAULT_OBJECT_ID`],
///   the virtual demo object, if omitted), or the real, MOCAP-tracked object
///   at the given bare MOCAP Streaming ID (`p 2`) — see
///   [`resolve_object_token`].
/// - `d`: drop off whatever object this agent currently holds.
/// - `r`: explicitly resume autonomy after an operator override.
///
/// The enum keeps parsed commands type-safe so UI code does not need to keep
/// working with raw strings. `PickUp`/`DropOff` carry the operator's raw
/// token unresolved; callers apply [`resolve_object_ids`] against the
/// catalog before building a wire intent.
#[derive(Clone)]
pub enum OperatorCommand {
    /// Drive toward the requested map-frame `(x, y)` waypoint.
    GoTo(f32, f32),

    /// Move away from the requested map-frame `(x, y)` point.
    MoveAway(f32, f32),

    /// Stay at the current position.
    Hold,

    /// Pick up the named catalog object (`None` resolves to
    /// [`DEFAULT_OBJECT_ID`]).
    PickUp(Option<String>),

    /// Drop off an object (`None` resolves to whatever this agent is
    /// observed to currently hold, via the UI's own `ObjectState` tracking;
    /// a UI surface with no such tracking should require an explicit id).
    DropOff(Option<String>),

    /// Release the operator override latch for the selected agent.
    ResumeAutonomy,
}

/// Human-readable parse error returned when the operator enters invalid input.
///
/// This wrapper keeps parsing failures explicit without exposing the internal
/// string storage directly.
#[derive(Debug)]
pub struct ParseError(String);

impl ParseError {
    /// Returns the message that should be displayed back to the operator.
    pub(crate) fn message(&self) -> &str {
        &self.0
    }
}

/// Parses one line of operator input into a console command.
///
/// Empty or whitespace-only input is not treated as an error; it returns
/// `Ok(None)` so callers can simply ignore blank lines. Invalid commands return
/// `Err(ParseError)` with a message suitable for displaying in the CLI.
pub fn parse_line(line: &str) -> Result<Option<OperatorCommand>, ParseError> {
    let mut tokens = line.split_whitespace();

    let Some(raw_verb) = tokens.next() else {
        return Ok(None);
    };

    let verb = raw_verb.to_ascii_lowercase();

    match verb.as_str() {
        "g" | "m" => {
            let x = parse_float(tokens.next(), &verb)?;
            let y = parse_float(tokens.next(), &verb)?;
            reject_extra(&mut tokens, &verb)?;

            if verb == "g" {
                Ok(Some(OperatorCommand::GoTo(x, y)))
            } else {
                Ok(Some(OperatorCommand::MoveAway(x, y)))
            }
        }
        "h" => {
            reject_extra(&mut tokens, &verb)?;
            Ok(Some(OperatorCommand::Hold))
        }
        "p" => {
            let object_id = tokens.next().map(str::to_owned);
            reject_extra(&mut tokens, &verb)?;
            Ok(Some(OperatorCommand::PickUp(object_id)))
        }
        "d" => {
            let object_id = tokens.next().map(str::to_owned);
            reject_extra(&mut tokens, &verb)?;
            Ok(Some(OperatorCommand::DropOff(object_id)))
        }
        "r" => {
            reject_extra(&mut tokens, &verb)?;
            Ok(Some(OperatorCommand::ResumeAutonomy))
        }
        other => Err(ParseError(format!(
            "unknown command `{other}`; use `g X Y`, `m X Y`, `h`, \
             `p [OBJECT_ID|MOCAP_ID]`, `d`, or `r`"
        ))),
    }
}

/// Parses one required floating-point argument.
///
/// `token` is optional because the caller may have reached the end of the
/// command line early. Missing values and malformed values are reported with
/// different errors so the operator gets a useful correction.
fn parse_float(token: Option<&str>, verb: &str) -> Result<f32, ParseError> {
    let token =
        token.ok_or_else(|| ParseError(format!("`{verb}` needs two numbers: `{verb} X Y`")))?;

    token
        .parse::<f32>()
        .map_err(|_| ParseError(format!("`{token}` is not a number")))
}

/// Rejects extra input after a valid command shape has already been parsed.
///
/// This prevents ambiguous commands like `g 1 2 3` from being silently accepted
/// and makes operator behavior easier to debug.
fn reject_extra(tokens: &mut std::str::SplitWhitespace<'_>, verb: &str) -> Result<(), ParseError> {
    if tokens.next().is_some() {
        return Err(ParseError(format!("`{verb}` got too many arguments")));
    }

    Ok(())
}

/// Resolves one `p`/`d` object token against the catalog.
///
/// The operator can name an object two ways: the catalog `object_id`
/// directly (`p block_mocap_01`), or the bare NatNet/Motive Streaming ID
/// painted on a real, MOCAP-tracked entry (`p 2`, matching a catalog
/// entry's `mocap_rigid_body_id`). The two are
/// disambiguated by whether the token parses as an integer — no catalog
/// `object_id` in this project is purely numeric, so this is unambiguous
/// without needing a sigil. A numeric token with no matching
/// `mocap_rigid_body_id`, or any non-numeric token, passes through
/// unchanged so a genuinely unknown id still reaches the onboard agent's
/// existing `UNKNOWN_OBJECT` rejection with the operator's own text intact.
pub fn resolve_object_token(token: &str, catalog: &ObjectCatalog) -> String {
    token
        .parse::<i32>()
        .ok()
        .and_then(|mocap_id| {
            catalog
                .iter()
                .find(|spec| spec.mocap_rigid_body_id() == Some(mocap_id))
        })
        .map_or_else(|| token.to_owned(), |spec| spec.object_id().to_owned())
}

/// Applies [`resolve_object_token`] to a parsed command's object id, if it
/// carries one. `PickUp`/`DropOff` with no token (the default-object /
/// currently-held cases) pass through untouched; every other command is
/// returned as-is. Callers should run this once, immediately after
/// [`parse_line`], before the command reaches [`behavior_request_for_command`]
/// or any object-pose lookup — the resolved `object_id` is what goes out on
/// the wire.
pub fn resolve_object_ids(command: OperatorCommand, catalog: &ObjectCatalog) -> OperatorCommand {
    match command {
        OperatorCommand::PickUp(object_id) => {
            OperatorCommand::PickUp(object_id.map(|token| resolve_object_token(&token, catalog)))
        }
        OperatorCommand::DropOff(object_id) => {
            OperatorCommand::DropOff(object_id.map(|token| resolve_object_token(&token, catalog)))
        }
        OperatorCommand::ResumeAutonomy => OperatorCommand::ResumeAutonomy,
        other => other,
    }
}

/// Converts one parsed operator command into one typed behavior request.
///
/// The operator's canonical `(x, y)` input becomes the planar map target
/// `[x, y]`. Pick and place remain single primitives; this function never
/// inserts an approach behavior.
pub fn behavior_request_for_command(
    command: &OperatorCommand,
    agent_id: &str,
    held_object_id: Option<&str>,
    stamp_ms: u64,
) -> Option<BehaviorRequest> {
    let behavior = match command {
        OperatorCommand::GoTo(x, y) => BehaviorPrimitive::NavigateTo {
            target: PositionTarget::Planar([*x, *y]),
            heading_yaw_rad: None,
        },
        OperatorCommand::MoveAway(x, y) => BehaviorPrimitive::MoveAway {
            from: PositionTarget::Planar([*x, *y]),
        },
        OperatorCommand::Hold => BehaviorPrimitive::Hold,
        OperatorCommand::PickUp(object_id) => BehaviorPrimitive::Pick {
            object_id: object_id
                .clone()
                .unwrap_or_else(|| DEFAULT_OBJECT_ID.to_owned()),
        },
        OperatorCommand::DropOff(object_id) => BehaviorPrimitive::Place {
            object_id: object_id
                .clone()
                .or_else(|| held_object_id.map(str::to_owned))?,
        },
        OperatorCommand::ResumeAutonomy => return None,
    };

    Some(BehaviorRequest {
        request_id: format!("{agent_id}-operator-{stamp_ms}"),
        agent_id: agent_id.to_owned(),
        source: BehaviorSource::Operator,
        behavior,
        frame_id: MAP_FRAME.to_owned(),
        stamp_ms,
    })
}

#[cfg(test)]
#[path = "../../../../tests/unit/operator_command.rs"]
mod tests;
