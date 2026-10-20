use super::*;

fn arguments(values: &[&str]) -> Vec<OsString> {
    values.iter().map(OsString::from).collect()
}

fn command_names(mode: ValidationMode) -> Vec<&'static str> {
    command_specs(mode).iter().map(|spec| spec.name).collect()
}

#[test]
fn validation_modes_select_the_exact_check_sets() {
    let common = [
        "Locked dependency fetch",
        "Formatting",
        "Compiler check",
        "Clippy",
        "Documentation build",
        "All-target tests",
        "Doctests",
    ];

    let mut full = common.to_vec();
    full.extend(["Release all-target build", "Dependency security audit"]);
    assert_eq!(command_names(ValidationMode::Full), full);

    let mut without_audit = common.to_vec();
    without_audit.push("Release all-target build");
    assert_eq!(command_names(ValidationMode::WithoutAudit), without_audit);

    assert_eq!(
        command_names(ValidationMode::QualityChecksOnly),
        common[..5]
    );
    assert_eq!(command_names(ValidationMode::TestsOnly), common[5..]);
    assert!(command_names(ValidationMode::SourcePolicyOnly).is_empty());
}

#[test]
fn validation_mode_parser_accepts_each_supported_mode() {
    assert_eq!(parse_validation_mode(&[]), Ok(ValidationMode::Full));
    assert_eq!(
        parse_validation_mode(&arguments(&["--without-audit"])),
        Ok(ValidationMode::WithoutAudit)
    );
    assert_eq!(
        parse_validation_mode(&arguments(&["--quality-checks-only"])),
        Ok(ValidationMode::QualityChecksOnly)
    );
    assert_eq!(
        parse_validation_mode(&arguments(&["--tests-only"])),
        Ok(ValidationMode::TestsOnly)
    );
    assert_eq!(
        parse_validation_mode(&arguments(&["--source-policy-only"])),
        Ok(ValidationMode::SourcePolicyOnly)
    );
}

#[test]
fn validation_mode_parser_rejects_unknown_and_additional_options() {
    let Err(unknown) = parse_validation_mode(&arguments(&["--unknown"])) else {
        panic!("an unknown validator option must be rejected");
    };
    assert_eq!(unknown, "Unknown validation option: --unknown");

    let Err(additional) = parse_validation_mode(&arguments(&["--without-audit", "--extra"])) else {
        panic!("an additional validator option must be rejected");
    };
    assert_eq!(
        additional,
        "Unexpected additional validation option: --extra"
    );
}
