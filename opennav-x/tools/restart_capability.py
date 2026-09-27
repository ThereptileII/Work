"""Validate executed app/helper capability reports before package attestation."""


def verified_restart_protocol(application, helper):
    """Return only the common, explicitly implemented native guard protocol."""
    if not isinstance(application, dict) or not isinstance(helper, dict):
        raise ValueError("Native restart capability reports must be objects")
    app_protocol = application.get("commissioning_restart_protocol")
    helper_protocol = helper.get("commissioning_restart_protocol")
    if (type(app_protocol) is not int or type(helper_protocol) is not int or
            app_protocol != 1 or helper_protocol != app_protocol):
        raise ValueError("App and helper must attest the same supported restart protocol")
    if (set(helper) != {"contract", "role", "commissioning_restart_protocol",
                        "profile_accessed", "child_started"} or
            helper["contract"] != "OpenNavX.RestartCapability.1" or
            helper["role"] != "restart-helper" or
            helper["profile_accessed"] is not False or
            helper["child_started"] is not False):
        raise ValueError("Native helper capability query was not side-effect-free")
    if (application.get("contract") != "OpenNavX.LoaderSelfTest.1" or
            application.get("passed") is not True or
            application.get("profile_initialized") is not False or
            application.get("plugins_loaded") is not False):
        raise ValueError("Native application capability requires its real loader self-test")
    return app_protocol
