import json

from actuator_tool.actuator_data import ActuatorInfo
from actuator_tool.actuator_report import SessionReport, create_session_folder, write_summary
from actuator_tool.actuator_data import TelemetryStore
from actuator_tool.config_schema import CalibrationConfig, SafetyLimits


def test_session_manifest_captures_connection_and_configuration(tmp_path):
    report = SessionReport(create_session_folder(tmp_path))
    try:
        manifest_hash = report.write_run_manifest(
            connection={"port": "SIM", "simulator": True},
            info=ActuatorInfo(actuator_id="sim-1", firmware_version="test"),
            safety=SafetyLimits(),
            calibration=CalibrationConfig(),
        )
        manifest = json.loads(report.artifacts.run_manifest_json.read_text(encoding="utf-8"))

        assert manifest["manifest_sha256"] == manifest_hash
        assert manifest["connection"]["simulator"] is True
        assert manifest["configuration_sha256"]

        write_summary(
            report.artifacts.summary_txt,
            info=None,
            telemetry_store=TelemetryStore(),
            ratio_fit=None,
            calibration=None,
        )
        assert manifest_hash in report.artifacts.summary_txt.read_text(encoding="utf-8")
    finally:
        report.close()
