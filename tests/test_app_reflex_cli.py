from actuator_tool import app_reflex


def test_cli_accepts_simulator_target(monkeypatch):
    captured = {}

    def fake_run(cmd, *, cwd, check, env):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["check"] = check
        captured["env"] = env

    monkeypatch.setattr(app_reflex.subprocess, "run", fake_run)

    app_reflex.main(["--sim"])

    assert captured["cmd"][:3] == [app_reflex.sys.executable, "-m", "reflex"]
    assert captured["cwd"] == app_reflex._PROJECT_ROOT
    assert captured["check"] is True
    assert captured["env"]["ACTUATOR_GUI_USE_SIM"] == "1"
    assert "ACTUATOR_GUI_PORT" not in captured["env"]


def test_cli_accepts_hardware_port(monkeypatch):
    captured = {}

    def fake_run(cmd, *, cwd, check, env):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["check"] = check
        captured["env"] = env

    monkeypatch.setattr(app_reflex.subprocess, "run", fake_run)

    app_reflex.main(["--port", "COM5", "--prod"])

    assert captured["cmd"] == [app_reflex.sys.executable, "-m", "reflex", "run", "--env", "prod"]
    assert captured["cwd"] == app_reflex._PROJECT_ROOT
    assert captured["check"] is True
    assert captured["env"]["ACTUATOR_GUI_USE_SIM"] == "0"
    assert captured["env"]["ACTUATOR_GUI_PORT"] == "COM5"
