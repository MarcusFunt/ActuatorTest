# Actuator Bench Tool — repository analysis

**Assessment date:** 2026-08-16  
**Revision reviewed:** `bb5a375` on `master` (merged guided-characterization workflow)  
**Scope:** source tree, Git history, GitHub repository/PR state, CI configuration,
documentation, checked-in runtime artefacts, and local execution readiness. This is
an engineering review, not a hardware safety certification.

## Executive assessment

This is a promising, unusually complete bench-tool prototype. It has a coherent
engineering purpose: a Reflex/Bokeh operator UI controls a serial actuator, records
telemetry, calibrates it, identifies a reduced-order physical plant, and validates a
two-inertia digital twin. The project has moved rapidly from a basic GUI to a
protocol-aware, simulator-backed calibration and characterization system.

The immediate priority is not another large capability. First make the repository
clean and reproducible, then make the physical-control path resilient to failures and
observable in CI. Those steps reduce the risk that a software or configuration change
creates unsafe bench behavior, while preserving the good identification work already
present.

**Overall readiness:** suitable for continued supervised bench development with the
simulator and explicit operator precautions; not yet demonstrated as a reproducible,
release-ready tool for unattended or broadly distributed hardware use.

## Evidence and limitations

| Area | Evidence | Result |
| --- | --- | --- |
| Repository history | 28 commits, 2026-05-28 to 2026-08-16; 5 refs/branches | Small, fast-moving project. |
| Ownership | 27 non-merge commits, all by `MarcusFunt` | Bus factor of one. |
| Recent delivery | PRs #1–#3 are merged; #3 brought guided characterization to `master` | Clear feature momentum. |
| Hosted CI | GitHub Actions `Python tests` runs #9 and #11 succeeded for PR #3 head `6211985` | Latest feature branch had green Python CI. |
| Local test execution | Local interpreter is Python 3.10.12 and has no pytest; project requires Python >=3.11 | I could not run the test suite locally. |
| Static syntax check | `python3 -m compileall -q src actuator_gui run_app.py e2e_test_com5.py` | Passed, but with Python 3.10 only; this is not a substitute for the required test run. |
| Browser check | The Chrome debugging target closed before page enumeration | No fresh visual/runtime browser validation was possible. |
| Remote desktop check | No connected Remote Desktop Commander device | No remote-device validation was possible. |

The browser artefacts committed from 2026-05 record a working simulator UI followed
by Bokeh WebSocket expiry/connection errors. They are useful historical clues, not
proof that current `master` still fails.

## What the project does

```text
Reflex operator UI ──┐
                    ├─ ActuatorClient ─ binary protocol ─ firmware on ESP/TMC2209
Bokeh telemetry UI ─┘          │
                               ├─ SimulatedTransport for software tests
                               └─ raw telemetry/event/session report artefacts

Measurements ──> calibration + identification ──> plant.json
                                                   ├─ two-inertia simulator
                                                   ├─ validation metrics
                                                   └─ MuJoCo / Project Chrono adapters
```

### Main layers

1. **Operator application.** `actuator_gui/actuator_gui.py` is the main Reflex
   application, connection manager, calibration/test launcher, report surface and
   visual dashboard. `characterize_page.py` adds the 10-stage guided physical-plant
   workflow. `bokeh_charts.py` supplies a separate Bokeh telemetry server.
2. **Host control and protocol.** `actuator_protocol.py` defines a framed,
   CRC-16/CCITT-FALSE binary protocol with parsing/recovery. `actuator_serial.py`
   contains real serial transport, a threaded client and a protocol-level simulator.
3. **Analysis and data.** `actuator_data.py`, `actuator_analysis.py`, and
   `actuator_tests.py` maintain telemetry and provide calibration, resonance,
   backlash, step, velocity-ramp and compliance routines.
4. **Physical identification.** The `plant_*` modules hold a versioned parameter
   schema, fitting functions, a two-inertia model, telemetry derivations, validation,
   and external-simulator adapters. This separation from the UI is a strong design
   decision.
5. **Firmware.** `firmware/ActuatorFirmware/ActuatorFirmware.ino` implements the
   protocol, encoder and driver interactions, safety faults, position/velocity/
   torque-proxy modes, current scheduling and persistence.

## Strengths worth preserving

- The product scope is unusually well stated. The README distinguishes the bench
  tool from out-of-scope ROS/MCAP/flashing work, and the plant-identification document
  is clear about a reduced-order model rather than an overclaimed belt simulation.
- Measurement semantics are careful. The guided workflow and docs explicitly reject
  electrical input power as a shaft-torque measurement, call for a hold-out trace,
  and describe the current validation as mechanical replay. That is excellent
  engineering communication.
- The protocol has explicit frame size bounds, CRC validation, sequence matching,
  parser statistics and recovery tests. Host/firmware default contracts have targeted
  tests in `tests/test_protocol.py`.
- There is real simulator-backed behavior coverage, not only pure-math tests. The
  suite contains 79 test functions across protocol, telemetry, calibration,
  identification, digital-twin, GUI-page construction and simulator-control flows.
- Safety mechanisms exist on both sides of the boundary: disabled startup,
  stop/ESTOP commands, firmware fault flags, limits, missed-step handling and
  torque-proxy timeout/excursion checks. This is a good foundation, not a reason to
  skip hardware validation.
- Session output is already useful for traceability: raw CSV, events, calibration,
  plots, actuator metadata, notes and summaries are captured in a timestamped folder.

## Findings and recommendations

Priorities below mean: **P0** stop accidental breakage before further work; **P1**
reduce material safety/reliability exposure in the next milestone; **P2** planned
hardening and maintainability work.

### P0 — repository hygiene and reproducibility

1. **The working tree is contaminated by a broad line-ending-only rewrite.**
   `git diff --numstat` reports 16,623 added and 16,602 removed lines across 49
   tracked files, including every major source, test, firmware and documentation file.
   Most changes are CRLF conversion, not meaningful edits. `git diff --check` also
   reports trailing whitespace on every converted line.

   **Why it matters:** this masks real changes, makes review unreliable, can cause
   merge conflict churn, and risks committing unrelated artefacts.

   **Action:** before the next feature commit, split or discard the line-ending-only
   change deliberately, add `.gitattributes` with a repository EOL policy, and verify
   a clean diff. Do not mix this cleanup with functional changes.

2. **Generated and stale artefacts are tracked.** The repository tracks launch logs
   (`launch_*.txt`) and `.playwright-cli` snapshots/logs/screenshots. The current
   ignore file excludes `reports/`, caches and `.venv`, but not those artefacts or
   `.ui-test-venv`.

   **Action:** remove generated diagnostic artefacts from version control in a
   separate cleanup commit, expand `.gitignore`, and retain reproducible test scripts
   rather than one-off output. Keep an intentionally curated screenshot only when it
   documents a release, and place it under `docs/` with a purpose.

3. **Local developer readiness is not reproducible here.** The launcher correctly
   requires Python 3.11, but this workspace has only Python 3.10 and no `pytest`.
   Dependencies are neither locked nor constraints-pinned: the project declares broad
   lower bounds for Reflex and Bokeh and unconstrained scientific dependencies.

   **Action:** create a documented Python 3.11 environment, add a lock/constraints
   strategy appropriate to the project (for example `uv.lock`, pip-tools constraints,
   or a tested requirements lock), and add a `make`/task entry point for install, test,
   lint and firmware build.

### P1 — control-path reliability and safety assurance

4. **A telemetry callback can terminate the only reader thread.**
   `ActuatorClient._handle_frame()` directly invokes every telemetry/event callback
   without containment (`src/actuator_tool/actuator_serial.py:1156-1169`). If report
   recording, UI forwarding, or any future callback raises, the reader thread exits;
   subsequent command responses will no longer arrive and commands time out.

   **Action:** isolate each callback with narrow exception handling, record an
   observable callback error/counter, and test that one failing callback does not stop
   telemetry or command/response traffic. A callback should not be allowed to break
   the actuator communications boundary.

5. **Hardware commands share mutable global state and a two-worker executor.**
   The UI module has one global `_ctx` and `ThreadPoolExecutor(max_workers=2)` while
   actions can call the same `ActuatorClient`. A future UI race or overlapping
   background event can interleave control sequences and report/session state.

   **Action:** introduce one command dispatcher/lock per actuator connection, a
   state machine with explicit `idle / connecting / testing / faulted / disconnecting`
   transitions, cancellation and timeouts, and a single owner for lifecycle cleanup.
   Test connect/disconnect, ESTOP, fault during a test, and two attempted concurrent
   commands. The UI should make busy/fault state impossible to bypass, not merely
   inconvenient.

6. **Configuration writes are many independent commands.** The UI loops over values
   then calls `SAVE_CONFIG` (`actuator_gui/actuator_gui.py:578-621`). A serial error
   can leave a live device partially reconfigured even if persistence never happens.

   **Action:** add a versioned, validated configuration transaction (stage → validate
   → apply/commit, with previous-config rollback where feasible), or at minimum read
   back and compare every persisted value before declaring success. Include firmware
   schema/version and a configuration checksum in the report.

7. **The Bokeh connection has an observed long-session weakness.** Current code sets
   `session_token_expiration=3600` (`actuator_gui/bokeh_charts.py:488-525`). Committed
   historical logs show `Token is expired` and lost WebSockets. The iframe can thus
   degrade during a long bench session without a clean operator recovery path.

   **Action:** reproduce this with an automated long-session/reconnect test, then
   either implement a token/session refresh path or show a prominent reconnectable
   chart state. Log the condition as an application event, not only a server warning.

8. **The web configuration is broader than a local bench tool needs.**
   `rxconfig.py:36` uses `cors_allowed_origins=["*"]`; Bokeh also maintains a growing
   hard-coded list of local ports. This is not necessarily exploitable while services
   bind only to loopback, but it becomes an avoidable exposure if hosts are changed by
   environment variables.

   **Action:** default both services to loopback, use an explicit development-origin
   allow-list, reject non-local binding unless an explicit production configuration is
   supplied, and document the threat model. Add an integration assertion for origin
   policy and host binding.

9. **Firmware is tested only indirectly and is not built in CI.** There is no
   PlatformIO/Arduino build configuration or hardware-in-the-loop workflow. Python
   tests parse a few firmware constants, which catches selected contract drift but not
   compilation, board-library compatibility, flash layout, timing regressions or
   real-device safety behavior.

   **Action:** add a reproducible firmware build target pinned to board/core/library
   versions; run it in CI. Establish a bench HIL smoke checklist with an interlocked,
   unloaded fixture: boot disabled, protocol round trip, ESTOP, fault/reset,
   limit enforcement and telemetry-schema compatibility. Make HIL a manually
   dispatched, evidence-producing workflow rather than an uncontrolled CI action.

### P2 — quality, architecture and product hardening

10. **One GUI module carries too much responsibility.**
    `actuator_gui/actuator_gui.py` is 2,787 lines and mixes state, styling, component
    construction, connection lifecycle, hardware actions, report handling and
    plotting integration. It is the leading change hotspot (nine historical commits)
    and is associated with five fix/bug commits.

    **Action:** do a behavior-preserving extraction: first move transport/session
    orchestration to a tested service, then separate page components, control forms,
    report actions and styling. Avoid a wholesale rewrite. Keep pure functions in
    `src/actuator_tool` and require tests for every extracted boundary.

11. **The test suite is valuable but CI’s quality gates are minimal.** CI currently
    installs editable dependencies and runs `pytest -q` only
    (`.github/workflows/python-tests.yml:1-21`). There is no formatter/linter, type
    check, coverage threshold, dependency audit, package build, firmware build or UI
    smoke test. Report writing and the full main GUI interaction path lack focused
    tests; the test reference to `actuator_gui.py` is largely import/page coverage.

    **Action:** add Ruff (format + lint), a pragmatic Pyright/mypy boundary, coverage
    reporting without an arbitrary initial threshold, `python -m build`, and a
    simulator-driven app smoke test. Add test cases for report provenance, callback
    failure, partial configuration failure, reconnect, fault during long test, and
    all guided-workflow precondition failures.

12. **Reports need stronger provenance for engineering reuse.** Session artefacts
    have raw data and events, but `SessionArtifacts` does not record the Git revision,
    package/dependency versions, operating-system/port/baud information, complete
    safety limits, firmware build ID or a calibration/config hash.

    **Action:** write immutable `run_manifest.json` at session start and add its hash
    to summaries. It should include software revision, dependency lock hash, host
    configuration, firmware identifier, actuator identity, calibration schema/version,
    safety settings and test-plan/operator inputs. This turns `plant.json` from a
    useful file into an auditable engineering artefact.

13. **Cross-platform claims have a Windows-only escape hatch.** The README supports
    macOS/Linux, but `open_folder()` launches `explorer` unconditionally
    (`actuator_gui/actuator_gui.py:1753-1757`).

    **Action:** use platform-specific open commands or offer the folder path and a
    downloadable archive. Add small tests around platform command selection.

14. **Public-project governance is absent.** The public GitHub repository has no
    LICENSE, CONTRIBUTING, SECURITY policy, release notes or issue labels/milestones.
    There are no repository issues apparent in the GitHub context; the work is
    represented as merged PRs.

    **Action:** decide whether this is private bench infrastructure or a reusable
    public tool. If public, add a licence, contribution/safety statement, security
    contact, issue templates and a short release process. Track the roadmap below as
    issues instead of relying on commit messages.

## Recommended delivery sequence

### Milestone 0 — establish a trustworthy baseline (1–2 focused changes)

1. Preserve any intended local work, then restore a clean, reviewable worktree.
2. Add `.gitattributes` and ignore rules; remove tracked logs and old automation
   output in their own commit.
3. Create the Python 3.11 environment and a dependency lock/constraints file.
4. Run the full suite locally and record its exact result. Confirm the post-merge
   `master` GitHub Actions run, not only the PR-head run.

**Exit criteria:** `git status` is clean after normal development actions; a fresh
Python 3.11 checkout can install and run all tests with pinned inputs.

### Milestone 1 — make a failed component fail safely (next engineering priority)

1. Serialize command ownership and model lifecycle/fault state explicitly.
2. Contain telemetry/event callback failures and expose transport health in the UI.
3. Make configuration persistence verified/atomic enough for the firmware contract.
4. Add regression tests for the failure paths and an operator-visible recovery flow.

**Exit criteria:** injected callback errors, connection loss, an ESTOP, a device fault,
and partial config write all lead to a clear safe state without a dead reader thread or
ambiguous UI status.

### Milestone 2 — build-and-test automation (parallel only after Milestone 0)

1. Expand CI to format/lint, test, package build, dependency audit and firmware build.
2. Add simulator UI smoke coverage and a Bokeh reconnect test.
3. Add manual HIL workflow with captured test artefacts and a fixed safe fixture.

**Exit criteria:** every PR proves host package integrity and firmware compilation;
hardware validation is reproducible and cannot move an unprotected mechanism.

### Milestone 3 — improve maintainability without destabilising behavior

1. Extract host session/controller orchestration from the GUI.
2. Split the main page into view components and bind them to a small state interface.
3. Add a run manifest and cross-platform report-folder experience.
4. Establish release notes, configuration/schema compatibility policy and project
   governance appropriate to the intended audience.

**Exit criteria:** the main UI module is no longer the control-plane monolith, and a
characterization result is traceable to exact software, firmware and settings.

## Suggested issue backlog

| Priority | Issue | Acceptance signal |
| --- | --- | --- |
| P0 | Normalize EOLs; remove tracked runtime artefacts | A clean diff shows only intended edits; generated paths remain untracked. |
| P0 | Add Python 3.11 lock/constraints and developer task entry point | Fresh environment passes installation and tests. |
| P1 | Prevent callback exceptions from killing `ActuatorClient` reader | Regression test proves commands and telemetry continue after an injected callback error. |
| P1 | Serialize actuator commands and formalize lifecycle state | Race/fault/disconnect tests pass; UI exposes state and recovery. |
| P1 | Transactional/verified configuration persistence | Device readback matches a complete expected config or an error leaves a safe known state. |
| P1 | Add reproducible firmware build plus manual HIL evidence | CI builds firmware; manual job captures protocol/ESTOP/fault evidence. |
| P1 | Harden local origin/binding policy and Bokeh reconnect | Origins are explicit; chart reconnection is tested and visible. |
| P2 | Split GUI control-plane responsibilities | New tested controller/service boundary; no feature behavior regression. |
| P2 | Add report provenance manifest | Every report session embeds revisions, settings and hashes. |
| P2 | Add governance/release fundamentals | Licence and safety scope are explicit; milestone issues track future work. |

## Final perspective

The identification and simulation work is the project’s differentiator. The best next
move is to protect that work with reproducible environments, rigorous control-path
failure handling and verifiable firmware/HIL gates. Once those foundations are in
place, the guided workflow can evolve confidently from a capable personal bench tool
into a dependable engineering instrument.
