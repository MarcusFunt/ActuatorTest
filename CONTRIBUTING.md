# Contributing

This is bench-control software. Do not submit changes that can enable motion or
alter persisted limits without simulator coverage and a documented manual HIL
result on an interlocked fixture.

Before opening a change, run `make check`; run `make firmware` for firmware or
protocol changes. Keep generated reports, launch logs, Playwright output and
virtual environments out of commits. Preserve the LF line-ending policy.

Changes to protocol payloads, telemetry schema, calibration schema or firmware
defaults must update their host/firmware contract tests and describe backwards
compatibility in `CHANGELOG.md`.
