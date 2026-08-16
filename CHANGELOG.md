# Changelog

## Unreleased

- Repository reproducibility, control-path resilience, firmware build and report
  provenance hardening are in progress.

## Compatibility policy

- Telemetry and calibration schemas use explicit versions; incompatible changes
  require a migration path or a major version increment.
- Host and firmware protocol changes must retain explicit compatibility tests.
- A release records the Git revision, firmware build environment and dependency
  lock revision used for the verification result.
