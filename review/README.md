# Review and evidence index

This directory is the stable entry point for independent review and published
evidence for the 45-second simulation video.

## Current production run

- Branch: `codex/storyboard-video`
- Base: `d5e825c8f6dab77aa6a1007c9731c226b588dfcf`
- Run window: 2026-09-20 23:21:35 PDT through approximately 2026-09-21
  11:21:35 PDT
- Requirements authority: [`../EXECUTION_OVERRIDES.md`](../EXECUTION_OVERRIDES.md)
- Implementation specification:
  [`../SIMULATION_VIDEO_IMPLEMENTATION_SPEC.md`](../SIMULATION_VIDEO_IMPLEMENTATION_SPEC.md)
- Status: implementation in progress; no final gate is claimed.

## Preserved setup evidence

- [Setup report](setup-20260920/report.md)
- [Setup evidence](setup-20260920/evidence/)

The setup report is historical evidence. Its former Astra/Luna assignments are
superseded by `EXECUTION_OVERRIDES.md`. The recorder `libcblas.dll` failure is an
implementation repair under active work and must not be described as passed
until a real capture succeeds.

Candidate commits, independent verdicts, representative frames, timing probes,
and final validation records will be linked here as they are produced.
