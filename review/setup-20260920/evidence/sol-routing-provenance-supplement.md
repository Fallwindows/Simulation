# Sol routing provenance supplement

This packet supplements earlier setup and r1 verdict records that accurately reported runtime identity as unknown at their issuance time.

The host later exposed safe selected `turn_context` fields in `runs/setup-20260920/routing-observed.json`, SHA-256 `3e4fe724a4ee5078de5722f919d91ccba782d8ba10b0d194eca9d1fa3a6a7a5b`.

For reviewer session `rollout-2026-09-20T22-21-51-01a0c269-d5ca-7a20-ab83-97b8d063651b` / thread `01a0c269-d5ca-7a20-ab83-97b8d063651b`, the host-selected runtime fields are:

- model: `gpt-5.6-sol`
- effort: `high`
- approval policy: `never`
- sandbox policy: `danger-full-access`

This confirms the intended Sol/high routing and the previously observed permission override. It is host runtime-selected identity evidence, not a provider-side attestation. Subsequent verdicts in this session may record the model and effort as observed from this evidence.
