# Independent RGB orientation and acceptance re-review

**Disposition: ACCEPTED**

- Candidate commit: `5b2521dcb229f32a98a2de91abc4aff708c88ff3`
- Tree: `42becf3bf9acec772eb4073c17123bde77032025`
- Parent: rejected `0d735571dbf5b9dcfd9b3391e28d1f0ec5aed12e`
- Runtime: GPT-5.6 Sol, high reasoning
- Candidate source was reviewed detached and was not modified, merged, or pushed.

## Verdict

**ACCEPTED.** The prior P1 review-evidence binding defect and P2 operator-documentation defect are closed. No new blocking finding was identified.

## P1 closure: checked-in review evidence is enforced

The sole production entry in `config/presentation/accepted_rgb_captures.json` binds capture `20260921-053814804` to repository path `review/evidence/verdict-capture-20260921-053814804.md` and SHA-256 `bbff6fbf052b70b14d9c1f7b0014fd2788bb92077db3e5f1ad8ba26e37e85c20`.

Independent checks established that:

- the path is tracked at candidate HEAD;
- its working-tree Git blob and `HEAD:` blob are both `2a24d925f640c8d7893fb9b9dfe969a5b9a80cec`;
- its 4,114 bytes hash to the cataloged SHA-256 above;
- those bytes match the original immutable reviewer-scratch verdict bytes.

`_review_evidence_path()` accepts only a canonical slash-separated repository-relative path, resolves it strictly, requires the resolved object to remain under the resolved repository root, and requires a regular file. `validate_rgb_capture_acceptance()` then compares the file bytes with the exact lowercase SHA-256 in the reviewed entry before it accepts classification or transform.

The exact production entry passes. Independent mutations all failed closed:

- forged verdict hash;
- nonexistent verdict;
- `..` traversal;
- absolute drive path;
- backslash form;
- embedded `.` segment;
- directory instead of a file;
- an existing but unrelated repository file;
- empty and non-string paths.

Resolution also rejects a symlink escape because the resolved final path must remain under the resolved repository root. Hard links or duplicate files do not weaken the content binding: the exact cataloged verdict bytes are still required.

Evidence: `rgb-5b2521d-adversarial.json`, SHA-256 `f09e609ae2f1f5532abc11d08968e66bff3a0403dce128e4ab35eba654e84cba`.

## P2 closure: operator documentation matches production state

`simulator/presentation/README.md` now accurately states that:

- the production catalog contains one accepted capture;
- its review record is a checked-in repository-relative verdict with an enforced SHA-256;
- missing, escaping, or mismatched evidence rejects before receipt creation;
- the production capture uses `none` because its FRESH MARKET sign already reads left-to-right;
- the generated fixture retains `hflip` for test coverage;
- receipt transforms are copied from exact catalog entries rather than inferred from labels.

This matches the implementation and checked-in catalog.

## Production orientation and prior transform regression

The exact production capture manifest from `runs/20260921-053814804/capture` passed the candidate RGB emitter against the production catalog. Its emitted role and view receipt bind:

- canonical capture hash `ab84ccc1f71790f896bab1375062ed1db9f40bf9a1531d582f7cd0e589121eba`;
- raw video hash `c453b46e40acd06928b384fd13a5a7822c16f46ca5f30b79fc829d1d8e05c703`;
- catalog hash `195177541735b4b443c0a6e630b1f843b7f48b1106b1e82c797b7c2c19867cf9`;
- exact reviewed-production classification;
- `operation=none`, the exact approved reason, and `raw_capture_bytes_modified=false`.

The emitted scratch RGB bundle hashes to `984020b827cd85ff64990da284fd84e884377e1ab1fa8feeeb9e27a39cf0d00b`. The associated production view receipt hashes to `3fcba5526577a1d36365a4f79e07a6931e5d007755eb3cb0133dbb08bec1f0fe` and preserves the source-video hash exactly.

No transform implementation changed relative to the previously inspected candidate. Regression tests prove that `none` emits no horizontal-flip filter, `hflip` still flips the generated fixture, arbitrary transform/reason values reject, receipt/catalog transform disagreements reject, forged complete mappings reject, and label changes cannot select a transform or classification.

## Verification

- Exact commit, tree, and parent matched the assignment.
- Focused presentation suite: **24/24 passed**.
- Full native Pixi discovery: **157/157 passed, zero skips**, including both live cross-process Zenoh transport tests.
- Python `compileall`: passed.
- `git diff --check`: passed.
- Detached worktree status: clean.
- Diff scope from the rejected parent is limited to the evidence files and attributes, catalog path, validator, README, fixture evidence path, and focused tests; renderer/filter behavior is unchanged.

No GPU or Isaac execution was used.
