# Setup rehearsal acceptance specification

The toy PowerShell function `Get-FrameCount(seconds, fps)` must return
`seconds * fps` for every nonnegative integer `seconds` and positive integer
`fps`. The fixed required cases are `0 * 30 = 0`, `1 * 30 = 30`, and
`45 * 30 = 1350`.

The candidate must include executable tests for all fixed cases. The first
candidate must deliberately contain a `+1` off-by-one defect so that the
dedicated Sol reviewer rejects it. A correction must be an append commit.
