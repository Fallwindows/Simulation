# Asimov 1 + OrcaHand V1 feasibility spike

This directory is a local, isolated feasibility result. It does not replace or
modify the G02 simulation systems.

The combined model starts from the pinned Asimov 1 URDF and attaches the
complete OrcaHand V1 extended-right subtree at the retained Asimov right wrist.
Both Asimov forearms remain present. Exact upstream revisions, hashes, copied
asset hashes, and preserved notices are in `upstream_manifest.json`.

## Key outputs

- `asimov_orcahand_right.urdf`: portable combined URDF with local mesh paths.
- `derived_usd/asimov_orcahand_right/asimov_orcahand_right.usda`: actual Isaac
  Sim 6.1 importer output and its payload package.
- `evidence/cpu_model_validation.json`: graph, URI, mesh, collision-load, and
  forward-kinematics checks.
- `evidence/isaac_final/isaac_import_articulation.json`: actual Isaac import and
  articulation measurements.
- `evidence/isaac_final/renders/`: three actual diagnostic renders.
- `evidence/isaac_final/motion_frames/`: 24 actual articulated frames at a
  nominal 12 fps.
- `ASIMOV_ORCAHAND_SPIKE_REPORT.md`: feasibility verdict, evidence,
  limitations, and licensing note.

## Reproduce

Checkout the exact public sources without editing them in place:

```powershell
git clone https://github.com/menloresearch/asimov-1 C:\path\to\asimov-1
git -C C:\path\to\asimov-1 checkout ccf5326f0adffd65edf930cbbac052454cdd82be
git clone https://github.com/orcahand/orcahand_description C:\path\to\orcahand_description
git -C C:\path\to\orcahand_description checkout b9b349a21ee0238c62b6cf92ae7597027867adf8
```

Build and run the CPU checks:

```powershell
C:\isaacsim\python.bat robot_spike\build_combined_model.py `
  --asimov-root C:\path\to\asimov-1 `
  --orca-root C:\path\to\orcahand_description `
  --output-root robot_spike
C:\isaacsim\python.bat robot_spike\validate_combined_model.py --root robot_spike
```

After confirming the shared GPU is idle, run Isaac into a fresh output path:

```powershell
C:\isaacsim\python.bat robot_spike\run_isaac_spike.py `
  --root robot_spike `
  --output robot_spike\evidence\isaac_recheck
```

The checked-in USD package is the exact final successful import. A re-import
next to it may receive an importer-generated numeric suffix rather than
overwriting the package.
