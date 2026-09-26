"""Isaac-compatible import, joint-target, and explicit reset seam.

Ordinary motion uses articulation drive targets. The direct pose/velocity
setters appear only in :meth:`ArticulationController.reset`, where restoring a
known simulation state is intentional and documented.
"""

from __future__ import annotations

from pathlib import Path
from typing import Protocol, Sequence

from .model import ModelValidationError, ProductionRobotSpec


class ArticulationBackend(Protocol):
    @property
    def dof_names(self) -> Sequence[str]: ...

    def set_dof_position_targets(self, positions, *, dof_indices) -> None: ...

    def set_dof_positions(self, positions, *, dof_indices) -> None: ...

    def set_dof_velocities(self, velocities, *, dof_indices) -> None: ...

    def set_world_poses(self, positions=None, orientations=None, *, indices=None) -> None: ...

    def set_velocities(
        self, linear_velocities=None, angular_velocities=None, *, indices=None
    ) -> None: ...


class ArticulationController:
    """Name-safe command and reset operations for one imported articulation."""

    def __init__(self, spec: ProductionRobotSpec, articulation: ArticulationBackend):
        self.spec = spec
        self.articulation = articulation
        self._runtime_names = tuple(articulation.dof_names)
        self._runtime_indices = spec.bind_runtime_dofs(self._runtime_names)

    @property
    def runtime_dof_indices(self) -> dict[str, int]:
        return dict(self._runtime_indices)

    def command_joint_positions(self, targets: dict[str, float]) -> tuple[str, ...]:
        """Set drive targets; physics advances joints on later simulation steps."""

        indices, values, names = self.spec.command_plan(self._runtime_names, targets)
        if indices:
            self.articulation.set_dof_position_targets([list(values)], dof_indices=list(indices))
        return names

    def reset(self) -> None:
        """Restore root/joint pose and all velocities to the configured state.

        Reset is the sole API path allowed to teleport articulation state. It
        also seeds drive targets to the reset pose so the first subsequent
        physics step does not pull toward stale pre-reset targets.
        """

        self.spec.bind_runtime_dofs(self._runtime_names)
        reset = self.spec.validate_targets(self.spec.reset_joint_positions)
        names_in_runtime_order = tuple(self._runtime_names)
        positions = [reset[name] for name in names_in_runtime_order]
        indices = list(range(len(names_in_runtime_order)))
        zeros = [0.0] * len(names_in_runtime_order)

        self.articulation.set_world_poses(
            positions=[list(self.spec.root_position_m)],
            orientations=[list(self.spec.root_orientation_wxyz)],
        )
        self.articulation.set_velocities(
            linear_velocities=[[0.0, 0.0, 0.0]],
            angular_velocities=[[0.0, 0.0, 0.0]],
        )
        self.articulation.set_dof_positions([positions], dof_indices=indices)
        self.articulation.set_dof_velocities([zeros], dof_indices=indices)
        self.articulation.set_dof_position_targets([positions], dof_indices=indices)


class IsaacRobotLoader:
    """Delayed-import adapter for Isaac Sim 6.1's experimental articulation API."""

    def __init__(self, spec: ProductionRobotSpec):
        self.spec = spec

    def import_urdf(self, usd_directory: Path) -> Path:
        """Import the non-fixed-base production URDF to a caller-owned path."""

        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

        usd_directory = usd_directory.resolve()
        usd_directory.mkdir(parents=True, exist_ok=True)
        kwargs = dict(self.spec.importer)
        if kwargs.get("fix_base") is not False:
            raise ModelValidationError("refusing to import the biped with a fixed base")
        config = URDFImporterConfig(
            urdf_path=str(self.spec.model.path),
            usd_path=str(usd_directory),
            **kwargs,
        )
        output = Path(URDFImporter(config).import_urdf()).resolve()
        if not output.is_file():
            raise RuntimeError(f"Isaac URDF importer returned missing output: {output}")
        return output

    def controller(self, articulation_root_path: str) -> ArticulationController:
        """Bind a controller after the caller starts physics/timeline."""

        from isaacsim.core.experimental.prims import Articulation

        articulation = Articulation(articulation_root_path)
        return ArticulationController(self.spec, articulation)
