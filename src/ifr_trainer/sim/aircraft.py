"""The aircraft interface: what an aircraft module gives the shared simulation.

Shared by every aircraft: the environment (atmosphere, wind, airport, navaids, radio
receivers), the rigid-body physics, the stability-derivative aerodynamic model, trim,
flap and gear movement, and the minimum / go-around / terrain monitoring.

Supplied by each aircraft module under ``ifr_trainer.aircraft``: its data, its engines
(``Propulsion``), its flight control system (``FlightControls``: fly-by-wire, or direct
cables), its cockpit settings, its displays, and its settings for each scenario.
"""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Protocol

from .aero import AeroConfig, AeroModel, Surfaces
from .airframe import Airframe
from .atmosphere import Air
from .cockpit import Cockpit
from .flight_model import AircraftState

if TYPE_CHECKING:
    from ..displays.common import Text
    from ..displays.panel import Panel


class Propulsion(Protocol):
    """Engines, lumped together. The pilot moves one thrust lever, 0 (idle) to 1 (full)."""

    @property
    def thrust_line_z_m(self) -> float:
        """Thrust line below the CG; positive thrust then pitches the nose up."""
        ...

    @property
    def detents(self) -> dict[str, float]:
        """Named lever positions, in order (may be empty). Number keys 1, 2, 3... select them."""
        ...

    def step(self, dt: float, lever: float) -> None: ...

    def set_steady(self, lever: float) -> None:
        """Settle the engines at this lever position (used at a reset)."""
        ...

    def thrust(self, air: Air, mach: float) -> float: ...

    def steady_thrust(self, lever: float, air: Air, mach: float) -> float:
        """Thrust once settled at this lever position (used by trim)."""
        ...


class FlightControls(Protocol):
    """Turns pilot inputs into control surface positions, every step."""

    def update(
        self,
        dt: float,
        inputs: Any,
        s: AircraftState,
        surfaces: Surfaces,
        cfg: AeroConfig,
        mass: float,
        thrust: float,
        cas_kt: float,
        flaps_extended: bool,
    ) -> None: ...


class Aircraft(ABC):
    """An aircraft type. Subclasses live in their own package under ``ifr_trainer.aircraft``."""

    id: str  # short name used on the command line, e.g. "a320"
    title: str
    airframe: Airframe

    @abstractmethod
    def new_propulsion(self) -> Propulsion: ...

    @abstractmethod
    def new_flight_controls(self, aero: AeroModel) -> FlightControls: ...

    @abstractmethod
    def new_cockpit(self, settings: dict) -> Cockpit:
        """Cockpit state from a scenario's ``cockpit`` settings."""

    @abstractmethod
    def new_panel(self, text: "Text") -> "Panel":
        """The aircraft's displays and panel controls."""

    @abstractmethod
    def scenario_settings(self) -> dict[str, dict]:
        """Per-scenario settings (speed, flaps, cockpit...) merged over the shared scenarios.
        Only the scenarios listed here are offered for this aircraft."""
