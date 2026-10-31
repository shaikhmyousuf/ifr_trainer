"""Aircraft modules. Each one is a package here whose ``build()`` returns an ``Aircraft``
(see ``ifr_trainer/sim/aircraft.py``). To add an aircraft, create its package and list it below.
"""

import importlib

from ..sim.aircraft import Aircraft

AIRCRAFT = {
    "a320": "ifr_trainer.aircraft.a320",
    "c172": "ifr_trainer.aircraft.c172",
    "citation": "ifr_trainer.aircraft.citation",
}
DEFAULT_AIRCRAFT = "a320"


def load_aircraft(name: str) -> Aircraft:
    if name not in AIRCRAFT:
        raise KeyError(f"unknown aircraft {name!r}; available: {', '.join(AIRCRAFT)}")
    return importlib.import_module(AIRCRAFT[name]).build()
