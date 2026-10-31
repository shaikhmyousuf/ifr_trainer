"""Loading of the TOML data files shipped inside the ``ifr_trainer`` packages."""

import tomllib
from importlib import resources
from pathlib import Path


def load_toml(name: str, path: Path | None = None, package: str = "ifr_trainer") -> dict:
    """Load ``path`` if given, otherwise the bundled file ``name`` inside ``package``.

    Shared files live in ``ifr_trainer/data`` (``name="data/airport.toml"``); an aircraft
    module passes its own package to load the files next to its code.
    """
    if path is not None:
        with open(path, "rb") as f:
            return tomllib.load(f)
    with resources.files(package).joinpath(name).open("rb") as f:
        return tomllib.load(f)
