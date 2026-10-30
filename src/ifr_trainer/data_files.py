"""Loading of the TOML data files shipped in ``ifr_trainer/data``."""

import tomllib
from importlib import resources
from pathlib import Path


def load_toml(name: str, path: Path | None = None) -> dict:
    """Load ``path`` if given, otherwise the bundled data file ``name``."""
    if path is not None:
        with open(path, "rb") as f:
            return tomllib.load(f)
    with resources.files("ifr_trainer").joinpath("data", name).open("rb") as f:
        return tomllib.load(f)
