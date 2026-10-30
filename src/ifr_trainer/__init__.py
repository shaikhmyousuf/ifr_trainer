def main() -> None:
    import os

    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    from .app import main as app_main

    app_main()
