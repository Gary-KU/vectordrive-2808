"""Run with python -m desktop. The default UI starts disconnected."""
def main():
    from .instance import acquire_gui_instance, restore_existing_gui
    instance = acquire_gui_instance()
    if instance is None:
        restore_existing_gui()
        return
    from .ui import run_gui
    try:
        run_gui()
    finally:
        instance.close()


if __name__ == "__main__":
    main()
