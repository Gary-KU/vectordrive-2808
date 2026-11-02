"""Entry point for PyInstaller; also works as python desktop/launch.py."""
import sys
from pathlib import Path

if not getattr(sys, "frozen", False):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

if __name__ == "__main__":
    if "--self-test" in sys.argv:
        from desktop.self_test import main
        raise SystemExit(main())
    else:
        from desktop.__main__ import main
        main()
