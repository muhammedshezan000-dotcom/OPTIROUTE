from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> int:
    project_root = Path(__file__).resolve().parent
    return subprocess.call(
        [
            sys.executable,
            "-m",
            "streamlit",
            "run",
            str(project_root / "dashboard" / "app.py"),
        ],
        cwd=project_root,
    )


if __name__ == "__main__":
    raise SystemExit(main())
