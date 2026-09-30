# RF Range Monitor

Tkinter desktop tools for Meshtastic radio telemetry, NanoVNA/tinySA readings,
RF coverage visualization, and terrain profiling.

## Project structure

```text
meshtatsicRangeTesting/
├── README.md
├── pyproject.toml               # Build metadata, dependencies, and tool configuration
├── main.py                       # Optional script launcher for the current app
├── .gitignore                    # Generated files and local environments
├── src/
│   ├── radio_range_monitor/      # Current RF coverage application package
│   │   ├── __init__.py
│   │   ├── __main__.py           # Supports: python -m radio_range_monitor
│   │   ├── dashboard.py          # Tkinter UI and event handlers
│   │   ├── coverage_model.py     # RF calculations, parsers, and device workers
│   │   └── legacy/               # Earlier CLI-based monitor kept for compatibility
│   │       ├── __init__.py
│   │       ├── __main__.py       # Supports: python -m radio_range_monitor.legacy
│   │       ├── dashboard.py      # Legacy dashboard, map, and RF planner UI
│   │       └── backend.py        # Legacy hardware, telemetry, and RF utilities
└── tests/                        # Unit tests for RF calculations and parsers
```

The project supports Python 3.10 and newer. Package metadata and runtime
dependencies are maintained in `pyproject.toml`.

## Run

Install the application and its dependencies from the project root:

```powershell
python -m pip install .
```

Launch the all-in-one RF monitor. The interactive map and terrain cross-section
are shown together on the Map & Terrain tab; two map clicks choose profile
endpoints and request the terrain profile.

```powershell
python -m radio_range_monitor
```

The map initially opens centered on the contiguous United States. The Home Base
marker still uses `HOME_LAT` and `HOME_LON` environment variables when set. On
the Map & Terrain tab, click once to choose the profile start point and click
again to choose the end point; the app draws the link and automatically requests
the terrain profile between them. Right-click map options and coordinate fields
remain available for setting Home Base and the virtual target manually.

`python main.py` is also available as a script launcher.

## Development

Install the project and its lint/format tooling:

```powershell
python -m pip install -e ".[dev]"
```

Run checks:

```powershell
python -m unittest discover -s tests -v
ruff check .
ruff format --check .
```

Apply formatting with `ruff format .`.
