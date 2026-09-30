"""Tkinter windows and map widgets for the legacy RF monitor."""

from __future__ import annotations

import io
import math
import os
import queue
import threading
import tkinter as tk
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from tkinter import ttk
from typing import Any
from urllib.request import Request, urlopen

import serial
from PIL import Image, ImageTk
from serial.tools import list_ports

from .backend import (
    EARTH_RADIUS_M,
    TELEMETRY,
    TELEMETRY_LOCK,
    RadioMonitor,
    build_recommendations,
    coordinates_to_world_pixel,
    fetch_elevation_profile,
    free_space_path_loss_db,
    haversine_m,
    terrain_path_analysis,
    world_pixel_to_coordinates,
)


# Main dashboard and its connection controls.
class RadioDashboard:
    """Native Tkinter UI; all widget changes run on Tk's main thread."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.monitor: RadioMonitor | None = None
        self.monitor_error: str | None = None
        self.monitor_ready = threading.Event()
        self.closing = False
        self.metric_values: dict[str, ttk.Label] = {}
        self.port_var = tk.StringVar(value=os.getenv("MESHTASTIC_PORT", ""))
        self.nanovna_port_var = tk.StringVar(value=os.getenv("NANOVNA_PORT", ""))
        self.tinysa_port_var = tk.StringVar(value=os.getenv("TINYSA_PORT", ""))
        self.home_lat_var = tk.StringVar(value=os.getenv("HOME_LAT", "0"))
        self.home_lon_var = tk.StringVar(value=os.getenv("HOME_LON", "0"))
        self.test_lat_var = tk.StringVar(value=os.getenv("HOME_LAT", "0"))
        self.test_lon_var = tk.StringVar(value=os.getenv("HOME_LON", "0"))

        root.title("Radio Performance & Coverage Monitor")
        root.geometry("980x760")
        root.minsize(760, 620)
        self._configure_style()
        self._build_layout()
        self.refresh_serial_ports()
        root.protocol("WM_DELETE_WINDOW", self.close)

        # Device probing can take time; do it off the Tk thread so the window
        # and connection controls appear immediately.
        threading.Thread(
            target=self._initialize_monitor,
            name="monitor-initialization",
            daemon=True,
        ).start()
        self.root.after(100, self._check_initialization)
        self.root.after(500, self.refresh_dashboard)

    def _configure_style(self) -> None:
        style = ttk.Style(self.root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        style.configure("Title.TLabel", font=("Segoe UI", 18, "bold"))
        style.configure("Section.TLabel", font=("Segoe UI", 12, "bold"))
        style.configure("MetricName.TLabel", font=("Segoe UI", 10))
        style.configure("MetricValue.TLabel", font=("Segoe UI", 18, "bold"))
        style.configure("Warning.TLabel", foreground="#9a3412")
        style.configure("Info.TLabel", foreground="#1d4ed8")
        style.configure("Success.TLabel", foreground="#166534")
        style.configure("Error.TLabel", foreground="#b91c1c")

    def _build_layout(self) -> None:
        outer = ttk.Frame(self.root, padding=18)
        outer.pack(fill="both", expand=True)

        heading = ttk.Frame(outer)
        heading.pack(fill="x")
        ttk.Label(
            heading, text="Radio Performance & Coverage Monitor", style="Title.TLabel"
        ).pack(side="left", anchor="w")
        ttk.Button(heading, text="Open RF Planner", command=self.open_rf_planner).pack(
            side="right", padx=6
        )
        ttk.Label(
            outer,
            text="Meshtastic telemetry, antenna SWR, spectrum baseline, and range.",
        ).pack(anchor="w", pady=(2, 14))

        connection = ttk.LabelFrame(
            outer, text="Meshtastic node connection", padding=10
        )
        connection.pack(fill="x", pady=(0, 12))
        ttk.Label(connection, text="Node serial port:").grid(
            row=0, column=0, sticky="w"
        )
        self.port_entry = ttk.Combobox(
            connection,
            textvariable=self.port_var,
            width=24,
            state="normal",
        )
        self.port_entry.grid(row=0, column=1, padx=(8, 8), sticky="ew")
        self.refresh_ports_button = ttk.Button(
            connection,
            text="Refresh ports",
            command=self.refresh_serial_ports,
        )
        self.refresh_ports_button.grid(row=0, column=2, padx=4)
        self.connect_button = ttk.Button(
            connection,
            text="Connect node",
            command=self.connect_node,
            state="disabled",
        )
        self.connect_button.grid(row=0, column=3, padx=4)
        self.disconnect_button = ttk.Button(
            connection,
            text="Disconnect",
            command=self.disconnect_node,
            state="disabled",
        )
        self.disconnect_button.grid(row=0, column=4, padx=4)
        connection.columnconfigure(1, weight=1)
        ttk.Label(
            connection,
            text="Select the Meshtastic node's COM port. The analyzer VID/PID filter "
            "does not identify Meshtastic devices.",
            wraplength=800,
        ).grid(row=1, column=0, columnspan=5, sticky="w", pady=(6, 0))
        self.mesh_status_label = ttk.Label(
            connection, text="Detecting devices...", style="Info.TLabel"
        )
        self.mesh_status_label.grid(
            row=2, column=0, columnspan=5, sticky="w", pady=(8, 0)
        )

        analyzer_frame = ttk.LabelFrame(outer, text="Analyzer serial ports", padding=10)
        analyzer_frame.pack(fill="x", pady=(0, 12))
        ttk.Label(analyzer_frame, text="NanoVNA:").grid(row=0, column=0, sticky="w")
        self.nanovna_port_entry = ttk.Combobox(
            analyzer_frame, textvariable=self.nanovna_port_var, state="normal", width=18
        )
        self.nanovna_port_entry.grid(row=0, column=1, padx=(6, 12), sticky="ew")
        ttk.Button(
            analyzer_frame,
            text="Start NanoVNA",
            command=lambda: self.start_analyzer("nanovna"),
        ).grid(row=0, column=2, padx=4)
        ttk.Label(analyzer_frame, text="tinySA:").grid(row=0, column=3, sticky="w")
        self.tinysa_port_entry = ttk.Combobox(
            analyzer_frame, textvariable=self.tinysa_port_var, state="normal", width=18
        )
        self.tinysa_port_entry.grid(row=0, column=4, padx=(6, 12), sticky="ew")
        ttk.Button(
            analyzer_frame,
            text="Start tinySA",
            command=lambda: self.start_analyzer("tinysa"),
        ).grid(row=0, column=5, padx=4)
        analyzer_frame.columnconfigure(1, weight=1)
        analyzer_frame.columnconfigure(4, weight=1)

        settings = ttk.LabelFrame(outer, text="Location and test position", padding=10)
        settings.pack(fill="x", pady=(0, 12))
        ttk.Label(settings, text="Home latitude:").grid(row=0, column=0, sticky="w")
        ttk.Entry(settings, textvariable=self.home_lat_var, width=14).grid(
            row=0, column=1, padx=(6, 12)
        )
        ttk.Label(settings, text="Home longitude:").grid(row=0, column=2, sticky="w")
        ttk.Entry(settings, textvariable=self.home_lon_var, width=14).grid(
            row=0, column=3, padx=(6, 8)
        )
        ttk.Button(settings, text="Apply home", command=self.apply_home).grid(
            row=0, column=4, padx=4
        )
        ttk.Label(settings, text="Test node lat/lon:").grid(
            row=1, column=0, sticky="w", pady=(8, 0)
        )
        ttk.Entry(settings, textvariable=self.test_lat_var, width=14).grid(
            row=1, column=1, padx=(6, 12), pady=(8, 0)
        )
        ttk.Entry(settings, textvariable=self.test_lon_var, width=14).grid(
            row=1, column=3, padx=(6, 8), pady=(8, 0)
        )
        ttk.Button(
            settings, text="Use test position", command=self.apply_test_position
        ).grid(row=1, column=4, padx=4, pady=(8, 0))
        ttk.Label(
            settings,
            text=f"Analyzer frequency: {float(os.getenv('RF_FREQUENCY_MHZ', '915')):g} MHz",
        ).grid(row=2, column=0, columnspan=5, sticky="w", pady=(8, 0))

        metrics = ttk.Frame(outer)
        metrics.pack(fill="x", pady=(0, 12))
        definitions = (
            ("rssi", "RSSI", "—"),
            ("snr", "SNR", "—"),
            ("swr", "SWR", "—"),
            ("noise", "Noise floor", "—"),
            ("distance", "Distance", "—"),
        )
        for column, (key, title, initial) in enumerate(definitions):
            card = ttk.LabelFrame(metrics, text=title, padding=12)
            card.grid(row=0, column=column, sticky="nsew", padx=4)
            value = ttk.Label(card, text=initial, style="MetricValue.TLabel")
            value.pack(anchor="center", pady=8)
            self.metric_values[key] = value
            metrics.columnconfigure(column, weight=1)

        statuses = ttk.LabelFrame(outer, text="Device status", padding=10)
        statuses.pack(fill="x", pady=(0, 12))
        self.device_status_labels: dict[str, ttk.Label] = {}
        for row, (key, label) in enumerate(
            (("nanovna_status", "NanoVNA"), ("tinysa_status", "tinySA"))
        ):
            ttk.Label(statuses, text=f"{label}:").grid(row=row, column=0, sticky="nw")
            value = ttk.Label(statuses, text="Starting...", wraplength=800)
            value.grid(row=row, column=1, sticky="w", padx=(8, 0), pady=2)
            self.device_status_labels[key] = value
        statuses.columnconfigure(1, weight=1)

        self.coordinate_label = ttk.Label(outer, text="Node coordinates: —")
        self.coordinate_label.pack(anchor="w", pady=(0, 8))

        recommendation_box = ttk.LabelFrame(
            outer, text="Improvement recommendations", padding=10
        )
        recommendation_box.pack(fill="both", expand=True)
        self.recommendations_label = ttk.Label(
            recommendation_box,
            text="Waiting for radio measurements...",
            justify="left",
            anchor="nw",
            wraplength=900,
        )
        self.recommendations_label.pack(fill="both", expand=True, anchor="nw")
        self.discovery_label = ttk.Label(outer, text="", justify="left", wraplength=920)
        self.discovery_label.pack(fill="x", anchor="w", pady=(8, 0))
        self.port_details_text = ""
        self.rf_planner: RFPlannerWindow | None = None

    def open_rf_planner(self) -> None:
        if self.rf_planner is not None:
            try:
                if self.rf_planner.window.winfo_exists():
                    self.rf_planner.window.lift()
                    self.rf_planner.window.focus_force()
                    return
            except tk.TclError:
                self.rf_planner = None
        self.rf_planner = RFPlannerWindow(self.root)

    def _initialize_monitor(self) -> None:
        try:
            monitor = RadioMonitor()
            if self.closing:
                monitor.stop()
            else:
                self.monitor = monitor
        except Exception as exc:
            self.monitor_error = str(exc)
        finally:
            self.monitor_ready.set()

    def refresh_serial_ports(self) -> None:
        """Refresh the selectable list of every serial port, regardless of VID/PID."""
        try:
            ports = list(list_ports.comports())
            devices = [port.device for port in ports]
            self.port_entry.configure(values=devices)
            self.nanovna_port_entry.configure(values=devices)
            self.tinysa_port_entry.configure(values=devices)

            if not self.port_var.get().strip():
                configured = os.getenv("MESHTASTIC_PORT", "").strip()
                if configured:
                    self.port_var.set(configured)
                elif len(devices) == 1:
                    self.port_var.set(devices[0])

            if ports:
                details = "; ".join(
                    f"{port.device}: {port.description or 'serial device'}"
                    for port in ports
                )
                self.port_details_text = f"Available serial ports: {details}"
            else:
                self.port_details_text = "No serial ports detected. Connect the node and click Refresh ports."
        except (serial.SerialException, OSError) as exc:
            self.port_details_text = f"Could not enumerate serial ports: {exc}"
        self._update_discovery_text()

    def _check_initialization(self) -> None:
        if self.closing:
            return
        if not self.monitor_ready.is_set():
            self.root.after(100, self._check_initialization)
            return

        if self.monitor_error:
            self.mesh_status_label.configure(
                text=f"Device initialization failed: {self.monitor_error}",
                style="Error.TLabel",
            )
            return
        self.connect_button.configure(state="normal")
        if self.monitor is not None:
            self.nanovna_port_var.set(
                self.nanovna_port_var.get().strip()
                or self.monitor.analyzer_ports["nanovna"]
                or ""
            )
            self.tinysa_port_var.set(
                self.tinysa_port_var.get().strip()
                or self.monitor.analyzer_ports["tinysa"]
                or ""
            )
        self.apply_home()
        self.refresh_dashboard()

    def connect_node(self) -> None:
        if self.monitor is None:
            self.mesh_status_label.configure(
                text="Device initialization is not complete.", style="Warning.TLabel"
            )
            return
        port = self.port_var.get().strip() or None
        if port and port in (
            self.nanovna_port_var.get().strip(),
            self.tinysa_port_var.get().strip(),
        ):
            self.mesh_status_label.configure(
                text="This port is assigned to an analyzer. Choose the Meshtastic "
                "node's own COM port.",
                style="Error.TLabel",
            )
            return
        try:
            self.monitor.connect_mesh(port)
            self.disconnect_button.configure(state="normal")
            self.mesh_status_label.configure(
                text=f"Starting CLI on {port or 'auto-detected port'}...",
                style="Info.TLabel",
            )
        except Exception as exc:
            self.mesh_status_label.configure(
                text=f"Could not start connection: {exc}", style="Error.TLabel"
            )

    def disconnect_node(self) -> None:
        if self.monitor is not None:
            self.monitor.disconnect_mesh()
        self.disconnect_button.configure(state="disabled")
        self.mesh_status_label.configure(text="Disconnected", style="Info.TLabel")

    def start_analyzer(self, role: str) -> None:
        if self.monitor is None:
            return
        port_var = self.nanovna_port_var if role == "nanovna" else self.tinysa_port_var
        port_name = port_var.get().strip()
        if not port_name:
            self.device_status_labels[f"{role}_status"].configure(
                text="Select a serial port first."
            )
            return
        other_port = (
            self.tinysa_port_var.get().strip()
            if role == "nanovna"
            else self.nanovna_port_var.get().strip()
        )
        if port_name in (self.port_var.get().strip(), other_port):
            self.device_status_labels[f"{role}_status"].configure(
                text="This port is already assigned to another device. "
                "Choose its own serial port."
            )
            return
        try:
            self.monitor.set_analyzer_port(role, port_name)
        except (OSError, ValueError, serial.SerialException) as exc:
            self.device_status_labels[f"{role}_status"].configure(
                text=f"Could not start: {exc}"
            )

    def apply_home(self) -> None:
        if self.monitor is None:
            return
        try:
            latitude = float(self.home_lat_var.get())
            longitude = float(self.home_lon_var.get())
            if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
                raise ValueError("Latitude must be -90..90 and longitude -180..180.")
            self.monitor.set_home(latitude, longitude)
        except ValueError as exc:
            self.mesh_status_label.configure(
                text=f"Invalid home coordinates: {exc}", style="Error.TLabel"
            )

    def apply_test_position(self) -> None:
        if self.monitor is None:
            return
        try:
            latitude = float(self.test_lat_var.get())
            longitude = float(self.test_lon_var.get())
            if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
                raise ValueError("Latitude must be -90..90 and longitude -180..180.")
            self.monitor.set_test_position(latitude, longitude)
        except ValueError as exc:
            self.coordinate_label.configure(
                text=f"Invalid test coordinates: {exc}", style="Error.TLabel"
            )

    @staticmethod
    def format_metric(value: float | None, unit: str) -> str:
        return f"{value:.1f} {unit}" if value is not None else "—"

    def refresh_dashboard(self) -> None:
        if self.closing:
            return
        with TELEMETRY_LOCK:
            snapshot = dict(TELEMETRY)
            snapshot["history"] = list(TELEMETRY["history"])

        self.metric_values["rssi"].configure(
            text=self.format_metric(snapshot["rssi_dbm"], "dBm")
        )
        self.metric_values["snr"].configure(
            text=self.format_metric(snapshot["snr_db"], "dB")
        )
        swr = snapshot["swr"]
        swr_text = (
            "—" if swr is None else (f"{swr:.2f}:1" if math.isfinite(swr) else "∞")
        )
        self.metric_values["swr"].configure(text=swr_text)
        self.metric_values["swr"].configure(
            foreground="#166534" if swr is not None and swr <= 1.5 else "#9a3412"
        )
        self.metric_values["noise"].configure(
            text=self.format_metric(snapshot["noise_floor_dbm"], "dBm")
        )
        distance = snapshot["distance_m"]
        self.metric_values["distance"].configure(
            text=f"{distance / 1000:.3f} km" if distance is not None else "—"
        )

        mesh_status = snapshot["mesh_status"]
        mesh_style = (
            "Success.TLabel"
            if mesh_status.startswith("CLI connected:")
            else "Error.TLabel"
            if "error" in mesh_status.lower() or "exited" in mesh_status.lower()
            else "Info.TLabel"
        )
        self.mesh_status_label.configure(text=mesh_status, style=mesh_style)
        if mesh_status == "Disconnected":
            self.disconnect_button.configure(state="disabled")
        elif self.monitor is not None and self.monitor.mesh_worker is not None:
            self.disconnect_button.configure(state="normal")

        for key in ("nanovna_status", "tinysa_status"):
            self.device_status_labels[key].configure(text=snapshot[key])
        if snapshot["node_lat"] is not None and snapshot["node_lon"] is not None:
            self.coordinate_label.configure(
                text=f"Node coordinates ({snapshot['coordinate_source']}): "
                f"{snapshot['node_lat']:.6f}, {snapshot['node_lon']:.6f}"
            )
        else:
            self.coordinate_label.configure(text="Node coordinates: not received")

        recommendations = build_recommendations(snapshot)
        if recommendations:
            text = "\n\n".join(message for _, message in recommendations)
            style = (
                "Warning.TLabel"
                if any(level == "warning" for level, _ in recommendations)
                else "Info.TLabel"
            )
        else:
            text = "No threshold-based recommendations at the moment."
            style = "Success.TLabel"
        self.recommendations_label.configure(text=text, style=style)

        notes = snapshot["discovery_notes"]
        self._update_discovery_text(notes)
        self.root.after(1000, self.refresh_dashboard)

    def _update_discovery_text(self, notes: list[str] | None = None) -> None:
        messages = [self.port_details_text]
        if notes:
            messages.append("Device discovery: " + " ".join(notes))
        self.discovery_label.configure(
            text="\n".join(message for message in messages if message)
        )

    def close(self) -> None:
        self.closing = True
        if self.monitor is not None:
            self.monitor.stop()
        self.root.destroy()


# OSM map widget and tile-download/cache behavior.
class OpenStreetMapView:
    """Interactive OSM tile map with click-to-place endpoints and local tile cache."""

    TILE_SIZE = 256
    MIN_ZOOM = 2
    MAX_ZOOM = 18

    def __init__(
        self,
        parent: tk.Misc,
        tx_var: tk.StringVar,
        rx_var: tk.StringVar,
        status_var: tk.StringVar,
    ) -> None:
        self.parent = parent
        self.tx_var = tx_var
        self.rx_var = rx_var
        self.status_var = status_var
        self.zoom = 12
        with TELEMETRY_LOCK:
            latitude = TELEMETRY.get("home_lat")
            longitude = TELEMETRY.get("home_lon")
        if latitude is None or longitude is None or (latitude == 0 and longitude == 0):
            latitude = float(os.getenv("HOME_LAT", "0"))
            longitude = float(os.getenv("HOME_LON", "0"))
        self.center_x, self.center_y = coordinates_to_world_pixel(
            float(latitude), float(longitude), self.zoom
        )

        self.tile_queue: queue.Queue[
            tuple[tuple[int, int, int], bytes | None, str | None]
        ] = queue.Queue()
        self.tile_data: dict[tuple[int, int, int], bytes] = {}
        self.tile_images: dict[tuple[int, int, int], ImageTk.PhotoImage] = {}
        self.pending_tiles: set[tuple[int, int, int]] = set()
        self.executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix="osm-tile")
        self.closed = False
        self.drag_origin: tuple[int, int] | None = None
        self.drag_moved = False
        self.last_tile_error: str | None = None
        self.on_points_changed: Any = lambda: None
        self.coverage_radius_m: float | None = None
        self.show_coverage_var = tk.BooleanVar(value=False)

        cache_root = Path(
            os.getenv("LOCALAPPDATA", str(Path.home() / "AppData" / "Local"))
        )
        self.cache_dir = cache_root / "RadioRangeMonitor" / "osm_tiles"

        self.frame = ttk.Frame(parent)
        self.frame.pack(fill="both", expand=True)
        toolbar = ttk.Frame(self.frame)
        toolbar.pack(fill="x", pady=(0, 6))

        self.point_mode = tk.StringVar(value="tx")
        ttk.Label(toolbar, text="Map click sets:").pack(side="left")
        self.mode_combo = ttk.Combobox(
            toolbar,
            textvariable=self.point_mode,
            values=("Transmitter", "Receiver"),
            state="readonly",
            width=14,
        )
        self.mode_combo.current(0)
        self.mode_combo.pack(side="left", padx=(6, 12))
        self.point_mode.set("Transmitter")
        ttk.Button(
            toolbar, text="−", width=3, command=lambda: self.change_zoom(-1)
        ).pack(side="left", padx=2)
        ttk.Button(
            toolbar, text="+", width=3, command=lambda: self.change_zoom(1)
        ).pack(side="left", padx=2)
        ttk.Button(
            toolbar, text="Center on TX", command=lambda: self.center_on("tx")
        ).pack(side="left", padx=(8, 2))
        ttk.Button(
            toolbar, text="Center on RX", command=lambda: self.center_on("rx")
        ).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Use home", command=self.set_home_as_tx).pack(
            side="left", padx=(8, 2)
        )
        ttk.Button(
            toolbar, text="Use live node", command=self.set_live_node_as_rx
        ).pack(side="left", padx=2)
        ttk.Checkbutton(
            toolbar,
            text="Show free-space coverage ring",
            variable=self.show_coverage_var,
            command=self.redraw,
        ).pack(side="left", padx=(10, 0))

        self.canvas = tk.Canvas(
            self.frame,
            background="#e5e7eb",
            highlightthickness=1,
            highlightbackground="#94a3b8",
            cursor="crosshair",
        )
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", self.redraw)
        self.canvas.bind("<ButtonPress-1>", self.begin_drag)
        self.canvas.bind("<B1-Motion>", self.pan_drag)
        self.canvas.bind("<ButtonRelease-1>", self.finish_drag)
        self.canvas.bind("<MouseWheel>", self.mousewheel_zoom)
        self.canvas.bind("<Button-4>", self.mousewheel_zoom_in)
        self.canvas.bind("<Button-5>", self.mousewheel_zoom_out)
        ttk.Label(
            self.frame,
            textvariable=self.status_var,
            wraplength=900,
        ).pack(fill="x", anchor="w", pady=(6, 0))
        ttk.Label(
            self.frame,
            text="Map data © OpenStreetMap contributors | Tiles from tile.openstreetmap.org",
        ).pack(anchor="e", pady=(3, 0))
        self.parent.after(100, self.drain_tiles)

    def _tile_path(self, key: tuple[int, int, int]) -> Path:
        zoom, x, y = key
        return self.cache_dir / str(zoom) / str(x) / f"{y}.png"

    def _request_tile(self, key: tuple[int, int, int]) -> None:
        if key in self.tile_data or key in self.pending_tiles or self.closed:
            return
        cache_path = self._tile_path(key)
        if cache_path.is_file():
            try:
                self.tile_data[key] = cache_path.read_bytes()
                return
            except OSError:
                pass
        self.pending_tiles.add(key)
        self.executor.submit(self._download_tile, key)

    def _download_tile(self, key: tuple[int, int, int]) -> None:
        zoom, x, y = key
        try:
            request = Request(
                f"https://tile.openstreetmap.org/{zoom}/{x}/{y}.png",
                headers={
                    "User-Agent": "RadioRangeMonitor/1.0 (Tk desktop RF planner)",
                },
            )
            with urlopen(request, timeout=12) as response:
                data = response.read()
            self.cache_dir.mkdir(parents=True, exist_ok=True)
            cache_path = self._tile_path(key)
            cache_path.parent.mkdir(parents=True, exist_ok=True)
            cache_path.write_bytes(data)
            self.tile_queue.put((key, data, None))
        except Exception as exc:
            self.tile_queue.put((key, None, str(exc)))

    def drain_tiles(self) -> None:
        if self.closed:
            return
        changed = False
        try:
            while True:
                key, data, error = self.tile_queue.get_nowait()
                self.pending_tiles.discard(key)
                if data is not None:
                    self.tile_data[key] = data
                    changed = True
                elif error:
                    self.last_tile_error = error
        except queue.Empty:
            pass
        if changed:
            self.redraw()
        if self.last_tile_error and not self.tile_data:
            self.status_var.set(
                "OpenStreetMap tiles could not be loaded. Check internet access. "
                f"Details: {self.last_tile_error}"
            )
        self.parent.after(150, self.drain_tiles)

    def redraw(self, _event: tk.Event[Any] | None = None) -> None:
        del _event
        if self.closed:
            return
        canvas = self.canvas
        width = max(canvas.winfo_width(), 320)
        height = max(canvas.winfo_height(), 240)
        canvas.delete("all")
        left = self.center_x - width / 2
        top = self.center_y - height / 2
        first_tile_x = math.floor(left / self.TILE_SIZE)
        last_tile_x = math.floor((left + width) / self.TILE_SIZE)
        first_tile_y = math.floor(top / self.TILE_SIZE)
        last_tile_y = math.floor((top + height) / self.TILE_SIZE)
        tile_count = 2**self.zoom

        for unwrapped_x in range(first_tile_x, last_tile_x + 1):
            tile_x = unwrapped_x % tile_count
            for tile_y in range(first_tile_y, last_tile_y + 1):
                if not 0 <= tile_y < tile_count:
                    continue
                key = (self.zoom, tile_x, tile_y)
                self._request_tile(key)
                canvas_x = unwrapped_x * self.TILE_SIZE - left
                canvas_y = tile_y * self.TILE_SIZE - top
                image = self.tile_images.get(key)
                if image is None and key in self.tile_data:
                    try:
                        image_data = Image.open(
                            io.BytesIO(self.tile_data[key])
                        ).convert("RGB")
                        image = ImageTk.PhotoImage(image_data, master=canvas)
                        self.tile_images[key] = image
                    except Exception as exc:
                        self.last_tile_error = f"Invalid map tile image: {exc}"
                        continue
                if image is not None:
                    canvas.create_image(
                        canvas_x, canvas_y, image=image, anchor="nw", tags="map_tile"
                    )
                else:
                    canvas.create_rectangle(
                        canvas_x,
                        canvas_y,
                        canvas_x + self.TILE_SIZE,
                        canvas_y + self.TILE_SIZE,
                        fill="#e5e7eb",
                        outline="#cbd5e1",
                    )
                    canvas.create_text(
                        canvas_x + self.TILE_SIZE / 2,
                        canvas_y + self.TILE_SIZE / 2,
                        text="Loading map…",
                        fill="#475569",
                    )

        if self.show_coverage_var.get() and self.coverage_radius_m is not None:
            self._draw_coverage_ring(left, top)
        self._draw_endpoints(left, top, width, height)
        lat, lon = world_pixel_to_coordinates(self.center_x, self.center_y, self.zoom)
        self.status_var.set(
            f"Map center {lat:.5f}, {lon:.5f} | zoom {self.zoom} | "
            "left-click to place the selected endpoint; drag to pan; mouse wheel to zoom."
        )
        if self.last_tile_error and not self.tile_data:
            self.status_var.set(
                f"OpenStreetMap tiles could not be loaded: {self.last_tile_error}"
            )

    def _draw_endpoints(self, left: float, top: float, width: int, height: int) -> None:
        point_pixels: dict[str, tuple[float, float]] = {}
        for key, variable, color, label in (
            ("tx", self.tx_var, "#dc2626", "TX"),
            ("rx", self.rx_var, "#2563eb", "RX"),
        ):
            try:
                latitude, longitude = (
                    float(value.strip()) for value in variable.get().split(",", 1)
                )
                px, py = coordinates_to_world_pixel(latitude, longitude, self.zoom)
                x, y = px - left, py - top
                point_pixels[key] = (x, y)
                if -30 <= x <= width + 30 and -30 <= y <= height + 30:
                    self.canvas.create_oval(
                        x - 9,
                        y - 9,
                        x + 9,
                        y + 9,
                        fill=color,
                        outline="white",
                        width=2,
                    )
                    self.canvas.create_text(
                        x + 12,
                        y - 12,
                        text=label,
                        fill=color,
                        anchor="sw",
                        font=("Segoe UI", 10, "bold"),
                    )
            except (ValueError, TypeError):
                continue

        if "tx" in point_pixels and "rx" in point_pixels:
            self.canvas.create_line(
                *point_pixels["tx"],
                *point_pixels["rx"],
                fill="#f59e0b",
                width=3,
                dash=(7, 4),
            )

    def _draw_coverage_ring(self, left: float, top: float) -> None:
        try:
            latitude, longitude = (
                float(value.strip()) for value in self.tx_var.get().split(",", 1)
            )
        except (ValueError, TypeError):
            return
        if self.coverage_radius_m is None or self.coverage_radius_m <= 0:
            return
        angular_radius = min(self.coverage_radius_m / EARTH_RADIUS_M, math.pi - 1e-6)
        points: list[float] = []
        previous_world_x: float | None = None
        world_size = 256 * 2**self.zoom
        lat1, lon1 = math.radians(latitude), math.radians(longitude)
        for bearing_degrees in range(0, 361, 5):
            bearing = math.radians(bearing_degrees)
            lat2 = math.asin(
                math.sin(lat1) * math.cos(angular_radius)
                + math.cos(lat1) * math.sin(angular_radius) * math.cos(bearing)
            )
            lon2 = lon1 + math.atan2(
                math.sin(bearing) * math.sin(angular_radius) * math.cos(lat1),
                math.cos(angular_radius) - math.sin(lat1) * math.sin(lat2),
            )
            ring_lat = math.degrees(lat2)
            ring_lon = (math.degrees(lon2) + 540) % 360 - 180
            world_x, world_y = coordinates_to_world_pixel(ring_lat, ring_lon, self.zoom)
            if previous_world_x is not None:
                while world_x - previous_world_x > world_size / 2:
                    world_x -= world_size
                while world_x - previous_world_x < -world_size / 2:
                    world_x += world_size
            previous_world_x = world_x
            points.extend(
                (
                    world_x - left,
                    world_y - top,
                )
            )
        self.canvas.create_line(
            *points,
            fill="#0f766e",
            width=2,
            dash=(6, 3),
            tags="coverage_ring",
        )

    def begin_drag(self, event: tk.Event[Any]) -> None:
        self.drag_origin = (event.x, event.y)
        self.drag_moved = False

    def pan_drag(self, event: tk.Event[Any]) -> None:
        if self.drag_origin is None:
            return
        old_x, old_y = self.drag_origin
        delta_x, delta_y = event.x - old_x, event.y - old_y
        if abs(delta_x) + abs(delta_y) > 3:
            self.drag_moved = True
        if self.drag_moved:
            self.center_x -= delta_x
            self.center_y -= delta_y
            self.drag_origin = (event.x, event.y)
            self.redraw()

    def finish_drag(self, event: tk.Event[Any]) -> None:
        if self.drag_origin is None:
            return
        if not self.drag_moved:
            width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
            world_x = self.center_x + event.x - width / 2
            world_y = self.center_y + event.y - height / 2
            latitude, longitude = world_pixel_to_coordinates(
                world_x, world_y, self.zoom
            )
            coordinate = f"{latitude:.6f},{longitude:.6f}"
            if self.point_mode.get() == "Transmitter":
                self.tx_var.set(coordinate)
            else:
                self.rx_var.set(coordinate)
            self.on_points_changed()
            self.redraw()
        self.drag_origin = None

    def change_zoom(self, delta: int) -> None:
        new_zoom = max(self.MIN_ZOOM, min(self.MAX_ZOOM, self.zoom + delta))
        if new_zoom == self.zoom:
            return
        lat, lon = world_pixel_to_coordinates(self.center_x, self.center_y, self.zoom)
        self.zoom = new_zoom
        self.center_x, self.center_y = coordinates_to_world_pixel(lat, lon, self.zoom)
        self.tile_images.clear()
        self.redraw()

    def mousewheel_zoom(self, event: tk.Event[Any]) -> None:
        self.change_zoom(1 if event.delta > 0 else -1)

    def mousewheel_zoom_in(self, event: tk.Event[Any]) -> None:
        del event
        self.change_zoom(1)

    def mousewheel_zoom_out(self, event: tk.Event[Any]) -> None:
        del event
        self.change_zoom(-1)

    def center_on(self, role: str) -> None:
        variable = self.tx_var if role == "tx" else self.rx_var
        try:
            latitude, longitude = (
                float(value.strip()) for value in variable.get().split(",", 1)
            )
            self.center_x, self.center_y = coordinates_to_world_pixel(
                latitude, longitude, self.zoom
            )
            self.redraw()
        except ValueError:
            self.status_var.set(f"Enter valid {role.upper()} coordinates first.")

    def set_home_as_tx(self) -> None:
        with TELEMETRY_LOCK:
            lat, lon = TELEMETRY["home_lat"], TELEMETRY["home_lon"]
        self.tx_var.set(f"{float(lat):.6f},{float(lon):.6f}")
        self.on_points_changed()
        self.center_on("tx")

    def set_live_node_as_rx(self) -> None:
        with TELEMETRY_LOCK:
            lat, lon = TELEMETRY["node_lat"], TELEMETRY["node_lon"]
        if lat is None or lon is None:
            self.status_var.set("No live node coordinates have been received yet.")
            return
        self.rx_var.set(f"{float(lat):.6f},{float(lon):.6f}")
        self.on_points_changed()
        self.center_on("rx")

    def close(self) -> None:
        self.closed = True
        self.executor.shutdown(wait=False, cancel_futures=True)


# Secondary planner window: link budget, terrain profile, and map tools.
class RFPlannerWindow:
    """Original RF link-budget and terrain-planning tools in a tabbed window."""

    def __init__(self, parent: tk.Tk) -> None:
        self.window = tk.Toplevel(parent)
        self.window.title("RF Link & Terrain Planner")
        self.window.geometry("940x760")
        self.window.minsize(780, 620)
        self.terrain_results: list[float] | None = None
        self.terrain_distance_m = 0.0
        self.last_link_result: dict[str, float] | None = None
        self.terrain_queue: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.terrain_status_var = tk.StringVar(value="Terrain profile not loaded.")
        self.link_result_var = tk.StringVar(
            value="Enter link parameters and calculate the link budget."
        )
        self.coverage_result_var = tk.StringVar(
            value="Calculate the link budget first."
        )
        self.input_vars = {
            "tx_lat": tk.StringVar(
                value=self._initial_coordinate("home_lat", "HOME_LAT")
            ),
            "tx_lon": tk.StringVar(
                value=self._initial_coordinate("home_lon", "HOME_LON")
            ),
            "rx_lat": tk.StringVar(
                value=self._initial_coordinate("node_lat", "HOME_LAT")
            ),
            "rx_lon": tk.StringVar(
                value=self._initial_coordinate("node_lon", "HOME_LON")
            ),
            "frequency_mhz": tk.StringVar(value=os.getenv("RF_FREQUENCY_MHZ", "915")),
            "tx_power_dbm": tk.StringVar(value="30"),
            "tx_gain_dbi": tk.StringVar(value="2"),
            "rx_gain_dbi": tk.StringVar(value="2"),
            "system_loss_db": tk.StringVar(value="2"),
            "rx_sensitivity_dbm": tk.StringVar(value="-120"),
            "tx_height_m": tk.StringVar(value="10"),
            "rx_height_m": tk.StringVar(value="2"),
            "earth_k_factor": tk.StringVar(value="1.3333"),
        }
        self.map_tx_var = tk.StringVar(
            value=(
                f"{self.input_vars['tx_lat'].get()},{self.input_vars['tx_lon'].get()}"
            )
        )
        self.map_rx_var = tk.StringVar(
            value=(
                f"{self.input_vars['rx_lat'].get()},{self.input_vars['rx_lon'].get()}"
            )
        )
        self._build_window()
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.window.after(150, self._drain_terrain_queue)

    @staticmethod
    def _initial_coordinate(key: str, env_name: str) -> str:
        with TELEMETRY_LOCK:
            value = TELEMETRY.get(key)
        if value is not None:
            return f"{float(value):.6f}"
        return os.getenv(env_name, "0")

    def _build_window(self) -> None:
        outer = ttk.Frame(self.window, padding=12)
        outer.pack(fill="both", expand=True)
        ttk.Label(
            outer,
            text="RF Planning — link budget, terrain path, and coverage estimate",
            style="Section.TLabel",
        ).pack(anchor="w", pady=(0, 8))
        ttk.Label(
            outer,
            text="Engineering estimates only: actual performance depends on antennas, "
            "installation, interference, and local conditions.",
            wraplength=880,
        ).pack(anchor="w", pady=(0, 10))

        self.notebook = ttk.Notebook(outer)
        self.notebook.pack(fill="both", expand=True)
        self.map_tab = ttk.Frame(self.notebook, padding=12)
        self.link_tab = ttk.Frame(self.notebook, padding=12)
        self.terrain_tab = ttk.Frame(self.notebook, padding=12)
        self.coverage_tab = ttk.Frame(self.notebook, padding=12)
        self.notebook.add(self.map_tab, text="Map")
        self.notebook.add(self.link_tab, text="Link Budget")
        self.notebook.add(self.terrain_tab, text="Terrain Profile")
        self.notebook.add(self.coverage_tab, text="Coverage Estimate")
        self._build_map_tab()
        self.notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)
        self._build_link_tab()
        self._build_terrain_tab()
        self._build_coverage_tab()

    def _build_map_tab(self) -> None:
        ttk.Label(
            self.map_tab,
            text="Choose which endpoint to set, then click the map. Drag to pan "
            "and use the mouse wheel or +/− buttons to zoom.",
            wraplength=860,
        ).pack(anchor="w", pady=(0, 8))
        self.map_status_var = tk.StringVar(value="Loading OpenStreetMap tiles...")
        self.map_points_var = tk.StringVar()
        self.map_view = OpenStreetMapView(
            self.map_tab,
            self.map_tx_var,
            self.map_rx_var,
            self.map_status_var,
        )
        self.map_view.on_points_changed = self.apply_map_points
        ttk.Label(
            self.map_tab,
            textvariable=self.map_points_var,
            justify="left",
        ).pack(fill="x", anchor="w", pady=(6, 0))
        self._update_map_points_label()

    def apply_map_points(self) -> None:
        """Copy selected map points to the inputs shared by all planner tabs."""
        for variable_name, coordinate_var, lat_key, lon_key in (
            ("tx", self.map_tx_var, "tx_lat", "tx_lon"),
            ("rx", self.map_rx_var, "rx_lat", "rx_lon"),
        ):
            try:
                latitude, longitude = (
                    float(value.strip()) for value in coordinate_var.get().split(",", 1)
                )
            except (ValueError, TypeError):
                self.map_status_var.set(
                    f"Invalid {variable_name.upper()} point on map."
                )
                return
            self.input_vars[lat_key].set(f"{latitude:.6f}")
            self.input_vars[lon_key].set(f"{longitude:.6f}")
        self._update_map_points_label()

    def _update_map_points_label(self) -> None:
        self.map_points_var.set(
            f"TX: {self.map_tx_var.get()}    RX: {self.map_rx_var.get()}"
        )

    def _on_tab_changed(self, event: tk.Event[Any]) -> None:
        del event
        if self.notebook.select() == str(self.map_tab):
            self.map_tx_var.set(
                f"{self.input_vars['tx_lat'].get()},{self.input_vars['tx_lon'].get()}"
            )
            self.map_rx_var.set(
                f"{self.input_vars['rx_lat'].get()},{self.input_vars['rx_lon'].get()}"
            )
            self._update_map_points_label()
            self.map_view.redraw()

    def close(self) -> None:
        self.map_view.close()
        self.window.destroy()

    def _build_link_tab(self) -> None:
        fields = (
            ("tx_lat", "Transmitter latitude", -90, 90),
            ("tx_lon", "Transmitter longitude", -180, 180),
            ("rx_lat", "Receiver latitude", -90, 90),
            ("rx_lon", "Receiver longitude", -180, 180),
            ("frequency_mhz", "Frequency (MHz)", 0.000001, None),
            ("tx_power_dbm", "Transmitter power (dBm)", None, None),
            ("tx_gain_dbi", "TX antenna gain (dBi)", None, None),
            ("rx_gain_dbi", "RX antenna gain (dBi)", None, None),
            ("system_loss_db", "Cable/other losses (dB)", 0, None),
            ("rx_sensitivity_dbm", "Receiver sensitivity (dBm)", None, None),
            ("tx_height_m", "TX antenna height AGL (m)", 0, None),
            ("rx_height_m", "RX antenna height AGL (m)", 0, None),
            ("earth_k_factor", "Effective Earth k-factor", 0.1, None),
        )
        left = ttk.Frame(self.link_tab)
        right = ttk.Frame(self.link_tab)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 14))
        right.grid(row=0, column=1, sticky="nsew")
        self.link_tab.columnconfigure(0, weight=1)
        self.link_tab.columnconfigure(1, weight=1)
        midpoint = (len(fields) + 1) // 2
        for index, (key, label, _, _) in enumerate(fields):
            parent = left if index < midpoint else right
            row = index if index < midpoint else index - midpoint
            ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=5)
            ttk.Entry(parent, textvariable=self.input_vars[key], width=17).grid(
                row=row, column=1, sticky="ew", padx=(8, 0), pady=5
            )
            parent.columnconfigure(1, weight=1)

        ttk.Button(
            self.link_tab,
            text="Calculate link budget",
            command=self.calculate_link_budget,
        ).grid(row=1, column=0, sticky="w", pady=(14, 8))
        ttk.Label(
            self.link_tab,
            textvariable=self.link_result_var,
            justify="left",
            anchor="nw",
            wraplength=850,
        ).grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(4, 0))
        self.link_tab.rowconfigure(2, weight=1)

    def _read_inputs(self) -> dict[str, float]:
        values: dict[str, float] = {}
        for name, variable in self.input_vars.items():
            try:
                values[name] = float(variable.get())
            except ValueError as exc:
                raise ValueError(f"{name.replace('_', ' ')} must be numeric.") from exc
        for key, minimum, maximum in (
            ("tx_lat", -90, 90),
            ("rx_lat", -90, 90),
            ("tx_lon", -180, 180),
            ("rx_lon", -180, 180),
        ):
            if not minimum <= values[key] <= maximum:
                raise ValueError(f"{key.replace('_', ' ')} is out of range.")
        for key in ("frequency_mhz", "earth_k_factor"):
            if values[key] <= 0:
                raise ValueError(f"{key.replace('_', ' ')} must be greater than zero.")
        for key in ("tx_height_m", "rx_height_m", "system_loss_db"):
            if values[key] < 0:
                raise ValueError(f"{key.replace('_', ' ')} cannot be negative.")
        return values

    def calculate_link_budget(self) -> dict[str, float] | None:
        try:
            values = self._read_inputs()
            distance_m = haversine_m(
                values["tx_lat"],
                values["tx_lon"],
                values["rx_lat"],
                values["rx_lon"],
            )
            if distance_m < 1:
                raise ValueError("Transmitter and receiver must be at least 1 m apart.")
            fspl_db = free_space_path_loss_db(values["frequency_mhz"], distance_m)
            received_dbm = (
                values["tx_power_dbm"]
                + values["tx_gain_dbi"]
                + values["rx_gain_dbi"]
                - values["system_loss_db"]
                - fspl_db
            )
            fade_margin_db = received_dbm - values["rx_sensitivity_dbm"]
            max_path_loss_db = (
                values["tx_power_dbm"]
                + values["tx_gain_dbi"]
                + values["rx_gain_dbi"]
                - values["system_loss_db"]
                - values["rx_sensitivity_dbm"]
            )
            range_km = 10 ** (
                (max_path_loss_db - 32.44 - 20 * math.log10(values["frequency_mhz"]))
                / 20
            )
            result = {
                **values,
                "distance_m": distance_m,
                "fspl_db": fspl_db,
                "received_dbm": received_dbm,
                "fade_margin_db": fade_margin_db,
                "free_space_range_km": range_km,
            }
            self.link_result_var.set(
                f"Path distance: {distance_m / 1000:.3f} km\n"
                f"Free-space path loss: {fspl_db:.1f} dB\n"
                f"Estimated received power (no terrain/clutter): {received_dbm:.1f} dBm\n"
                f"Fade margin vs. receiver sensitivity: {fade_margin_db:.1f} dB\n"
                f"Free-space sensitivity-limited range: {range_km:.2f} km\n\n"
                "The range assumes ideal free-space propagation and is an upper-bound "
                "estimate, not a guaranteed service radius."
            )
            self._draw_coverage(result)
            self.last_link_result = result
            self.map_view.coverage_radius_m = result["free_space_range_km"] * 1000
            self.map_view.redraw()
            return result
        except (ValueError, OverflowError) as exc:
            self.link_result_var.set(f"Could not calculate link: {exc}")
            return None

    def _build_terrain_tab(self) -> None:
        controls = ttk.Frame(self.terrain_tab)
        controls.pack(fill="x")
        ttk.Button(
            controls,
            text="Fetch terrain profile",
            command=self.fetch_terrain,
        ).pack(side="left")
        ttk.Label(
            controls,
            text="Uses OpenTopoData SRTM90m elevation data; internet required.",
        ).pack(side="left", padx=10)
        self.terrain_status_label = ttk.Label(
            self.terrain_tab,
            textvariable=self.terrain_status_var,
            wraplength=870,
            justify="left",
        )
        self.terrain_status_label.pack(fill="x", pady=(10, 8))
        self.terrain_canvas = tk.Canvas(
            self.terrain_tab,
            height=380,
            background="white",
            highlightthickness=1,
            highlightbackground="#a3a3a3",
        )
        self.terrain_canvas.pack(fill="both", expand=True)
        self.terrain_canvas.bind("<Configure>", self._redraw_terrain)
        ttk.Label(
            self.terrain_tab,
            text="Terrain elevations are sampled at roughly 90 m dataset resolution; "
            "small obstacles, buildings, and vegetation are not represented.",
            wraplength=870,
        ).pack(anchor="w", pady=(8, 0))

    def fetch_terrain(self) -> None:
        try:
            values = self._read_inputs()
            distance_m = haversine_m(
                values["tx_lat"],
                values["tx_lon"],
                values["rx_lat"],
                values["rx_lon"],
            )
            if distance_m < 1:
                raise ValueError("Transmitter and receiver must be at least 1 m apart.")
        except ValueError as exc:
            self.terrain_status_var.set(f"Invalid profile inputs: {exc}")
            return

        self.terrain_status_var.set("Fetching elevation samples...")
        self.terrain_canvas.delete("all")
        threading.Thread(
            target=self._fetch_terrain_worker,
            args=(values, distance_m),
            name="terrain-profile-fetch",
            daemon=True,
        ).start()

    def _fetch_terrain_worker(
        self, values: dict[str, float], distance_m: float
    ) -> None:
        try:
            elevations = fetch_elevation_profile(
                values["tx_lat"],
                values["tx_lon"],
                values["rx_lat"],
                values["rx_lon"],
                sample_count=41,
            )
            analysis = terrain_path_analysis(
                elevations,
                distance_m,
                values["frequency_mhz"],
                values["tx_height_m"],
                values["rx_height_m"],
                values["earth_k_factor"],
            )
            path_loss = (
                free_space_path_loss_db(values["frequency_mhz"], distance_m)
                + analysis["diffraction_loss_db"]
            )
            received_dbm = (
                values["tx_power_dbm"]
                + values["tx_gain_dbi"]
                + values["rx_gain_dbi"]
                - values["system_loss_db"]
                - path_loss
            )
            margin_db = received_dbm - values["rx_sensitivity_dbm"]
            self.terrain_queue.put(
                (
                    "success",
                    (elevations, distance_m, analysis, received_dbm, margin_db),
                )
            )
        except Exception as exc:
            self.terrain_queue.put(("error", str(exc)))

    def _drain_terrain_queue(self) -> None:
        if not self.window.winfo_exists():
            return
        try:
            while True:
                kind, payload = self.terrain_queue.get_nowait()
                if kind == "error":
                    self.terrain_status_var.set(f"Terrain profile failed: {payload}")
                else:
                    (
                        self.terrain_results,
                        self.terrain_distance_m,
                        analysis,
                        received_dbm,
                        margin_db,
                    ) = payload
                    ratio = analysis["min_fresnel_clearance_ratio"]
                    clearance_text = (
                        "not available"
                        if math.isnan(ratio)
                        else f"{ratio:.2f} × first Fresnel radius"
                    )
                    self.terrain_status_var.set(
                        f"Distance {self.terrain_distance_m / 1000:.2f} km | "
                        f"diffraction estimate {analysis['diffraction_loss_db']:.1f} dB | "
                        f"predicted received power {received_dbm:.1f} dBm | "
                        f"fade margin {margin_db:.1f} dB | minimum Fresnel clearance "
                        f"{clearance_text}.\n"
                        "Uses a single dominant knife-edge approximation and effective "
                        "Earth k-factor; not a full ITU-R P.452/Longley-Rice prediction."
                    )
                    self._redraw_terrain()
        except queue.Empty:
            pass
        self.window.after(150, self._drain_terrain_queue)

    def _redraw_terrain(self, _event: tk.Event[Any] | None = None) -> None:
        del _event
        canvas = getattr(self, "terrain_canvas", None)
        elevations = self.terrain_results
        if canvas is None or not elevations:
            return
        width = max(canvas.winfo_width(), 400)
        height = max(canvas.winfo_height(), 250)
        canvas.delete("all")
        pad_left, pad_right, pad_top, pad_bottom = 65, 20, 25, 45
        plot_width = width - pad_left - pad_right
        plot_height = height - pad_top - pad_bottom
        min_elevation = min(elevations)
        max_elevation = max(elevations)
        try:
            tx_height = float(self.input_vars["tx_height_m"].get())
            rx_height = float(self.input_vars["rx_height_m"].get())
        except ValueError:
            tx_height, rx_height = 10.0, 2.0
        vertical_min = min(min_elevation - 10, elevations[0], elevations[-1])
        vertical_max = max(
            max_elevation + 10,
            elevations[0] + tx_height,
            elevations[-1] + rx_height,
        )
        if vertical_max == vertical_min:
            vertical_max += 1

        def xy(index: int, elevation: float) -> tuple[float, float]:
            x = pad_left + plot_width * index / (len(elevations) - 1)
            y = (
                pad_top
                + (vertical_max - elevation)
                / (vertical_max - vertical_min)
                * plot_height
            )
            return x, y

        terrain_points: list[float] = []
        for index, elevation in enumerate(elevations):
            terrain_points.extend(xy(index, elevation))
        canvas.create_line(*terrain_points, fill="#8b5a2b", width=3, smooth=False)
        tx_top = elevations[0] + tx_height
        rx_top = elevations[-1] + rx_height
        tx_xy = xy(0, tx_top)
        rx_xy = xy(len(elevations) - 1, rx_top)
        canvas.create_line(*tx_xy, *rx_xy, fill="#2563eb", width=2, dash=(6, 3))
        canvas.create_text(
            pad_left, pad_top - 8, text=f"{vertical_max:.0f} m", anchor="sw"
        )
        canvas.create_text(
            pad_left, pad_top + plot_height + 10, text="Transmitter", anchor="n"
        )
        canvas.create_text(
            pad_left + plot_width,
            pad_top + plot_height + 10,
            text="Receiver",
            anchor="n",
        )
        canvas.create_text(
            pad_left + plot_width / 2,
            height - 8,
            text=f"Path distance: {self.terrain_distance_m / 1000:.2f} km",
            anchor="s",
        )
        canvas.create_line(
            pad_left + 8,
            pad_top + 8,
            pad_left + 35,
            pad_top + 8,
            fill="#8b5a2b",
            width=3,
        )
        canvas.create_text(pad_left + 42, pad_top + 8, text="Terrain", anchor="w")
        canvas.create_line(
            pad_left + 110,
            pad_top + 8,
            pad_left + 137,
            pad_top + 8,
            fill="#2563eb",
            width=2,
            dash=(6, 3),
        )
        canvas.create_text(
            pad_left + 144, pad_top + 8, text="Direct radio path", anchor="w"
        )

    def _build_coverage_tab(self) -> None:
        ttk.Button(
            self.coverage_tab,
            text="Calculate free-space coverage estimate",
            command=self.calculate_link_budget,
        ).pack(anchor="w", pady=(0, 8))
        ttk.Label(
            self.coverage_tab,
            textvariable=self.coverage_result_var,
            justify="left",
            wraplength=850,
        ).pack(fill="x", anchor="w", pady=(0, 8))
        self.coverage_canvas = tk.Canvas(
            self.coverage_tab,
            height=390,
            background="#f8fafc",
            highlightthickness=1,
            highlightbackground="#a3a3a3",
        )
        self.coverage_canvas.pack(fill="both", expand=True)
        self.coverage_canvas.bind("<Configure>", self._redraw_coverage)
        ttk.Label(
            self.coverage_tab,
            text="Illustration only: omnidirectional, flat free-space estimate. "
            "It is not a terrain map or guaranteed coverage boundary.",
            wraplength=850,
        ).pack(anchor="w", pady=(8, 0))

    def _draw_coverage(self, result: dict[str, float]) -> None:
        radius_km = result["free_space_range_km"]
        self.coverage_result_var.set(
            f"Estimated free-space range to receiver sensitivity: {radius_km:.2f} km\n"
            f"Current path length: {result['distance_m'] / 1000:.2f} km | "
            f"Estimated receive power: {result['received_dbm']:.1f} dBm | "
            f"Fade margin: {result['fade_margin_db']:.1f} dB"
        )
        self._redraw_coverage()

    def _redraw_coverage(self, _event: tk.Event[Any] | None = None) -> None:
        del _event
        canvas = getattr(self, "coverage_canvas", None)
        if canvas is None:
            return
        width, height = max(canvas.winfo_width(), 400), max(canvas.winfo_height(), 280)
        canvas.delete("all")
        center_x, center_y = width / 2, height / 2
        radius = min(width, height) * 0.37
        for fraction in (1.0, 0.66, 0.33):
            circle_radius = radius * fraction
            canvas.create_oval(
                center_x - circle_radius,
                center_y - circle_radius,
                center_x + circle_radius,
                center_y + circle_radius,
                outline="#2563eb" if fraction == 1.0 else "#94a3b8",
                width=2 if fraction == 1.0 else 1,
                dash=() if fraction == 1.0 else (4, 3),
            )
        canvas.create_line(
            center_x - radius, center_y, center_x + radius, center_y, fill="#cbd5e1"
        )
        canvas.create_line(
            center_x, center_y - radius, center_x, center_y + radius, fill="#cbd5e1"
        )
        canvas.create_oval(
            center_x - 5,
            center_y - 5,
            center_x + 5,
            center_y + 5,
            fill="#dc2626",
            outline="",
        )
        canvas.create_text(center_x, center_y + 16, text="TX", anchor="n")
        try:
            range_km = float(
                self.coverage_result_var.get().split(":", 1)[1].split(" km", 1)[0]
            )
        except (ValueError, IndexError):
            range_km = 0.0
        canvas.create_text(
            center_x,
            15,
            text=(
                f"Omnidirectional free-space estimate — {range_km:.2f} km"
                if range_km
                else "Calculate a link budget to show estimated range"
            ),
            anchor="n",
        )


def main() -> None:
    root = tk.Tk()
    RadioDashboard(root)
    root.mainloop()


if __name__ == "__main__":
    main()
