"""Tkinter dashboard, interactive coverage map, and terrain-profile UI."""

from __future__ import annotations

import math
import os
import queue
import threading
from typing import Any, Callable

import matplotlib

matplotlib.use("TkAgg")

import tkinter as tk
from tkinter import ttk

import numpy as np
import serial
import tkintermapview
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from serial.tools import list_ports

from .coverage_model import (
    MIN_REQUIRED_LORA_SNR_DB,
    RADIO_SENSITIVITY_DBM,
    TELEMETRY,
    TELEMETRY_LOCK,
    AnalyzerWorker,
    MeshtasticListener,
    fetch_elevation_profile,
    generate_coverage_grid,
    set_status,
)

DEFAULT_MAP_CENTER = (39.5, -98.35)
DEFAULT_MAP_ZOOM = 4
APP_BACKGROUND = "#f3f6fb"
SURFACE_BACKGROUND = "#ffffff"
TEXT_PRIMARY = "#172033"
TEXT_MUTED = "#667085"
ACCENT = "#2563eb"


# Tk event handlers live here; worker threads communicate through queues.
class RFDesktopApp:
    """Own the Tk UI and coordinate the independent hardware/network workers."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Radio Performance & Coverage Monitor")
        self.root.geometry("1400x900")
        self.root.minsize(1100, 720)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.ui_queue: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.mesh_listener: MeshtasticListener | None = None
        self.analyzer_workers: dict[str, tuple[threading.Event, AnalyzerWorker]] = {}
        self.terrain_thread: threading.Thread | None = None
        self.coverage_thread: threading.Thread | None = None
        self.coverage_generation = 0
        self.terrain_generation = 0
        self._input_signature: tuple[Any, ...] | None = None
        self.closing = False
        self._map_next_point_is_start = True
        self.home_marker: Any = None
        self.target_marker: Any = None
        self.link_path: Any = None
        self.link_los_blocked: bool | None = None
        self.coverage_polygons: list[Any] = []

        self.home_lat_var = tk.StringVar(value=os.getenv("HOME_LAT", "0"))
        self.home_lon_var = tk.StringVar(value=os.getenv("HOME_LON", "0"))
        self.target_lat_var = tk.StringVar(value="")
        self.target_lon_var = tk.StringVar(value="")
        self.mesh_port_var = tk.StringVar(value=os.getenv("MESHTASTIC_PORT", ""))
        self.nanovna_port_var = tk.StringVar(value=os.getenv("NANOVNA_PORT", ""))
        self.tinysa_port_var = tk.StringVar(value=os.getenv("TINYSA_PORT", ""))
        self.frequency_var = tk.StringVar(value=os.getenv("RF_FREQUENCY_MHZ", "915"))
        self.tx_power_var = tk.DoubleVar(value=30.0)
        self.tx_height_var = tk.DoubleVar(value=10.0)
        self.node_height_var = tk.DoubleVar(value=2.0)
        self.tx_gain_var = tk.DoubleVar(value=2.0)
        self.rx_gain_var = tk.DoubleVar(value=2.0)
        self.noise_snr_var = tk.DoubleVar(value=MIN_REQUIRED_LORA_SNR_DB)
        self.sensitivity_var = tk.DoubleVar(value=RADIO_SENSITIVITY_DBM)

        self._configure_styles()
        self._build_ui()
        self.refresh_ports()
        self._set_initial_map_position()
        self.root.after(250, self.poll_background)
        self.root.after(1000, self.refresh_dashboard)
        self.root.after(500, self.schedule_coverage_update)

    def _configure_styles(self) -> None:
        style = ttk.Style(self.root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        self.root.configure(background=APP_BACKGROUND)
        style.configure(".", font=("Segoe UI", 10), foreground=TEXT_PRIMARY)
        style.configure("TFrame", background=APP_BACKGROUND)
        style.configure("Surface.TFrame", background=SURFACE_BACKGROUND)
        style.configure("Card.TFrame", background=SURFACE_BACKGROUND)
        style.configure(
            "Surface.TLabelframe",
            background=SURFACE_BACKGROUND,
            bordercolor="#d9e1ed",
            relief="solid",
        )
        style.configure(
            "Surface.TLabelframe.Label",
            font=("Segoe UI", 10, "bold"),
            foreground=TEXT_PRIMARY,
            background=SURFACE_BACKGROUND,
        )
        style.configure(
            "Title.TLabel",
            font=("Segoe UI", 23, "bold"),
            foreground=TEXT_PRIMARY,
            background=APP_BACKGROUND,
        )
        style.configure(
            "Subtitle.TLabel",
            font=("Segoe UI", 10),
            foreground=TEXT_MUTED,
            background=APP_BACKGROUND,
        )
        style.configure(
            "Section.TLabel",
            font=("Segoe UI", 12, "bold"),
            foreground=TEXT_PRIMARY,
            background=APP_BACKGROUND,
        )
        style.configure(
            "FieldHeading.TLabel",
            font=("Segoe UI", 9, "bold"),
            foreground=TEXT_MUTED,
            background=APP_BACKGROUND,
        )
        style.configure(
            "ScaleValue.TLabel",
            font=("Segoe UI", 9, "bold"),
            foreground=ACCENT,
            background=APP_BACKGROUND,
        )
        style.configure(
            "CardTitle.TLabel",
            font=("Segoe UI", 9, "bold"),
            foreground=TEXT_MUTED,
            background=SURFACE_BACKGROUND,
        )
        style.configure(
            "MetricValue.TLabel",
            font=("Segoe UI", 21, "bold"),
            foreground=TEXT_PRIMARY,
            background=SURFACE_BACKGROUND,
        )
        style.configure("MetricName.TLabel", font=("Segoe UI", 10))
        style.configure("Good.TLabel", foreground="#166534", background=APP_BACKGROUND)
        style.configure(
            "Warning.TLabel", foreground="#9a3412", background=APP_BACKGROUND
        )
        style.configure("Error.TLabel", foreground="#b91c1c", background=APP_BACKGROUND)
        style.configure(
            "TLabelFrame",
            background=APP_BACKGROUND,
            bordercolor="#d9e1ed",
            relief="solid",
        )
        style.configure(
            "TLabelframe.Label",
            font=("Segoe UI", 10, "bold"),
            foreground=TEXT_PRIMARY,
            background=APP_BACKGROUND,
        )
        style.configure("TButton", padding=(12, 7), background="#e8edf5", borderwidth=0)
        style.map(
            "TButton",
            background=[("active", "#dbe5f2"), ("pressed", "#cbd8e8")],
            foreground=[("disabled", "#98a2b3")],
        )
        style.configure(
            "Primary.TButton",
            padding=(14, 8),
            background=ACCENT,
            foreground="#ffffff",
            borderwidth=0,
            font=("Segoe UI", 10, "bold"),
        )
        style.map(
            "Primary.TButton",
            background=[("active", "#1d4ed8"), ("pressed", "#1e40af")],
            foreground=[("disabled", "#dbeafe")],
        )
        style.configure("TNotebook", background=APP_BACKGROUND, borderwidth=0)
        style.configure(
            "TNotebook.Tab",
            padding=(16, 9),
            background="#e8edf5",
            foreground=TEXT_MUTED,
            font=("Segoe UI", 10, "bold"),
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", SURFACE_BACKGROUND), ("active", "#e0e8f3")],
            foreground=[("selected", ACCENT), ("active", TEXT_PRIMARY)],
        )
        style.configure(
            "Horizontal.TScale", background=APP_BACKGROUND, troughcolor="#dbe4f0"
        )
        style.configure(
            "TPanedwindow", background=APP_BACKGROUND, sashwidth=8, sashrelief="flat"
        )
        style.configure("TSeparator", background="#d9e1ed", foreground="#d9e1ed")
        style.configure(
            "Step.TLabel",
            font=("Segoe UI", 10, "bold"),
            foreground=ACCENT,
            background="#eaf1ff",
            padding=(9, 7),
        )
        style.configure(
            "LineStatus.TLabel",
            font=("Segoe UI", 10, "bold"),
            foreground=TEXT_PRIMARY,
            background=SURFACE_BACKGROUND,
            padding=(8, 5),
        )
        style.configure(
            "LineGood.TLabel",
            font=("Segoe UI", 10, "bold"),
            foreground="#15803d",
            background=SURFACE_BACKGROUND,
            padding=(8, 5),
        )
        style.configure(
            "LineBlocked.TLabel",
            font=("Segoe UI", 10, "bold"),
            foreground="#dc2626",
            background=SURFACE_BACKGROUND,
            padding=(8, 5),
        )

    def _build_ui(self) -> None:
        container = ttk.Frame(self.root, padding=(22, 18))
        container.pack(fill="both", expand=True)
        header = ttk.Frame(container)
        header.pack(fill="x", pady=(0, 16))
        ttk.Label(header, text="Radio Range Monitor", style="Title.TLabel").pack(
            anchor="w"
        )
        ttk.Label(
            header,
            text="Live telemetry, RF coverage planning, and terrain line-of-sight",
            style="Subtitle.TLabel",
        ).pack(anchor="w", pady=(3, 0))
        self.notebook = ttk.Notebook(container)
        self.notebook.pack(fill="both", expand=True)
        self.dashboard_tab = ttk.Frame(self.notebook, padding=18)
        self.map_tab = ttk.Frame(self.notebook, padding=12)
        self.notebook.add(self.dashboard_tab, text="Overview")
        self.notebook.add(self.map_tab, text="Map & Terrain")
        self._build_dashboard_tab()
        self._build_map_tab()

    def _build_dashboard_tab(self) -> None:
        ports_frame = ttk.LabelFrame(
            self.dashboard_tab, text="  DEVICE CONNECTIONS  ", padding=(14, 10)
        )
        ports_frame.pack(fill="x", pady=(0, 14))
        port_rows = (
            ("Meshtastic node", self.mesh_port_var, "mesh"),
            ("NanoVNA", self.nanovna_port_var, "nanovna"),
            ("tinySA", self.tinysa_port_var, "tinysa"),
        )
        self.port_boxes: dict[str, ttk.Combobox] = {}
        self.connection_buttons: dict[str, ttk.Button] = {}
        for row, (label, variable, role) in enumerate(port_rows):
            ttk.Label(ports_frame, text=f"{label}:").grid(
                row=row, column=0, sticky="w", pady=3
            )
            combo = ttk.Combobox(
                ports_frame, textvariable=variable, state="normal", width=24
            )
            combo.grid(row=row, column=1, sticky="ew", padx=8, pady=3)
            self.port_boxes[role] = combo
            button = ttk.Button(
                ports_frame,
                text="Connect" if role == "mesh" else "Start",
                style="Primary.TButton",
                command=(
                    self.connect_meshtastic
                    if role == "mesh"
                    else lambda selected_role=role: self.start_analyzer(selected_role)
                ),
            )
            button.grid(row=row, column=2, padx=4, pady=3)
            self.connection_buttons[role] = button
            if role == "mesh":
                ttk.Button(
                    ports_frame, text="Disconnect", command=self.disconnect_meshtastic
                ).grid(row=row, column=3, padx=4)
        ttk.Button(ports_frame, text="Refresh ports", command=self.refresh_ports).grid(
            row=3, column=2, sticky="e", pady=(6, 0)
        )
        ports_frame.columnconfigure(1, weight=1)

        ttk.Label(
            self.dashboard_tab, text="Live telemetry", style="Section.TLabel"
        ).pack(anchor="w", pady=(2, 8))
        metrics = ttk.Frame(self.dashboard_tab)
        metrics.pack(fill="x", pady=(0, 14))
        self.metric_vars: dict[str, tk.StringVar] = {}
        for column, (key, title) in enumerate(
            (
                ("rssi_dbm", "Meshtastic RSSI"),
                ("snr_db", "Meshtastic SNR"),
                ("swr", "NanoVNA SWR"),
                ("noise_floor_dbm", "tinySA Noise Floor"),
                ("distance_m", "Node Distance"),
            )
        ):
            card = ttk.Frame(
                metrics,
                style="Card.TFrame",
                padding=(12, 11),
                relief="solid",
                borderwidth=1,
            )
            card.grid(row=0, column=column, sticky="nsew", padx=4, ipady=3)
            ttk.Label(card, text=title.upper(), style="CardTitle.TLabel").pack(
                anchor="w"
            )
            variable = tk.StringVar(value="—")
            self.metric_vars[key] = variable
            ttk.Label(card, textvariable=variable, style="MetricValue.TLabel").pack(
                anchor="w", pady=(10, 4)
            )
            metrics.columnconfigure(column, weight=1)

        self.hardware_status_var = tk.StringVar(value="Initializing...")
        ttk.Label(
            self.dashboard_tab,
            textvariable=self.hardware_status_var,
            style="Subtitle.TLabel",
            wraplength=1200,
        ).pack(fill="x", anchor="w", pady=(0, 12))
        recommendations = ttk.LabelFrame(
            self.dashboard_tab,
            text="  RADIO HEALTH & SUGGESTIONS  ",
            padding=(14, 10),
        )
        recommendations.pack(fill="both", expand=True)
        self.recommendations = tk.Text(
            recommendations,
            height=7,
            wrap="word",
            state="disabled",
            font=("Segoe UI", 10),
            background=SURFACE_BACKGROUND,
            foreground=TEXT_PRIMARY,
            insertbackground=ACCENT,
            selectbackground="#dbeafe",
            selectforeground=TEXT_PRIMARY,
            relief="flat",
            borderwidth=0,
            padx=8,
            pady=8,
            highlightthickness=0,
        )
        self.recommendations.pack(fill="both", expand=True)
        ttk.Label(
            self.dashboard_tab,
            text="Choose a real profile start point on the map for localized range measurements.",
            style="Subtitle.TLabel",
            wraplength=1200,
        ).pack(anchor="w", pady=(12, 0))

    def _build_map_tab(self) -> None:
        body = ttk.Frame(self.map_tab)
        body.pack(fill="both", expand=True)
        controls_container = ttk.Frame(body, width=310)
        controls_container.pack(side="left", fill="y")
        controls_canvas = tk.Canvas(
            controls_container,
            width=290,
            highlightthickness=0,
            borderwidth=0,
            background=APP_BACKGROUND,
        )
        controls_scrollbar = ttk.Scrollbar(
            controls_container, orient="vertical", command=controls_canvas.yview
        )
        controls_canvas.configure(yscrollcommand=controls_scrollbar.set)
        controls_scrollbar.pack(side="right", fill="y")
        controls_canvas.pack(side="left", fill="y", expand=True)
        controls = ttk.Frame(controls_canvas, padding=(4, 4, 12, 4))
        controls_window = controls_canvas.create_window(
            (0, 0), window=controls, anchor="nw"
        )
        controls.bind(
            "<Configure>",
            lambda event: controls_canvas.configure(
                scrollregion=(0, 0, 0, event.widget.winfo_reqheight())
            ),
        )
        controls_canvas.bind(
            "<Configure>",
            lambda event: controls_canvas.itemconfigure(
                controls_window, width=event.width
            ),
        )
        map_frame = ttk.Frame(body)
        map_frame.pack(side="left", fill="both", expand=True)
        map_and_profile = ttk.Panedwindow(map_frame, orient=tk.VERTICAL)
        map_and_profile.pack(fill="both", expand=True)
        map_view_frame = ttk.Frame(map_and_profile, style="Surface.TFrame", padding=4)
        terrain_frame = ttk.LabelFrame(
            map_and_profile,
            text="  TERRAIN PROFILE  ",
            padding=(10, 7),
            style="Surface.TLabelframe",
        )
        map_and_profile.add(map_view_frame, weight=3)
        map_and_profile.add(terrain_frame, weight=2)

        ttk.Label(controls, text="PROFILE", style="Section.TLabel").pack(
            anchor="w", pady=(0, 4)
        )
        self.point_selection_status_var = tk.StringVar(
            value="Step 1 of 2 — click the map to select a start point."
        )
        ttk.Label(
            controls,
            textvariable=self.point_selection_status_var,
            style="Step.TLabel",
            wraplength=270,
            justify="left",
        ).pack(anchor="w", pady=(0, 6))
        ttk.Label(
            controls,
            text="Click once for the start, then again for the end. The terrain "
            "profile loads automatically. You can also enter coordinates below.",
            wraplength=270,
        ).pack(anchor="w", pady=(0, 8))
        ttk.Label(controls, text="START POINT", style="FieldHeading.TLabel").pack(
            anchor="w", pady=(3, 1)
        )
        self._coordinate_field(
            controls,
            "Latitude",
            self.home_lat_var,
            self.apply_home_coordinates,
        )
        self._coordinate_field(
            controls,
            "Longitude",
            self.home_lon_var,
            self.apply_home_coordinates,
        )
        ttk.Button(
            controls,
            text="Set start point",
            style="Primary.TButton",
            command=self.apply_home_coordinates,
        ).pack(fill="x", pady=(5, 10))
        ttk.Separator(controls).pack(fill="x", pady=4)
        ttk.Label(controls, text="END POINT", style="FieldHeading.TLabel").pack(
            anchor="w", pady=(5, 1)
        )
        self._coordinate_field(
            controls,
            "Latitude",
            self.target_lat_var,
            self.apply_target_coordinates,
        )
        self._coordinate_field(
            controls,
            "Longitude",
            self.target_lon_var,
            self.apply_target_coordinates,
        )
        ttk.Button(
            controls,
            text="Set end point",
            command=self.apply_target_coordinates,
        ).pack(fill="x", pady=(5, 5))
        ttk.Button(
            controls,
            text="Use latest Meshtastic GPS",
            command=self.use_live_node_position,
        ).pack(fill="x", pady=(0, 10))
        height_row = ttk.Frame(controls)
        height_row.pack(fill="x", pady=(0, 8))
        ttk.Label(height_row, text="Node height AGL (m):").pack(side="left")
        node_height = ttk.Spinbox(
            height_row,
            textvariable=self.node_height_var,
            from_=0.5,
            to=100,
            increment=0.5,
            width=8,
            command=self._coverage_control_changed,
        )
        node_height.pack(side="right")
        node_height.bind("<Return>", self._coverage_control_changed)
        node_height.bind("<FocusOut>", self._coverage_control_changed)

        ttk.Separator(controls).pack(fill="x", pady=5)
        ttk.Label(controls, text="RF coverage model", style="MetricName.TLabel").pack(
            anchor="w", pady=(5, 2)
        )
        self._scale_control(
            controls, "TX antenna height AGL (m)", self.tx_height_var, 2, 100, 2
        )
        self._scale_control(
            controls, "TX antenna gain (dBi)", self.tx_gain_var, -5, 15, 1
        )
        self._scale_control(
            controls, "RX antenna gain (dBi)", self.rx_gain_var, -5, 15, 1
        )
        self._scale_control(controls, "TX power (dBm)", self.tx_power_var, 0, 30, 1)
        self._scale_control(
            controls,
            "Required LoRa SNR (dB)",
            self.noise_snr_var,
            -20,
            5,
            0.5,
        )
        self._scale_control(
            controls,
            "Radio sensitivity (dBm)",
            self.sensitivity_var,
            -140,
            -80,
            1,
        )
        ttk.Label(controls, text="Frequency (MHz)").pack(anchor="w", pady=(4, 0))
        frequency_box = ttk.Combobox(
            controls,
            textvariable=self.frequency_var,
            values=("868", "915"),
            state="normal",
        )
        frequency_box.pack(fill="x")
        frequency_box.bind("<<ComboboxSelected>>", self._coverage_control_changed)
        frequency_box.bind("<Return>", self._coverage_control_changed)
        ttk.Button(
            controls,
            text="Recalculate coverage",
            command=self.schedule_coverage_update,
        ).pack(fill="x", pady=(8, 5))

        ttk.Label(controls, text="Coverage layer:").pack(anchor="w", pady=(8, 2))
        legend = (
            ("#b7e4c7", "Clear (>10 dB margin)"),
            ("#fff1a8", "Fringe (0–10 dB margin)"),
            ("#f5c2c7", "Predicted blocked (<0 dB)"),
        )
        for color, text in legend:
            item = ttk.Frame(controls)
            item.pack(fill="x", pady=1)
            tk.Label(item, background=color, width=3, relief="solid").pack(
                side="left", padx=(0, 6)
            )
            ttk.Label(item, text=text, wraplength=235).pack(side="left")
        self.coverage_status_var = tk.StringVar(value="Coverage not calculated.")
        ttk.Label(
            controls,
            textvariable=self.coverage_status_var,
            wraplength=270,
            justify="left",
        ).pack(fill="x", anchor="w", pady=(10, 0))

        with TELEMETRY_LOCK:
            center_lat = float(TELEMETRY["home_lat"])
            center_lon = float(TELEMETRY["home_lon"])
        self.map_widget = tkintermapview.TkinterMapView(map_view_frame, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_position(center_lat, center_lon)
        self.map_widget.set_zoom(12)
        self.map_widget.add_left_click_map_command(self._map_select_profile_point)
        self.map_widget.add_right_click_menu_command(
            label="Set Home Base here",
            command=self._map_set_home,
            pass_coords=True,
        )
        self.map_widget.add_right_click_menu_command(
            label="Place virtual target here",
            command=self._map_set_target,
            pass_coords=True,
        )
        self.map_link_status_var = tk.StringVar(value="Link LOS: select two points.")
        self.map_link_status_label = ttk.Label(
            map_view_frame,
            textvariable=self.map_link_status_var,
            style="LineStatus.TLabel",
            anchor="e",
        )
        self.map_link_status_label.pack(fill="x")
        ttk.Label(
            map_view_frame,
            text="Map tiles © OpenStreetMap contributors | Hata overlay is an urban-model estimate.",
            anchor="e",
        ).pack(fill="x")
        self._build_terrain_panel(terrain_frame)

    @staticmethod
    def _coordinate_field(
        parent: ttk.Frame,
        label: str,
        variable: tk.StringVar,
        on_enter: Callable[[], None] | None = None,
    ) -> None:
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=label, width=11).pack(side="left")
        entry = ttk.Entry(row, textvariable=variable)
        entry.pack(side="left", fill="x", expand=True)
        if on_enter is not None:
            entry.bind(
                "<Return>",
                lambda event: RFDesktopApp._submit_coordinate(event, on_enter),
            )

    @staticmethod
    def _submit_coordinate(event: tk.Event, callback: Callable[[], None]) -> str:
        del event
        callback()
        return "break"

    def _scale_control(
        self,
        parent: ttk.Frame,
        label: str,
        variable: tk.DoubleVar,
        minimum: float,
        maximum: float,
        resolution: float,
    ) -> None:
        heading = ttk.Frame(parent)
        heading.pack(fill="x", pady=(7, 0))
        ttk.Label(heading, text=label, wraplength=195).pack(side="left", anchor="w")
        value_var = tk.StringVar(value=f"{variable.get():g}")
        ttk.Label(heading, textvariable=value_var, style="ScaleValue.TLabel").pack(
            side="right"
        )
        scale = ttk.Scale(
            parent,
            variable=variable,
            from_=minimum,
            to=maximum,
            orient="horizontal",
            command=lambda value: self._scale_value_changed(
                value, value_var, variable, minimum, resolution
            ),
        )
        scale.pack(fill="x", pady=(1, 0))
        bounds = ttk.Frame(parent)
        bounds.pack(fill="x")
        ttk.Label(bounds, text=f"{minimum:g}", style="Subtitle.TLabel").pack(
            side="left"
        )
        ttk.Label(bounds, text=f"{maximum:g}", style="Subtitle.TLabel").pack(
            side="right"
        )

    def _scale_value_changed(
        self,
        value: str,
        display_var: tk.StringVar,
        variable: tk.DoubleVar,
        minimum: float,
        resolution: float,
    ) -> None:
        numeric_value = float(value)
        snapped = minimum + round((numeric_value - minimum) / resolution) * resolution
        variable.set(snapped)
        display_var.set(f"{snapped:g}")
        self.schedule_coverage_update()

    def _build_terrain_panel(self, parent: ttk.Frame) -> None:
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill="x", pady=(0, 8))
        ttk.Button(
            toolbar,
            text="Build profile",
            style="Primary.TButton",
            command=self.fetch_terrain,
        ).pack(side="left")
        ttk.Button(
            toolbar,
            text="Reset points",
            command=self.reset_profile_points,
        ).pack(side="left", padx=(6, 0))
        self.terrain_status_var = tk.StringVar(
            value="Select two points on the map to build a terrain profile."
        )
        ttk.Label(
            toolbar,
            textvariable=self.terrain_status_var,
            wraplength=900,
        ).pack(side="left", padx=12, fill="x", expand=True)
        self.figure = Figure(figsize=(9, 2.5), dpi=100, tight_layout=True)
        self.axes = self.figure.add_subplot(111)
        self.axes.set_title("Terrain cross-section")
        self.axes.set_xlabel("Distance from start point (m)")
        self.axes.set_ylabel("Elevation (m)")
        self._style_terrain_axes()
        self.figure_canvas = FigureCanvasTkAgg(self.figure, master=parent)
        self.figure_canvas.get_tk_widget().pack(fill="both", expand=True)
        ttk.Label(
            parent,
            text="Terrain data: Open-Meteo / Copernicus DEM. Curvature-adjusted terrain "
            "uses Earth radius 6,371,000 m; LOS is green when clear and red when blocked.",
            wraplength=1200,
        ).pack(anchor="w", pady=(6, 0))

    def _set_initial_map_position(self) -> None:
        self.apply_home_coordinates(initial=True)
        self.map_widget.set_position(*DEFAULT_MAP_CENTER)
        self.map_widget.set_zoom(DEFAULT_MAP_ZOOM)

    def refresh_ports(self) -> None:
        try:
            ports = list(list_ports.comports())
            names = [port.device for port in ports]
            for combo in self.port_boxes.values():
                combo.configure(values=names)
            auto_roles: dict[str, str] = {}
            for port in ports:
                signature = " ".join(
                    str(value or "")
                    for value in (
                        port.description,
                        port.manufacturer,
                        port.product,
                    )
                ).lower()
                if "tinysa" in signature or "tiny sa" in signature:
                    auto_roles["tinysa"] = port.device
                elif "nanovna" in signature or "nano vna" in signature:
                    auto_roles["nanovna"] = port.device
            for role, variable in (
                ("nanovna", self.nanovna_port_var),
                ("tinysa", self.tinysa_port_var),
            ):
                if not variable.get().strip() and role in auto_roles:
                    variable.set(auto_roles[role])
            descriptions = ", ".join(
                f"{port.device} ({port.description or 'serial'})" for port in ports
            )
            self.hardware_status_var.set(
                "Serial ports: " + descriptions
                if descriptions
                else "No serial ports detected."
            )
        except (serial.SerialException, OSError) as exc:
            self.hardware_status_var.set(f"Could not enumerate serial ports: {exc}")

    def connect_meshtastic(self) -> None:
        port = self.mesh_port_var.get().strip() or None
        if port and port in (
            self.nanovna_port_var.get().strip(),
            self.tinysa_port_var.get().strip(),
        ):
            set_status(
                "mesh_status", "Choose a serial port not assigned to an analyzer."
            )
            return
        if not self.disconnect_meshtastic():
            return
        self.mesh_listener = MeshtasticListener(port)
        self.mesh_listener.start()

    def disconnect_meshtastic(self) -> bool:
        if self.mesh_listener is not None:
            listener = self.mesh_listener
            listener.stop()
            listener.join(timeout=3)
            if listener.is_alive():
                set_status(
                    "mesh_status",
                    "Disconnect requested; waiting for serial connection to stop.",
                )
                return False
            self.mesh_listener = None
        set_status("mesh_status", "Disconnected")
        return True

    def start_analyzer(self, role: str) -> None:
        variable = self.nanovna_port_var if role == "nanovna" else self.tinysa_port_var
        port = variable.get().strip()
        if not port:
            set_status(f"{role}_status", "Select a serial port first.")
            return
        if port in (
            self.mesh_port_var.get().strip(),
            (
                self.tinysa_port_var.get().strip()
                if role == "nanovna"
                else self.nanovna_port_var.get().strip()
            ),
        ):
            set_status(f"{role}_status", "Port is already assigned to another device.")
            return
        try:
            frequency = float(self.frequency_var.get())
            if not 150 <= frequency <= 1500:
                raise ValueError
        except ValueError:
            set_status(f"{role}_status", "Frequency must be between 150 and 1500 MHz.")
            return

        existing = self.analyzer_workers.get(role)
        if existing is not None:
            existing[0].set()
            existing[1].join(timeout=2)
            if existing[1].is_alive():
                set_status(
                    f"{role}_status",
                    "Previous sweep is still stopping; wait before reconnecting.",
                )
                return
        stop_event = threading.Event()
        worker = AnalyzerWorker(role, port, stop_event, frequency)
        self.analyzer_workers[role] = (stop_event, worker)
        set_status(f"{role}_status", f"Starting on {port}...")
        worker.start()

    def _map_set_home(self, coordinates: tuple[float, float]) -> None:
        self.home_lat_var.set(f"{coordinates[0]:.6f}")
        self.home_lon_var.set(f"{coordinates[1]:.6f}")
        self.apply_home_coordinates()

    def _map_set_target(self, coordinates: tuple[float, float]) -> None:
        self.target_lat_var.set(f"{coordinates[0]:.6f}")
        self.target_lon_var.set(f"{coordinates[1]:.6f}")
        self.apply_target_coordinates()

    def _map_select_profile_point(self, coordinates: tuple[float, float]) -> None:
        if self._map_next_point_is_start:
            self._map_set_home(coordinates)
            self._clear_profile_result()
            if self.target_marker is not None:
                self.target_marker.delete()
                self.target_marker = None
            if self.link_path is not None:
                self.link_path.delete()
                self.link_path = None
            self.target_lat_var.set("")
            self.target_lon_var.set("")
            self.link_los_blocked = None
            self.map_link_status_var.set("Link LOS: select an end point.")
            self.map_link_status_label.configure(style="LineStatus.TLabel")
            self.point_selection_status_var.set(
                "Step 2 of 2 — click the map to select an end point."
            )
            self.terrain_status_var.set(
                "Start point selected. Click the map again to choose the end point."
            )
            self._map_next_point_is_start = False
            return

        self._map_set_target(coordinates)
        self._map_next_point_is_start = True
        self.point_selection_status_var.set(
            "Both points selected — requesting the terrain profile."
        )
        self.fetch_terrain()

    def reset_profile_points(self) -> None:
        self._map_next_point_is_start = True
        self._clear_profile_result()
        self.coverage_generation += 1
        self._input_signature = None
        try:
            if self._coverage_after_id is not None:
                self.root.after_cancel(self._coverage_after_id)
                self._coverage_after_id = None
        except (AttributeError, tk.TclError):
            pass

        for marker_name in ("home_marker", "target_marker", "link_path"):
            marker = getattr(self, marker_name)
            if marker is not None:
                marker.delete()
                setattr(self, marker_name, None)

        self.home_lat_var.set("")
        self.home_lon_var.set("")
        self.target_lat_var.set("")
        self.target_lon_var.set("")
        self.link_los_blocked = None
        self.map_link_status_var.set("Link LOS: select two points.")
        self.map_link_status_label.configure(style="LineStatus.TLabel")
        self.point_selection_status_var.set(
            "Step 1 of 2 — click the map to select a start point."
        )
        with TELEMETRY_LOCK:
            TELEMETRY["home_lat"] = float(os.getenv("HOME_LAT", "0"))
            TELEMETRY["home_lon"] = float(os.getenv("HOME_LON", "0"))

        for polygon in self.coverage_polygons:
            polygon.delete()
        self.coverage_polygons.clear()
        self.coverage_status_var.set(
            "Select a start point on the map to recalculate coverage."
        )

        self.terrain_status_var.set(
            "Points cleared. Click the map to select a start point, then an end point."
        )

    def _clear_profile_result(self) -> None:
        self.terrain_generation += 1
        self.axes.clear()
        self.axes.set_title("Terrain cross-section")
        self.axes.set_xlabel("Distance from start point (m)")
        self.axes.set_ylabel("Elevation (m)")
        self._style_terrain_axes()
        self.figure_canvas.draw_idle()

    def _style_terrain_axes(self) -> None:
        self.figure.set_facecolor(SURFACE_BACKGROUND)
        self.axes.set_facecolor(SURFACE_BACKGROUND)
        self.axes.tick_params(colors=TEXT_MUTED, labelsize=8)
        self.axes.xaxis.label.set_color(TEXT_MUTED)
        self.axes.yaxis.label.set_color(TEXT_MUTED)
        self.axes.title.set_color(TEXT_PRIMARY)
        self.axes.title.set_fontsize(11)
        self.axes.grid(True, color="#dfe5ee", linewidth=0.8, alpha=0.8)
        for side in ("top", "right"):
            self.axes.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            self.axes.spines[side].set_color("#cbd5e1")

    @staticmethod
    def _parse_coordinates(lat_text: str, lon_text: str) -> tuple[float, float]:
        latitude, longitude = float(lat_text), float(lon_text)
        if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
            raise ValueError("Latitude must be -90..90; longitude -180..180.")
        return latitude, longitude

    def apply_home_coordinates(self, initial: bool = False) -> None:
        try:
            latitude, longitude = self._parse_coordinates(
                self.home_lat_var.get(), self.home_lon_var.get()
            )
        except ValueError as exc:
            if not initial:
                self.coverage_status_var.set(f"Invalid Home Base: {exc}")
            return
        with TELEMETRY_LOCK:
            TELEMETRY["home_lat"] = latitude
            TELEMETRY["home_lon"] = longitude
        if self.home_marker is not None:
            self.home_marker.delete()
        self.home_marker = self.map_widget.set_marker(
            latitude, longitude, text="Home Base", marker_color_circle="#166534"
        )
        if not initial:
            self._map_next_point_is_start = False
            self.point_selection_status_var.set(
                "Start point set — select or enter the end point."
            )
        if initial:
            self.map_widget.set_position(latitude, longitude)
        self.schedule_coverage_update()

    def apply_target_coordinates(self) -> None:
        try:
            latitude, longitude = self._parse_coordinates(
                self.target_lat_var.get(), self.target_lon_var.get()
            )
        except ValueError as exc:
            self.terrain_status_var.set(f"Invalid target coordinates: {exc}")
            return
        if self.target_marker is not None:
            self.target_marker.delete()
        self.target_marker = self.map_widget.set_marker(
            latitude, longitude, text="Virtual target", marker_color_circle="#1d4ed8"
        )
        if self.link_path is not None:
            self.link_path.delete()
        with TELEMETRY_LOCK:
            home_lat, home_lon = TELEMETRY["home_lat"], TELEMETRY["home_lon"]
        self.link_path = self.map_widget.set_path(
            [(home_lat, home_lon), (latitude, longitude)],
            color="#1d4ed8",
            width=3,
            name="Selected-point link (LOS pending)",
        )
        self.link_los_blocked = None
        self.map_link_status_var.set("Link LOS: waiting for terrain profile.")
        self.map_link_status_label.configure(style="LineStatus.TLabel")
        self._map_next_point_is_start = True
        self.point_selection_status_var.set(
            "Both points set — load the terrain profile or select a new start point."
        )
        self.map_widget.set_position(latitude, longitude)
        self.terrain_status_var.set(
            "Target set. Fetch the terrain profile to load 50 elevation samples."
        )

    def use_live_node_position(self) -> None:
        with TELEMETRY_LOCK:
            latitude, longitude = TELEMETRY["node_lat"], TELEMETRY["node_lon"]
        if latitude is None or longitude is None:
            self.terrain_status_var.set("No Meshtastic GPS position received yet.")
            return
        self.target_lat_var.set(f"{latitude:.6f}")
        self.target_lon_var.set(f"{longitude:.6f}")
        self.apply_target_coordinates()

    def _coverage_control_changed(self, _value: str | None = None) -> None:
        del _value
        self.schedule_coverage_update()

    def schedule_coverage_update(self) -> None:
        if self.closing:
            return
        try:
            if self._coverage_after_id is not None:
                self.root.after_cancel(self._coverage_after_id)
        except (AttributeError, tk.TclError):
            pass
        self._coverage_after_id = self.root.after(300, self._start_coverage_calculation)

    def _start_coverage_calculation(self) -> None:
        try:
            home_lat, home_lon = self._parse_coordinates(
                self.home_lat_var.get(), self.home_lon_var.get()
            )
            frequency = float(self.frequency_var.get())
            if not 150 <= frequency <= 1500:
                raise ValueError("Use a frequency from 150 to 1500 MHz for Hata.")
            with TELEMETRY_LOCK:
                swr = TELEMETRY["swr"]
                noise = TELEMETRY["noise_floor_dbm"]
        except ValueError as exc:
            self.coverage_status_var.set(f"Coverage inputs invalid: {exc}")
            return

        self.coverage_generation += 1
        generation = self.coverage_generation
        parameters = (
            home_lat,
            home_lon,
            frequency,
            float(self.tx_power_var.get()),
            float(self.tx_gain_var.get()),
            float(self.rx_gain_var.get()),
            float(self.tx_height_var.get()),
            float(self.node_height_var.get()),
            float(swr) if swr is not None else None,
            float(noise) if noise is not None else None,
            float(self.sensitivity_var.get()),
            float(self.noise_snr_var.get()),
        )
        self._input_signature = self._input_signature_from_ui(
            parameters[8], parameters[9]
        )
        self.coverage_status_var.set("Calculating vectorized 15-km Hata grid...")

        def calculate() -> None:
            try:
                result = generate_coverage_grid(*parameters)
                self.ui_queue.put(("coverage", (generation, result, parameters)))
            except Exception as exc:
                self.ui_queue.put(("coverage_error", (generation, str(exc))))

        self.coverage_thread = threading.Thread(
            target=calculate, name=f"coverage-grid-{generation}", daemon=True
        )
        self.coverage_thread.start()

    def _apply_coverage_result(
        self,
        result: dict[str, Any],
        parameters: tuple[Any, ...],
    ) -> None:
        for polygon in self.coverage_polygons:
            try:
                polygon.delete()
            except (AttributeError, tk.TclError):
                pass
        self.coverage_polygons.clear()
        color_by_class = result["colors"]
        for category, positions in result["polygons"]:
            try:
                polygon = self.map_widget.set_polygon(
                    positions,
                    fill_color=color_by_class[category],
                    outline_color=color_by_class[category],
                    border_width=0,
                    name=f"rf-coverage-{self.coverage_generation}-{len(self.coverage_polygons)}",
                )
                self.coverage_polygons.append(polygon)
            except (tk.TclError, ValueError) as exc:
                self.coverage_status_var.set(f"Could not draw coverage layer: {exc}")
                break
        clear_count = result["counts"][2]
        fringe_count = result["counts"][1]
        blocked_count = result["counts"][0]
        noise_text = (
            f"{result['noise_floor_dbm']:.1f} dBm"
            if result["noise_floor_dbm"] is not None
            else "not yet measured (configured radio sensitivity used)"
        )
        swr_text = (
            f"{parameters[8]:.2f}:1"
            if parameters[8] is not None and math.isfinite(parameters[8])
            else (
                "not yet measured (matched-feed assumption)"
                if parameters[8] is None
                else "∞"
            )
        )
        out_of_height_range = not (
            30 <= parameters[6] <= 200 and 1 <= parameters[7] <= 10
        )
        height_note = (
            "Antenna height is outside the Hata model's recommended 30–200 m base / "
            "1–10 m mobile range. "
            if out_of_height_range
            else ""
        )
        model_note = (
            f"{height_note}Urban Okumura-Hata; distances under 1 km are clamped to "
            "1 km. Terrain/buildings/foliage are not included."
        )
        self.coverage_status_var.set(
            f"{result['grid_points']:,} grid points / 15 km radius | "
            f"clear {clear_count:,} | fringe {fringe_count:,} | "
            f"blocked {blocked_count:,}\n"
            f"Effective RX threshold {result['sensitivity_dbm']:.1f} dBm "
            f"(noise {noise_text}, required SNR {self.noise_snr_var.get():.1f} dB) | "
            f"SWR {swr_text}, mismatch loss {result['mismatch_loss_db']:.2f} dB\n"
            f"{model_note}"
        )

    def fetch_terrain(self) -> None:
        try:
            start = self._parse_coordinates(
                self.home_lat_var.get(), self.home_lon_var.get()
            )
            end = self._parse_coordinates(
                self.target_lat_var.get(), self.target_lon_var.get()
            )
            tx_height = float(self.tx_height_var.get())
            node_height = float(self.node_height_var.get())
            if node_height <= 0:
                raise ValueError("Node height must be greater than zero.")
        except ValueError as exc:
            self.terrain_status_var.set(f"Cannot fetch terrain: {exc}")
            return
        if self.terrain_thread is not None and self.terrain_thread.is_alive():
            self.terrain_status_var.set(
                "A terrain request is already running. Retry after it finishes."
            )
            return
        self.terrain_generation += 1
        generation = self.terrain_generation
        self.terrain_status_var.set("Requesting 50 elevation samples...")

        def fetch() -> None:
            try:
                distances, raw, adjusted = fetch_elevation_profile(
                    start[0], start[1], end[0], end[1], samples=50
                )
                line_of_sight = np.linspace(
                    raw[0] + tx_height,
                    adjusted[-1] + node_height,
                    len(distances),
                )
                self.ui_queue.put(
                    (
                        "terrain",
                        (generation, distances, raw, adjusted, line_of_sight),
                    )
                )
            except Exception as exc:
                self.ui_queue.put(("terrain_error", (generation, str(exc))))

        self.terrain_thread = threading.Thread(
            target=fetch, name="terrain-profile", daemon=True
        )
        self.terrain_thread.start()

    def _draw_terrain(
        self,
        distances: np.ndarray,
        raw: np.ndarray,
        adjusted: np.ndarray,
        line_of_sight: np.ndarray,
    ) -> None:
        total_distance = float(distances[-1])
        blocked = bool(np.any(line_of_sight[1:-1] < adjusted[1:-1]))
        los_color = "#dc2626" if blocked else "#16a34a"
        los_label = (
            "Optical line of sight (blocked)"
            if blocked
            else "Optical line of sight (clear)"
        )
        self.axes.clear()
        self.axes.fill_between(
            distances,
            adjusted,
            min(0.0, float(np.min(adjusted))),
            color="#a3a3a3",
            alpha=0.45,
            label="Curvature-adjusted terrain",
        )
        self.axes.plot(
            distances,
            adjusted,
            color="#4b5563",
            linewidth=1.5,
            label="Ground (Earth-curvature adjusted)",
        )
        self.axes.plot(
            distances,
            line_of_sight,
            color=los_color,
            linewidth=2,
            label=los_label,
        )
        self.axes.scatter(
            [distances[0], distances[-1]],
            [
                raw[0] + float(self.tx_height_var.get()),
                adjusted[-1] + float(self.node_height_var.get()),
            ],
            color=["#166534", "#1d4ed8"],
            zorder=5,
            label="Antenna endpoints",
        )
        self.axes.set_title("Selected Start → End Terrain Cross-Section")
        self.axes.set_xlabel("Distance from start point (m)")
        self.axes.set_ylabel("Elevation (m)")
        self._style_terrain_axes()
        self.axes.legend(loc="best")
        self.figure_canvas.draw_idle()
        self._set_link_line_of_sight(blocked)
        self.terrain_status_var.set(
            f"LOS {'BLOCKED' if blocked else 'CLEAR'} — "
            f"{total_distance / 1000:.2f} km; curvature included."
        )
        self.point_selection_status_var.set(
            "Profile ready — click the map to choose a new start point."
        )

    def _set_link_line_of_sight(self, blocked: bool) -> None:
        self.link_los_blocked = blocked
        if self.link_path is not None:
            self.link_path.delete()
        try:
            start = self._parse_coordinates(
                self.home_lat_var.get(), self.home_lon_var.get()
            )
            end = self._parse_coordinates(
                self.target_lat_var.get(), self.target_lon_var.get()
            )
        except ValueError as exc:
            self.map_link_status_var.set(f"Link LOS unavailable: {exc}")
            return

        status = "BLOCKED" if blocked else "CLEAR"
        color = "#dc2626" if blocked else "#16a34a"
        self.map_link_status_label.configure(
            style="LineBlocked.TLabel" if blocked else "LineGood.TLabel"
        )
        self.link_path = self.map_widget.set_path(
            [start, end],
            color=color,
            width=3,
            name=f"Selected-point link (LOS {status.lower()})",
        )
        self.map_link_status_var.set(f"Link LOS: {status}")

    def poll_background(self) -> None:
        if self.closing:
            return
        try:
            while True:
                event, payload = self.ui_queue.get_nowait()
                if event == "terrain":
                    generation, *terrain = payload
                    if generation == self.terrain_generation:
                        self._draw_terrain(*terrain)
                elif event == "terrain_error":
                    generation, message = payload
                    if generation == self.terrain_generation:
                        self.terrain_status_var.set(
                            f"Terrain request failed: {message}"
                        )
                        self.point_selection_status_var.set(
                            "Profile failed — click the map to choose a new start point."
                        )
                elif event == "coverage":
                    generation, result, parameters = payload
                    if generation == self.coverage_generation:
                        self._apply_coverage_result(result, parameters)
                elif event == "coverage_error":
                    generation, message = payload
                    if generation == self.coverage_generation:
                        self.coverage_status_var.set(
                            f"Coverage calculation failed: {message}"
                        )
        except queue.Empty:
            pass
        self.root.after(200, self.poll_background)

    def refresh_dashboard(self) -> None:
        if self.closing:
            return
        with TELEMETRY_LOCK:
            snapshot = dict(TELEMETRY)

        self.metric_vars["rssi_dbm"].set(
            f"{snapshot['rssi_dbm']:.1f} dBm"
            if snapshot["rssi_dbm"] is not None
            else "—"
        )
        self.metric_vars["snr_db"].set(
            f"{snapshot['snr_db']:.1f} dB" if snapshot["snr_db"] is not None else "—"
        )
        swr = snapshot["swr"]
        self.metric_vars["swr"].set(
            "—" if swr is None else f"{swr:.2f}:1" if math.isfinite(swr) else "∞"
        )
        self.metric_vars["noise_floor_dbm"].set(
            f"{snapshot['noise_floor_dbm']:.1f} dBm"
            if snapshot["noise_floor_dbm"] is not None
            else "—"
        )
        distance = snapshot["distance_m"]
        self.metric_vars["distance_m"].set(
            f"{distance / 1000:.3f} km" if distance is not None else "—"
        )
        statuses = (
            f"Meshtastic: {snapshot['mesh_status']} | "
            f"NanoVNA: {snapshot['nanovna_status']} | "
            f"tinySA: {snapshot['tinysa_status']}"
        )
        self.hardware_status_var.set(statuses)
        self._update_recommendations(snapshot)

        signature = self._input_signature_from_ui(
            snapshot["swr"], snapshot["noise_floor_dbm"]
        )
        if signature != self._input_signature:
            self._input_signature = signature
            self.schedule_coverage_update()
        self.root.after(1000, self.refresh_dashboard)

    def _input_signature_from_ui(
        self,
        swr: float | None = None,
        noise: float | None = None,
    ) -> tuple[Any, ...]:
        if swr is None and noise is None:
            with TELEMETRY_LOCK:
                swr, noise = TELEMETRY["swr"], TELEMETRY["noise_floor_dbm"]
        return (
            swr,
            noise,
            self.home_lat_var.get(),
            self.home_lon_var.get(),
            self.frequency_var.get(),
            self.tx_power_var.get(),
            self.tx_height_var.get(),
            self.node_height_var.get(),
            self.tx_gain_var.get(),
            self.rx_gain_var.get(),
            self.noise_snr_var.get(),
            self.sensitivity_var.get(),
        )

    def _update_recommendations(self, data: dict[str, Any]) -> None:
        messages: list[str] = []
        if data["swr"] is not None and data["swr"] > 1.5:
            messages.append(
                "• SWR is above 1.5:1. Tune the antenna and inspect feed-line "
                "connections; mismatch loss reduces transmitted power."
            )
        if data["noise_floor_dbm"] is not None and data["noise_floor_dbm"] > -100:
            messages.append(
                "• Noise floor is above -100 dBm. Check local RF interference; "
                "try a quieter channel or a suitable cavity bandpass filter."
            )
        history = data.get("history", [])
        if (
            data["swr"] is not None
            and data["swr"] <= 1.5
            and data["noise_floor_dbm"] is not None
            and data["noise_floor_dbm"] <= -100
            and len(history) >= 2
        ):
            newest = history[-1]
            if any(
                0 < (newest[0] - old[0]).total_seconds() <= 300
                and abs(newest[1] - old[1]) <= 500
                and old[2] - newest[2] >= 15
                for old in history[:-1]
            ):
                messages.append(
                    "• RSSI fell at least 15 dB over a short distance while SWR "
                    "and noise are good. Raise the antenna to clear terrain or "
                    "structural obstructions."
                )
        if not messages:
            messages.append(
                "No threshold-based recommendation currently. Connect the devices "
                "and use Home Base coordinates for localized estimates."
            )
        self.recommendations.configure(state="normal")
        self.recommendations.delete("1.0", "end")
        self.recommendations.insert("1.0", "\n\n".join(messages))
        self.recommendations.configure(state="disabled")

    def close(self) -> None:
        self.closing = True
        if self.mesh_listener is not None:
            self.mesh_listener.stop()
        for worker_state in self.analyzer_workers.values():
            worker_state[0].set()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    RFDesktopApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
