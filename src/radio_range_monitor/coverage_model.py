"""RF calculations, telemetry state, and threaded device/data workers.

The corresponding Tkinter interface is in ``dashboard.py``.
Launch the application with ``python -m radio_range_monitor``.
"""

from __future__ import annotations

import math
import os
import re
import threading
import time
from datetime import datetime, timezone
from typing import Any

import numpy as np
import requests
import serial
from pubsub import pub

# RF model parameters and thread-safe telemetry shared with the UI.
EARTH_RADIUS_M = 6_371_000.0
MAX_COVERAGE_RADIUS_M = 15_000.0
GRID_SPACING_M = 500.0
MIN_REQUIRED_LORA_SNR_DB = -7.5
RADIO_SENSITIVITY_DBM = -120.0
SYSTEM_LOSS_DB = 2.0
SWEEP_POINTS = 101
BAUD_RATE = 115200

TELEMETRY_LOCK = threading.RLock()
TELEMETRY: dict[str, Any] = {
    "rssi_dbm": None,
    "snr_db": None,
    "swr": None,
    "swr_frequency_hz": None,
    "noise_floor_dbm": None,
    "node_lat": None,
    "node_lon": None,
    "distance_m": None,
    "coordinate_source": None,
    "last_packet": None,
    "last_swr_update": None,
    "last_noise_update": None,
    "mesh_status": "Disconnected",
    "nanovna_status": "Not connected",
    "tinysa_status": "Not connected",
    "home_lat": float(os.getenv("HOME_LAT", "0")),
    "home_lon": float(os.getenv("HOME_LON", "0")),
}

NUMBER_PATTERN = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")


# Geodesic helpers and vectorized RF coverage calculations.
def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres."""
    phi1, phi2 = np.radians(lat1), np.radians(lat2)
    delta_phi = np.radians(lat2 - lat1)
    delta_lambda = np.radians(lon2 - lon1)
    a = (
        np.sin(delta_phi / 2) ** 2
        + np.cos(phi1) * np.cos(phi2) * np.sin(delta_lambda / 2) ** 2
    )
    return float(2 * EARTH_RADIUS_M * np.arcsin(np.sqrt(np.minimum(1.0, a))))


def destination_point(
    latitude: float, longitude: float, distance_m: float, bearing_rad: float
) -> tuple[float, float]:
    """Return WGS84 coordinates reached along a geodesic bearing."""
    angular = distance_m / EARTH_RADIUS_M
    lat1, lon1 = math.radians(latitude), math.radians(longitude)
    lat2 = math.asin(
        math.sin(lat1) * math.cos(angular)
        + math.cos(lat1) * math.sin(angular) * math.cos(bearing_rad)
    )
    lon2 = lon1 + math.atan2(
        math.sin(bearing_rad) * math.sin(angular) * math.cos(lat1),
        math.cos(angular) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), (math.degrees(lon2) + 540) % 360 - 180


def hata_urban_path_loss_db(
    frequency_mhz: float,
    base_height_m: float,
    mobile_height_m: float,
    distance_km: np.ndarray | float,
) -> np.ndarray:
    """Urban Okumura-Hata median path loss, vectorized for distances in km."""
    if not 150 <= frequency_mhz <= 1500:
        raise ValueError("Hata model requires a frequency between 150 and 1500 MHz.")
    if base_height_m <= 0 or mobile_height_m <= 0:
        raise ValueError("Antenna heights must be greater than zero.")

    distance = np.maximum(np.asarray(distance_km, dtype=float), 1.0)
    log_frequency = math.log10(frequency_mhz)
    log_base_height = math.log10(base_height_m)
    mobile_correction = (1.1 * log_frequency - 0.7) * mobile_height_m - (
        1.56 * log_frequency - 0.8
    )
    return (
        69.55
        + 26.16 * log_frequency
        - 13.82 * log_base_height
        - mobile_correction
        + (44.9 - 6.55 * log_base_height) * np.log10(distance)
    )


def swr_mismatch_loss_db(swr: float | None) -> float:
    """Convert SWR to mismatch loss in dB; missing SWR assumes a matched system."""
    if swr is None:
        return 0.0
    if math.isinf(swr):
        return 60.0
    if swr < 1.0:
        raise ValueError("SWR cannot be less than 1.0.")
    reflection = (swr - 1.0) / (swr + 1.0)
    return min(60.0, -10.0 * math.log10(max(1e-12, 1.0 - reflection**2)))


def effective_receiver_sensitivity_dbm(
    noise_floor_dbm: float | None,
    specified_sensitivity_dbm: float = RADIO_SENSITIVITY_DBM,
    required_snr_db: float = MIN_REQUIRED_LORA_SNR_DB,
) -> float:
    """Raise the configured sensitivity threshold when the measured noise is high."""
    if noise_floor_dbm is None:
        return specified_sensitivity_dbm
    return max(specified_sensitivity_dbm, noise_floor_dbm + required_snr_db)


def local_offset_to_latlon(
    home_lat: float, home_lon: float, east_m: float, north_m: float
) -> tuple[float, float]:
    """Convert small local east/north offsets to latitude/longitude."""
    lat = home_lat + math.degrees(north_m / EARTH_RADIUS_M)
    cosine = max(1e-6, math.cos(math.radians(home_lat)))
    lon = home_lon + math.degrees(east_m / (EARTH_RADIUS_M * cosine))
    return lat, (lon + 540) % 360 - 180


def generate_coverage_grid(
    home_lat: float,
    home_lon: float,
    frequency_mhz: float,
    tx_power_dbm: float,
    tx_gain_dbi: float,
    rx_gain_dbi: float,
    tx_height_m: float,
    rx_height_m: float,
    swr: float | None,
    noise_floor_dbm: float | None,
    specified_sensitivity_dbm: float = RADIO_SENSITIVITY_DBM,
    required_snr_db: float = MIN_REQUIRED_LORA_SNR_DB,
    max_radius_m: float = MAX_COVERAGE_RADIUS_M,
    spacing_m: float = GRID_SPACING_M,
) -> dict[str, Any]:
    """Vectorize a 15-km Hata coverage mesh and merge neighboring cells by class.

    Returned polygons are compact row-runs rather than one widget per point.
    Signal classes: 2=clear (>10 dB margin), 1=fringe (0..10 dB), 0=blocked.
    """
    if not -90 <= home_lat <= 90 or not -180 <= home_lon <= 180:
        raise ValueError("Home latitude/longitude is outside valid bounds.")
    if max_radius_m <= 0 or spacing_m <= 0:
        raise ValueError("Coverage radius and grid spacing must be positive.")

    offsets = np.arange(-max_radius_m, max_radius_m + spacing_m, spacing_m)
    east, north = np.meshgrid(offsets, offsets, indexing="xy")
    distances = np.hypot(east, north)
    in_range = distances <= max_radius_m
    cos_home = max(1e-6, math.cos(math.radians(home_lat)))
    longitudes = home_lon + np.degrees(east / (EARTH_RADIUS_M * cos_home))
    longitudes = (longitudes + 540.0) % 360.0 - 180.0

    path_loss = hata_urban_path_loss_db(
        frequency_mhz,
        tx_height_m,
        rx_height_m,
        np.maximum(distances, 1000.0) / 1000.0,
    )
    mismatch_loss = swr_mismatch_loss_db(swr)
    received_power = (
        tx_power_dbm
        + tx_gain_dbi
        + rx_gain_dbi
        - SYSTEM_LOSS_DB
        - mismatch_loss
        - path_loss
    )
    sensitivity = effective_receiver_sensitivity_dbm(
        noise_floor_dbm,
        specified_sensitivity_dbm=specified_sensitivity_dbm,
        required_snr_db=required_snr_db,
    )
    margin = received_power - sensitivity
    signal_class = np.where(margin >= 10.0, 2, np.where(margin >= 0.0, 1, 0))
    signal_class = np.where(in_range, signal_class, -1)

    colors = {
        2: "#b7e4c7",  # pale green for clear coverage
        1: "#fff1a8",  # pale yellow for fringe coverage
        0: "#f5c2c7",  # pale red for predicted out-of-range cells
    }
    half = spacing_m / 2.0
    polygons: list[tuple[int, list[tuple[float, float]]]] = []
    for row_index, row in enumerate(signal_class):
        valid_indices = np.flatnonzero(row >= 0)
        if valid_indices.size == 0:
            continue
        start = int(valid_indices[0])
        previous_index = start
        current_class = int(row[start])
        for raw_index in valid_indices[1:]:
            index = int(raw_index)
            category = int(row[index])
            if index == previous_index + 1 and category == current_class:
                previous_index = index
                continue

            east_start = offsets[start] - half
            east_end = offsets[previous_index] + half
            north_center = offsets[row_index]
            polygon = [
                local_offset_to_latlon(
                    home_lat, home_lon, east_start, north_center + half
                ),
                local_offset_to_latlon(
                    home_lat, home_lon, east_end, north_center + half
                ),
                local_offset_to_latlon(
                    home_lat, home_lon, east_end, north_center - half
                ),
                local_offset_to_latlon(
                    home_lat, home_lon, east_start, north_center - half
                ),
            ]
            polygons.append((current_class, polygon))
            start = previous_index = index
            current_class = category

        east_start = offsets[start] - half
        east_end = offsets[previous_index] + half
        north_center = offsets[row_index]
        polygons.append(
            (
                current_class,
                [
                    local_offset_to_latlon(
                        home_lat, home_lon, east_start, north_center + half
                    ),
                    local_offset_to_latlon(
                        home_lat, home_lon, east_end, north_center + half
                    ),
                    local_offset_to_latlon(
                        home_lat, home_lon, east_end, north_center - half
                    ),
                    local_offset_to_latlon(
                        home_lat, home_lon, east_start, north_center - half
                    ),
                ],
            )
        )

    counts = {
        category: int(np.count_nonzero(signal_class == category))
        for category in (2, 1, 0)
    }
    return {
        "polygons": polygons,
        "counts": counts,
        "sensitivity_dbm": sensitivity,
        "noise_floor_dbm": noise_floor_dbm,
        "mismatch_loss_db": mismatch_loss,
        "max_received_dbm": float(np.max(received_power[in_range])),
        "grid_points": int(np.count_nonzero(in_range)),
        "colors": colors,
    }


# Serial command handling and parsers for the supported analyzer output formats.
def parse_numeric_columns(line: str) -> list[float]:
    return [float(value) for value in NUMBER_PATTERN.findall(line)]


def serial_command(port: str, command: str, timeout_s: float = 25.0) -> str:
    """Open one analyzer serial port, issue an ASCII command, and collect its reply."""
    chunks: list[bytes] = []
    start = time.monotonic()
    last_data: float | None = None
    with serial.Serial(
        port, baudrate=BAUD_RATE, timeout=0.15, write_timeout=1.0
    ) as connection:
        connection.reset_input_buffer()
        connection.write((command + "\r\n").encode("ascii"))
        connection.flush()
        while time.monotonic() - start < timeout_s:
            chunk = connection.read(min(max(connection.in_waiting, 1), 4096))
            if chunk:
                chunks.append(chunk)
                last_data = time.monotonic()
            elif last_data is not None and time.monotonic() - last_data > 0.5:
                break
    return b"".join(chunks).decode("utf-8", errors="replace")


def parse_tinysa_sweep(response: str, start_hz: int, stop_hz: int) -> list[float]:
    """Return dBm readings from ASCII frequency/level tinySA sweep rows."""
    levels: list[float] = []
    for line in response.splitlines():
        values = parse_numeric_columns(line)
        if len(values) >= 2:
            frequency, level = values[0], values[1]
            if start_hz <= frequency <= stop_hz and -220 <= level <= 50:
                levels.append(level)
    return levels


def parse_nanovna_sweep(
    response: str, start_hz: int, stop_hz: int
) -> list[tuple[float, float, float]]:
    """Return frequency, real(S11), imaginary(S11) rows from ASCII output."""
    rows: list[tuple[float, float, float]] = []
    for line in response.splitlines():
        values = parse_numeric_columns(line)
        if len(values) >= 3 and start_hz <= values[0] <= stop_hz:
            rows.append((values[0], values[1], values[2]))
    return rows


# Each worker owns its serial connection loop so slow sweeps never block Tk.
class AnalyzerWorker(threading.Thread):
    """Poll one serial RF analyzer without blocking the GUI thread."""

    def __init__(
        self,
        role: str,
        port: str,
        stop_event: threading.Event,
        frequency_mhz: float,
    ) -> None:
        super().__init__(name=f"{role}-worker", daemon=True)
        self.role = role
        self.port = port
        self.stop_event = stop_event
        self.frequency_mhz = frequency_mhz

    def run(self) -> None:
        status_key = f"{self.role}_status"
        start_hz = int(self.frequency_mhz * 1_000_000 - 5_000_000)
        stop_hz = start_hz + 10_000_000
        while not self.stop_event.is_set():
            try:
                if self.role == "tinysa":
                    response = serial_command(
                        self.port,
                        f"scan {start_hz} {stop_hz} {SWEEP_POINTS} 3",
                    )
                    levels = parse_tinysa_sweep(response, start_hz, stop_hz)
                    if not levels:
                        raise ValueError("No tinySA frequency/dBm sweep rows found.")
                    levels_array = np.asarray(levels, dtype=float)
                    noise_floor = float(np.percentile(levels_array, 20))
                    with TELEMETRY_LOCK:
                        TELEMETRY["noise_floor_dbm"] = noise_floor
                        TELEMETRY["last_noise_update"] = datetime.now(timezone.utc)
                else:
                    response = serial_command(
                        self.port, f"scan {start_hz} {stop_hz} {SWEEP_POINTS}"
                    )
                    rows = parse_nanovna_sweep(response, start_hz, stop_hz)
                    if not rows:
                        raise ValueError(
                            "No NanoVNA frequency/S11 rows found; check firmware protocol."
                        )
                    _, real, imaginary = min(
                        rows,
                        key=lambda row: abs(row[0] - self.frequency_mhz * 1e6),
                    )
                    reflection = math.hypot(real, imaginary)
                    swr = (
                        float("inf")
                        if reflection >= 1
                        else (1 + reflection) / (1 - reflection)
                    )
                    with TELEMETRY_LOCK:
                        TELEMETRY["swr"] = swr
                        TELEMETRY["last_swr_update"] = datetime.now(timezone.utc)
                set_status(status_key, f"Connected: {self.port}")
            except (serial.SerialException, OSError, ValueError) as exc:
                set_status(status_key, f"{self.port}: {exc}")
            self.stop_event.wait(2.0)


# Meshtastic uses its callback API; only the packet callback updates telemetry.
class MeshtasticListener(threading.Thread):
    """Listen to the official Meshtastic Python API on the selected USB port."""

    def __init__(self, port: str | None) -> None:
        super().__init__(name="meshtastic-listener", daemon=True)
        self.port = port
        self.stop_event = threading.Event()
        self.interface: Any = None

    @staticmethod
    def coordinate(
        position: dict[str, Any], decimal_key: str, integer_key: str
    ) -> float | None:
        if position.get(decimal_key) is not None:
            return float(position[decimal_key])
        if position.get(integer_key) is not None:
            return float(position[integer_key]) / 10_000_000.0
        return None

    def on_receive(self, packet: dict[str, Any], interface: Any) -> None:
        del interface
        decoded = packet.get("decoded") or {}
        position = decoded.get("position") or {}
        latitude = self.coordinate(position, "latitude", "latitudeI")
        longitude = self.coordinate(position, "longitude", "longitudeI")
        now = datetime.now(timezone.utc)
        with TELEMETRY_LOCK:
            if packet.get("rxRssi") is not None:
                TELEMETRY["rssi_dbm"] = float(packet["rxRssi"])
            if packet.get("rxSnr") is not None:
                TELEMETRY["snr_db"] = float(packet["rxSnr"])
            if latitude is not None and longitude is not None:
                TELEMETRY["node_lat"] = latitude
                TELEMETRY["node_lon"] = longitude
                TELEMETRY["coordinate_source"] = "Meshtastic packet"
                TELEMETRY["distance_m"] = haversine_m(
                    TELEMETRY["home_lat"], TELEMETRY["home_lon"], latitude, longitude
                )
            TELEMETRY["last_packet"] = now
            if (
                TELEMETRY["rssi_dbm"] is not None
                and TELEMETRY["distance_m"] is not None
            ):
                TELEMETRY.setdefault("history", []).append(
                    (now, TELEMETRY["distance_m"], TELEMETRY["rssi_dbm"])
                )
                TELEMETRY["history"] = TELEMETRY["history"][-200:]

    def on_connection(self, interface: Any, topic: Any = None) -> None:
        del interface, topic
        set_status(
            "mesh_status", f"Connected: {self.port or 'auto-detected serial port'}"
        )

    def run(self) -> None:
        try:
            from meshtastic.serial_interface import SerialInterface

            pub.subscribe(self.on_receive, "meshtastic.receive")
            pub.subscribe(self.on_connection, "meshtastic.connection.established")
            set_status(
                "mesh_status", f"Connecting to {self.port or 'auto-detected node'}..."
            )
            self.interface = SerialInterface(devPath=self.port)
            set_status(
                "mesh_status", f"Connected: {self.port or 'auto-detected serial port'}"
            )
            while not self.stop_event.wait(0.5):
                pass
        except Exception as exc:
            set_status("mesh_status", f"Connection error: {exc}")
        finally:
            pub.unsubscribe(self.on_receive, "meshtastic.receive")
            pub.unsubscribe(self.on_connection, "meshtastic.connection.established")
            if self.interface is not None:
                self.interface.close()

    def stop(self) -> None:
        self.stop_event.set()


def set_status(key: str, value: str) -> None:
    with TELEMETRY_LOCK:
        TELEMETRY[key] = value


# Terrain API work is called from a background worker, never from the Tk loop.
def fetch_elevation_profile(
    start_lat: float,
    start_lon: float,
    end_lat: float,
    end_lon: float,
    samples: int = 50,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fetch a 50-point Open-Elevation profile and apply Earth curvature."""
    fractions = np.linspace(0.0, 1.0, samples)
    lat1, lon1 = math.radians(start_lat), math.radians(start_lon)
    lat2, lon2 = math.radians(end_lat), math.radians(end_lon)
    delta = 2 * math.asin(
        math.sqrt(
            math.sin((lat2 - lat1) / 2) ** 2
            + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2) ** 2
        )
    )
    distance_m = delta * EARTH_RADIUS_M
    if distance_m < 1:
        raise ValueError("Place the target at least 1 metre from the home base.")

    # Spherical interpolation yields evenly spaced geographic sample coordinates.
    sin_delta = math.sin(delta)
    if abs(sin_delta) < 1e-12:
        latitudes = np.linspace(start_lat, end_lat, samples)
        longitudes = np.linspace(start_lon, end_lon, samples)
    else:
        a = np.sin((1 - fractions) * delta) / sin_delta
        b = np.sin(fractions * delta) / sin_delta
        x = a * math.cos(lat1) * math.cos(lon1) + b * math.cos(lat2) * math.cos(lon2)
        y = a * math.cos(lat1) * math.sin(lon1) + b * math.cos(lat2) * math.sin(lon2)
        z = a * math.sin(lat1) + b * math.sin(lat2)
        latitudes = np.degrees(np.arctan2(z, np.hypot(x, y)))
        longitudes = np.degrees(np.arctan2(y, x))

    locations = [
        {"latitude": float(lat), "longitude": float(lon)}
        for lat, lon in zip(latitudes, longitudes)
    ]
    elevation_api_url = os.getenv("ELEVATION_API_URL")
    if elevation_api_url:
        response = requests.post(
            elevation_api_url,
            json={"locations": locations},
            headers={"User-Agent": "RadioRangeMonitor/1.0"},
            timeout=35,
        )
    else:
        response = requests.get(
            "https://api.open-meteo.com/v1/elevation",
            params={
                "latitude": ",".join(
                    f"{location['latitude']:.6f}" for location in locations
                ),
                "longitude": ",".join(
                    f"{location['longitude']:.6f}" for location in locations
                ),
            },
            headers={"User-Agent": "RadioRangeMonitor/1.0"},
            timeout=35,
        )
    response.raise_for_status()
    if elevation_api_url:
        elevations = [
            row.get("elevation") for row in response.json().get("results", [])
        ]
    else:
        elevations = response.json().get("elevation", [])
    if len(elevations) != samples:
        raise RuntimeError(
            f"Elevation API returned {len(elevations)} of {samples} samples."
        )
    if any(elevation is None for elevation in elevations):
        raise RuntimeError(
            "Terrain data is unavailable for one or more profile points."
        )
    raw_elevation = np.asarray(
        [float(elevation) for elevation in elevations], dtype=float
    )
    distance_along = fractions * distance_m
    curvature_adjusted = raw_elevation + distance_along**2 / (2 * EARTH_RADIUS_M)
    return distance_along, raw_elevation, curvature_adjusted
