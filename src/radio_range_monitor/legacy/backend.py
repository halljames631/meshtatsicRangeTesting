"""Tkinter desktop monitor for Meshtastic, NanoVNA, and tinySA telemetry.

Install dependencies with:
    pip install meshtastic pyserial

The Meshtastic listener starts the official ``meshtastic`` CLI with
``--listen --debug`` and reads its packet log output.
Configure the home position and optional serial ports with environment variables:
    HOME_LAT, HOME_LON, MESHTASTIC_PORT, NANOVNA_PORT, TINYSA_PORT

The analyzers' serial protocols vary by model and firmware. Discovery classifies
devices from their USB description and version/help responses. The NanoVNA worker
supports ASCII sweep rows formatted as frequency_Hz, S11_real, S11_imaginary.
The tinySA worker uses its documented ASCII ``scan`` command and expects rows of
frequency_Hz and dBm. Incompatible response formats are reported in device status.
"""

from __future__ import annotations

import json
import math
import os
import re
import shutil
import statistics
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from typing import Any
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import serial
from serial.tools import list_ports

# Device protocols and shared process-wide telemetry.
USB_VID = 0x0483
USB_PID = 0x5740
BAUD_RATE = 115200
DEFAULT_FREQUENCY_HZ = 915_000_000
SWEEP_SPAN_HZ = 10_000_000
SWEEP_POINTS = 101
EARTH_RADIUS_M = 6_371_000
LIGHT_SPEED_M_S = 299_792_458

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
    "mesh_status": "Not connected",
    "nanovna_status": "Not detected",
    "tinysa_status": "Not detected",
    "discovery_notes": [],
    "history": [],
    "home_lat": 0.0,
    "home_lon": 0.0,
}

NUMBER_PATTERN = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")


# Geodesic, path-loss, terrain, and map-projection calculations.
def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Compute the great-circle distance between two latitude/longitude pairs."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    hav = (
        math.sin(delta_phi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    )
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(min(1.0, hav)))


def interpolate_great_circle(
    lat1: float, lon1: float, lat2: float, lon2: float, fraction: float
) -> tuple[float, float]:
    """Return the point at fraction 0..1 along the shortest great-circle path."""
    bearing_y = math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2))
    bearing_x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(
        math.radians(lat1)
    ) * math.cos(math.radians(lat2)) * math.cos(math.radians(lon2 - lon1))
    bearing = math.atan2(bearing_y, bearing_x)
    angular_distance = haversine_m(lat1, lon1, lat2, lon2) / EARTH_RADIUS_M
    angular_distance *= fraction
    lat1_rad = math.radians(lat1)
    lat = math.asin(
        math.sin(lat1_rad) * math.cos(angular_distance)
        + math.cos(lat1_rad) * math.sin(angular_distance) * math.cos(bearing)
    )
    lon = math.radians(lon1) + math.atan2(
        math.sin(bearing) * math.sin(angular_distance) * math.cos(lat1_rad),
        math.cos(angular_distance) - math.sin(lat1_rad) * math.sin(lat),
    )
    return math.degrees(lat), (math.degrees(lon) + 540) % 360 - 180


def free_space_path_loss_db(frequency_mhz: float, distance_m: float) -> float:
    """Free-space path loss in dB, using MHz and kilometres."""
    if frequency_mhz <= 0 or distance_m <= 0:
        raise ValueError("Frequency and distance must be greater than zero.")
    return 32.44 + 20 * math.log10(frequency_mhz) + 20 * math.log10(distance_m / 1000)


def knife_edge_loss_db(v: float) -> float:
    """Approximate single knife-edge diffraction loss (ITU-R P.526 form)."""
    if v <= -0.78:
        return 0.0
    return 6.9 + 20 * math.log10(math.sqrt((v - 0.1) ** 2 + 1) + v - 0.1)


def terrain_path_analysis(
    elevations_m: list[float],
    distance_m: float,
    frequency_mhz: float,
    tx_height_m: float,
    rx_height_m: float,
    earth_k_factor: float = 4 / 3,
) -> dict[str, float]:
    """Estimate worst terrain obstruction, first-Fresnel clearance, and loss."""
    if len(elevations_m) < 3:
        raise ValueError("At least three elevation samples are required.")
    if distance_m <= 0 or frequency_mhz <= 0 or earth_k_factor <= 0:
        raise ValueError("Distance, frequency, and k-factor must be positive.")

    wavelength = LIGHT_SPEED_M_S / (frequency_mhz * 1_000_000)
    start_altitude = elevations_m[0] + tx_height_m
    end_altitude = elevations_m[-1] + rx_height_m
    max_v = float("-inf")
    min_fresnel_clearance_ratio = float("inf")

    for index, elevation in enumerate(elevations_m[1:-1], start=1):
        fraction = index / (len(elevations_m) - 1)
        along_m = distance_m * fraction
        remaining_m = distance_m - along_m
        earth_bulge_m = along_m * remaining_m / (2 * earth_k_factor * EARTH_RADIUS_M)
        direct_ray_altitude = start_altitude * (1 - fraction) + end_altitude * fraction
        obstruction_m = elevation + earth_bulge_m - direct_ray_altitude
        fresnel_radius_m = math.sqrt(wavelength * along_m * remaining_m / distance_m)
        min_fresnel_clearance_ratio = min(
            min_fresnel_clearance_ratio,
            (direct_ray_altitude - elevation - earth_bulge_m) / fresnel_radius_m,
        )
        v = obstruction_m * math.sqrt(
            2 * distance_m / (wavelength * along_m * remaining_m)
        )
        max_v = max(max_v, v)

    if max_v == float("-inf"):
        max_v = -1.0
    if min_fresnel_clearance_ratio == float("inf"):
        min_fresnel_clearance_ratio = float("nan")
    return {
        "max_v": max_v,
        "diffraction_loss_db": knife_edge_loss_db(max_v),
        "min_fresnel_clearance_ratio": min_fresnel_clearance_ratio,
    }


def fetch_elevation_profile(
    start_lat: float,
    start_lon: float,
    end_lat: float,
    end_lon: float,
    sample_count: int = 41,
) -> list[float]:
    """Fetch SRTM90m elevations for points along a great-circle path."""
    if not 3 <= sample_count <= 99:
        raise ValueError("Terrain sample count must be between 3 and 99.")
    locations = [
        interpolate_great_circle(
            start_lat, start_lon, end_lat, end_lon, i / (sample_count - 1)
        )
        for i in range(sample_count)
    ]
    location_arg = "|".join(f"{lat:.6f},{lon:.6f}" for lat, lon in locations)
    url = "https://api.opentopodata.org/v1/srtm90m?" + urlencode(
        {"locations": location_arg}
    )
    request = Request(url, headers={"User-Agent": "RadioRangeMonitor/1.0"})
    with urlopen(request, timeout=25) as response:
        result = json.loads(response.read().decode("utf-8"))
    if result.get("status") != "OK":
        raise RuntimeError(f"Elevation service returned: {result.get('status')}")
    samples = result.get("results", [])
    if len(samples) != sample_count:
        raise RuntimeError(
            f"Elevation service returned {len(samples)} of {sample_count} samples."
        )
    elevations = [sample.get("elevation") for sample in samples]
    if any(elevation is None for elevation in elevations):
        raise RuntimeError(
            "Terrain data is unavailable for one or more points on this route."
        )
    return [float(elevation) for elevation in elevations]


def coordinates_to_world_pixel(
    latitude: float, longitude: float, zoom: int
) -> tuple[float, float]:
    """Project WGS84 coordinates into Web Mercator world-pixel coordinates."""
    latitude = max(-85.05112878, min(85.05112878, latitude))
    scale = 256 * (2**zoom)
    x = (longitude + 180.0) / 360.0 * scale
    lat_rad = math.radians(latitude)
    y = (1 - math.asinh(math.tan(lat_rad)) / math.pi) / 2 * scale
    return x, y


def world_pixel_to_coordinates(x: float, y: float, zoom: int) -> tuple[float, float]:
    """Convert Web Mercator world-pixel coordinates to latitude/longitude."""
    scale = 256 * (2**zoom)
    longitude = (x % scale) / scale * 360.0 - 180.0
    y = max(0.0, min(scale, y))
    mercator = math.pi * (1 - 2 * y / scale)
    latitude = math.degrees(math.atan(math.sinh(mercator)))
    return latitude, longitude


def set_status(key: str, message: str) -> None:
    with TELEMETRY_LOCK:
        TELEMETRY[key] = message


# Device discovery and the serial response formats supported by this app.
def serial_command(port_name: str, command: str, timeout_seconds: float = 5.0) -> str:
    """Send a line-oriented command and read until the response goes quiet."""
    chunks: list[bytes] = []
    start = time.monotonic()
    last_data: float | None = None

    with serial.Serial(
        port_name,
        baudrate=BAUD_RATE,
        timeout=0.15,
        write_timeout=1.0,
    ) as connection:
        connection.reset_input_buffer()
        connection.write((command + "\r\n").encode("ascii"))
        connection.flush()

        while time.monotonic() - start < timeout_seconds:
            waiting = connection.in_waiting
            chunk = connection.read(min(max(waiting, 1), 4096))
            if chunk:
                chunks.append(chunk)
                last_data = time.monotonic()
            elif last_data is not None and time.monotonic() - last_data > 0.5:
                break

    return b"".join(chunks).decode("utf-8", errors="replace")


def discover_devices() -> tuple[list[str], dict[str, str], list[str]]:
    """Probe ChibiOS VID/PID serial ports and classify recognizable analyzers."""
    ports = list(list_ports.comports())
    all_ports = [port.device for port in ports]
    recognized: dict[str, str] = {}
    notes: list[str] = []
    candidates = [port for port in ports if port.vid == USB_VID and port.pid == USB_PID]

    for port in candidates:
        description = " ".join(
            str(value or "")
            for value in (port.description, port.manufacturer, port.product)
        )
        replies: list[str] = []
        for command in ("version", "help"):
            try:
                result = serial_command(port.device, command, timeout_seconds=2.5)
                if result.strip():
                    replies.append(result)
            except (serial.SerialException, OSError) as exc:
                notes.append(f"{port.device}: {command} probe failed: {exc}")

        signature = f"{description}\n{''.join(replies)}".lower()
        # tinySA's command list includes scanraw and attenuate; test this first
        # because some firmware shares generic commands with other analyzers.
        if (
            "tinysa" in signature
            or "tiny sa" in signature
            or ("scanraw" in signature and "attenuate" in signature)
        ):
            recognized["tinysa"] = port.device
        elif (
            "nanovna" in signature or "nano vna" in signature or "nanorfe" in signature
        ):
            recognized["nanovna"] = port.device
        elif "frequencies" in signature and "data" in signature:
            # Some NanoVNA firmware does not include its product name in replies.
            recognized["nanovna"] = port.device
        else:
            notes.append(
                f"{port.device}: no known analyzer signature in version/help response."
            )

    if not candidates:
        notes.append(
            f"No analyzer ports with USB VID/PID {USB_VID:04X}:{USB_PID:04X} "
            "matched auto-detection. Select the analyzer's port manually."
        )
    return all_ports, recognized, notes


def numbers_in(line: str) -> list[float]:
    return [float(match) for match in NUMBER_PATTERN.findall(line)]


def parse_nano_vna_rows(
    response: str, start_hz: int, stop_hz: int
) -> list[tuple[float, float, float]]:
    """Parse frequency, real(S11), imaginary(S11) rows from analyzer output."""
    rows: list[tuple[float, float, float]] = []
    for line in response.splitlines():
        values = numbers_in(line)
        if len(values) >= 3 and start_hz <= values[0] <= stop_hz:
            rows.append((values[0], values[1], values[2]))
    return rows


def parse_tinysa_rows(response: str, start_hz: int, stop_hz: int) -> list[float]:
    """Parse dBm levels from tinySA frequency/level scan rows."""
    levels: list[float] = []
    for line in response.splitlines():
        values = numbers_in(line)
        if len(values) >= 2:
            frequency_hz, level_dbm = values[0], values[1]
            if start_hz <= frequency_hz <= stop_hz and -220 <= level_dbm <= 50:
                levels.append(level_dbm)
    return levels


# Each hardware worker owns its own blocking I/O loop; the Tk thread never waits
# for serial reads or Meshtastic CLI output.
class AnalyzerWorker(threading.Thread):
    """Repeatedly collect one analyzer's telemetry without blocking Streamlit."""

    def __init__(
        self,
        role: str,
        port_name: str | None,
        stop_event: threading.Event,
        frequency_hz: int,
    ) -> None:
        super().__init__(name=f"{role}-worker", daemon=True)
        self.role = role
        self.port_name = port_name
        self.stop_event = stop_event
        self.frequency_hz = frequency_hz

    def run(self) -> None:
        if not self.port_name:
            set_status(f"{self.role}_status", "No port detected/configured")
            return

        while not self.stop_event.is_set():
            try:
                if self.role == "nanovna":
                    self.read_swr()
                else:
                    self.read_noise_floor()
                set_status(f"{self.role}_status", f"Connected: {self.port_name}")
            except (serial.SerialException, OSError, ValueError) as exc:
                set_status(
                    f"{self.role}_status",
                    f"{self.port_name}: {exc}",
                )
            self.stop_event.wait(2.0)

    def read_swr(self) -> None:
        start_hz = self.frequency_hz - SWEEP_SPAN_HZ // 2
        stop_hz = self.frequency_hz + SWEEP_SPAN_HZ // 2
        response = serial_command(
            self.port_name,
            f"scan {start_hz} {stop_hz} {SWEEP_POINTS}",
            timeout_seconds=25.0,
        )
        rows = parse_nano_vna_rows(response, start_hz, stop_hz)
        if not rows:
            raise ValueError(
                "No frequency/S11 real/imaginary rows received; verify this "
                "NanoVNA firmware's serial sweep command and output format."
            )

        frequency, real, imaginary = min(
            rows, key=lambda row: abs(row[0] - self.frequency_hz)
        )
        reflection = math.hypot(real, imaginary)
        swr = (
            float("inf")
            if reflection >= 1.0
            else (1.0 + reflection) / (1.0 - reflection)
        )
        with TELEMETRY_LOCK:
            TELEMETRY["swr"] = swr
            TELEMETRY["swr_frequency_hz"] = frequency
            TELEMETRY["last_swr_update"] = datetime.now(timezone.utc)

    def read_noise_floor(self) -> None:
        start_hz = self.frequency_hz - SWEEP_SPAN_HZ // 2
        stop_hz = self.frequency_hz + SWEEP_SPAN_HZ // 2
        response = serial_command(
            self.port_name,
            f"scan {start_hz} {stop_hz} {SWEEP_POINTS} 3",
            timeout_seconds=25.0,
        )
        levels = parse_tinysa_rows(response, start_hz, stop_hz)
        if not levels:
            raise ValueError(
                "No frequency/dBm scan rows received; verify tinySA response format."
            )

        # Use the 20th percentile to estimate the baseline without letting a few
        # strong carriers dominate the noise-floor estimate.
        noise_floor = (
            statistics.quantiles(levels, n=5)[0] if len(levels) > 1 else levels[0]
        )
        with TELEMETRY_LOCK:
            TELEMETRY["noise_floor_dbm"] = noise_floor
            TELEMETRY["last_noise_update"] = datetime.now(timezone.utc)


class MeshtasticWorker(threading.Thread):
    """Run the official Meshtastic CLI and consume its debug packet log lines."""

    PACKET_LOG_MARKER = "Publishing meshtastic.receive"

    def __init__(self, port_name: str | None, stop_event: threading.Event) -> None:
        super().__init__(name="meshtastic-cli-worker", daemon=True)
        self.port_name = port_name
        self.stop_event = stop_event
        self.process: subprocess.Popen[str] | None = None
        self.process_lock = threading.Lock()

    @staticmethod
    def packet_number(line: str, field: str) -> float | None:
        """Read a numeric field from the CLI's Python-dict packet log."""
        match = re.search(
            rf"""['"]{re.escape(field)}['"]\s*:\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)""",
            line,
        )
        return float(match.group(1)) if match else None

    @classmethod
    def parse_packet_log(cls, line: str) -> dict[str, float | None] | None:
        """Extract packet metrics and optional position from one CLI log line."""
        if cls.PACKET_LOG_MARKER not in line or "packet=" not in line:
            return None

        # Parse known scalar fields from the packet dictionary. Its decoded
        # payload can contain a protobuf repr, so the entire log is not always
        # a valid Python literal and should not be evaluated.
        return {
            "rssi": cls.packet_number(line, "rxRssi"),
            "snr": cls.packet_number(line, "rxSnr"),
            "latitude": cls.packet_number(line, "latitude"),
            "longitude": cls.packet_number(line, "longitude"),
            "latitude_i": cls.packet_number(line, "latitudeI"),
            "longitude_i": cls.packet_number(line, "longitudeI"),
        }

    def consume_packet_log(self, line: str) -> None:
        packet = self.parse_packet_log(line)
        if packet is None:
            return

        rssi = packet["rssi"]
        snr = packet["snr"]
        latitude = packet["latitude"]
        longitude = packet["longitude"]
        if latitude is None and packet["latitude_i"] is not None:
            latitude = packet["latitude_i"] / 10_000_000
        if longitude is None and packet["longitude_i"] is not None:
            longitude = packet["longitude_i"] / 10_000_000

        now = datetime.now(timezone.utc)
        with TELEMETRY_LOCK:
            if rssi is not None:
                TELEMETRY["rssi_dbm"] = rssi
            if snr is not None:
                TELEMETRY["snr_db"] = snr
            if latitude is not None and longitude is not None:
                TELEMETRY["node_lat"] = latitude
                TELEMETRY["node_lon"] = longitude
                TELEMETRY["coordinate_source"] = "Meshtastic CLI packet"
            TELEMETRY["last_packet"] = now
            node_lat, node_lon = TELEMETRY["node_lat"], TELEMETRY["node_lon"]
            if node_lat is not None and node_lon is not None:
                TELEMETRY["distance_m"] = haversine_m(
                    TELEMETRY["home_lat"],
                    TELEMETRY["home_lon"],
                    node_lat,
                    node_lon,
                )
            if (
                TELEMETRY["rssi_dbm"] is not None
                and TELEMETRY["distance_m"] is not None
            ):
                TELEMETRY["history"].append(
                    (now, TELEMETRY["distance_m"], TELEMETRY["rssi_dbm"])
                )
                TELEMETRY["history"] = TELEMETRY["history"][-200:]

    def command(self) -> list[str]:
        """Prefer the installed CLI executable, falling back to its Python module."""
        executable = shutil.which("meshtastic")
        command = [executable] if executable else [sys.executable, "-m", "meshtastic"]
        if self.port_name:
            command.extend(["--port", self.port_name])
        command.extend(["--listen", "--debug"])
        return command

    def run(self) -> None:
        while not self.stop_event.is_set():
            process: subprocess.Popen[str] | None = None
            recent_output: list[str] = []
            try:
                process = subprocess.Popen(
                    self.command(),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    bufsize=1,
                )
                with self.process_lock:
                    self.process = process
                    if self.stop_event.is_set() and process.poll() is None:
                        process.terminate()
                selected_port = self.port_name or "CLI auto-detect"
                set_status(
                    "mesh_status", f"CLI started on {selected_port}; awaiting node"
                )

                if process.stdout is None:
                    raise RuntimeError("Meshtastic CLI output stream was not created")

                for line in process.stdout:
                    if self.stop_event.is_set():
                        break
                    recent_output.append(line.strip())
                    recent_output = recent_output[-5:]
                    if "Connection changed: meshtastic.connection.established" in line:
                        set_status("mesh_status", f"CLI connected: {selected_port}")
                    self.consume_packet_log(line)

                return_code = process.wait()
                if not self.stop_event.is_set():
                    detail = next(
                        (
                            output
                            for output in reversed(recent_output)
                            if output and "DEBUG " not in output
                        ),
                        "",
                    )
                    set_status(
                        "mesh_status",
                        f"CLI exited with code {return_code}"
                        f"{': ' + detail if detail else ''}; retrying in 5 seconds",
                    )
            except (OSError, subprocess.SubprocessError, RuntimeError) as exc:
                set_status("mesh_status", f"Could not start Meshtastic CLI: {exc}")
            finally:
                with self.process_lock:
                    if process is not None and process.poll() is not None:
                        self.process = None

            self.stop_event.wait(5.0)

    def stop_cli(self) -> None:
        """Stop the CLI subprocess so its blocking output read can finish."""
        with self.process_lock:
            process = self.process
            if process is not None and process.poll() is None:
                try:
                    process.terminate()
                except OSError:
                    pass


# Coordinate hardware-worker lifecycle and keep UI-facing state in one place.
class RadioMonitor:
    """Discover devices once and keep their independent workers alive."""

    def __init__(self) -> None:
        self.stop_event = threading.Event()
        self.mesh_worker: MeshtasticWorker | None = None
        all_ports, roles, notes = discover_devices()
        self.available_serial_ports = all_ports
        roles["nanovna"] = os.getenv("NANOVNA_PORT", roles.get("nanovna"))
        roles["tinysa"] = os.getenv("TINYSA_PORT", roles.get("tinysa"))

        analyzer_ports = {roles.get("nanovna"), roles.get("tinysa")}
        available_mesh_ports = [
            port for port in all_ports if port not in analyzer_ports
        ]
        mesh_port = os.getenv("MESHTASTIC_PORT")
        if not mesh_port and len(available_mesh_ports) == 1:
            mesh_port = available_mesh_ports[0]
        elif not mesh_port and len(available_mesh_ports) > 1:
            notes.append(
                "Multiple serial ports found. Select the Meshtastic node port "
                "from the connection selector."
            )

        try:
            frequency_hz = int(float(os.getenv("RF_FREQUENCY_MHZ", "915")) * 1_000_000)
        except ValueError as exc:
            raise ValueError("RF_FREQUENCY_MHZ must be a numeric MHz value") from exc
        if frequency_hz <= 0:
            raise ValueError("RF_FREQUENCY_MHZ must be greater than zero")

        with TELEMETRY_LOCK:
            TELEMETRY["discovery_notes"] = notes
        self.mesh_port = mesh_port
        self.analyzer_ports = {
            "nanovna": roles.get("nanovna"),
            "tinysa": roles.get("tinysa"),
        }
        self.analyzer_frequency_hz = frequency_hz
        self.analyzer_workers: dict[str, tuple[threading.Event, AnalyzerWorker]] = {}
        for role in ("nanovna", "tinysa"):
            self.set_analyzer_port(role, roles.get(role))

    def connect_mesh(self, port_name: str | None) -> None:
        """Start or restart the Meshtastic CLI listener on the requested port."""
        self.disconnect_mesh()
        self.mesh_port = port_name
        mesh_stop_event = threading.Event()
        self.mesh_worker = MeshtasticWorker(port_name, mesh_stop_event)
        set_status(
            "mesh_status",
            f"Starting Meshtastic CLI on {port_name or 'auto-detected port'}...",
        )
        self.mesh_worker.start()

    def disconnect_mesh(self) -> None:
        """Stop the active Meshtastic CLI listener."""
        worker = self.mesh_worker
        if worker is not None:
            worker.stop_event.set()
            worker.stop_cli()
            worker.join(timeout=2.0)
            self.mesh_worker = None
        set_status("mesh_status", "Disconnected")

    def set_analyzer_port(self, role: str, port_name: str | None) -> None:
        """Stop an analyzer's previous worker and start it on the selected port."""
        if role not in ("nanovna", "tinysa"):
            raise ValueError(f"Unsupported analyzer role: {role}")
        current = self.analyzer_workers.pop(role, None)
        if current is not None:
            stop_event, worker = current
            stop_event.set()
            worker.join(timeout=2.0)

        if not port_name:
            self.analyzer_ports[role] = None
            set_status(f"{role}_status", "No port selected; choose a serial port")
            return

        other_role = "tinysa" if role == "nanovna" else "nanovna"
        if port_name == self.analyzer_ports.get(other_role):
            raise ValueError(
                f"{port_name} is already assigned to {other_role}; each device "
                "needs its own serial port."
            )

        self.analyzer_ports[role] = port_name
        stop_event = threading.Event()
        worker = AnalyzerWorker(role, port_name, stop_event, self.analyzer_frequency_hz)
        self.analyzer_workers[role] = (stop_event, worker)
        set_status(f"{role}_status", f"Starting on {port_name}...")
        worker.start()

    def set_home(self, latitude: float, longitude: float) -> None:
        with TELEMETRY_LOCK:
            TELEMETRY["home_lat"] = latitude
            TELEMETRY["home_lon"] = longitude
            node_lat, node_lon = TELEMETRY["node_lat"], TELEMETRY["node_lon"]
            if node_lat is not None and node_lon is not None:
                TELEMETRY["distance_m"] = haversine_m(
                    latitude, longitude, node_lat, node_lon
                )

    def set_test_position(self, latitude: float, longitude: float) -> None:
        with TELEMETRY_LOCK:
            TELEMETRY["node_lat"] = latitude
            TELEMETRY["node_lon"] = longitude
            TELEMETRY["coordinate_source"] = "Test input"
            TELEMETRY["distance_m"] = haversine_m(
                TELEMETRY["home_lat"],
                TELEMETRY["home_lon"],
                latitude,
                longitude,
            )

    def stop(self) -> None:
        self.stop_event.set()
        self.disconnect_mesh()
        for role in tuple(self.analyzer_workers):
            current = self.analyzer_workers.pop(role)
            current[0].set()
            current[1].join(timeout=2.0)


# Convert current measurements and recent history into user-facing advice.
def build_recommendations(data: dict[str, Any]) -> list[tuple[str, str]]:
    advice: list[tuple[str, str]] = []
    swr, noise = data.get("swr"), data.get("noise_floor_dbm")

    if swr is not None and swr > 1.5:
        advice.append(
            (
                "warning",
                "SWR is above 1.5. Check antenna tuning and feed-line connections; "
                "mismatch can increase reflected power and reduce radiated power.",
            )
        )
    if noise is not None and noise > -100:
        advice.append(
            (
                "warning",
                "The measured trace baseline is above -100 dBm. Check for local RF "
                "interference; consider a quieter channel or an appropriate cavity "
                "bandpass filter.",
            )
        )

    history = data.get("history", [])
    if (
        swr is not None
        and swr <= 1.5
        and noise is not None
        and noise <= -100
        and len(history) >= 2
    ):
        latest_time, latest_distance, latest_rssi = history[-1]
        for old_time, old_distance, old_rssi in reversed(history[:-1]):
            elapsed = (latest_time - old_time).total_seconds()
            distance_change = abs(latest_distance - old_distance)
            rssi_drop = old_rssi - latest_rssi
            if 0 < elapsed <= 300 and distance_change <= 500 and rssi_drop >= 15:
                advice.append(
                    (
                        "info",
                        "RSSI dropped by at least 15 dB over a short distance while "
                        "SWR and noise are good. Consider raising the antenna to "
                        "clear nearby structural or terrain obstructions.",
                    )
                )
                break
    return advice
