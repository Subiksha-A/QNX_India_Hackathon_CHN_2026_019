#!/usr/bin/env python3
"""
Unified QNX Vehicle Gateway Dashboard
=====================================

Combines the three existing pieces:

1. lidar_sensing / lidar_uart C bridge
   - UART: /dev/ser1
   - TCP: 5555
   - sends newline-delimited LiDAR JSON

2. QNX CAN gateway C application
   - UDP telemetry: 8080
   - sends CAN/safety/queue/CPU metrics as JSON

3. Python web dashboard
   - HTTP: 5000
   - browser based
   - NO matplotlib
   - NO numpy
   - NO tkinter
   - NO DISPLAY required

Open from a laptop:
    http://172.17.107.235:5000

QNX services:
    LiDAR C bridge: 127.0.0.1:5555
    CAN telemetry:  UDP 0.0.0.0:8080
    Web dashboard: HTTP 0.0.0.0:5000

Start the integrated C program first, then:
    python3 lidar_s.py

Only ONE C executable is required:
    ./qnx_vehicle_gateway

It owns both the LiDAR UART/TCP bridge and the dual-CAN gateway.

The dashboard provides:
    - LiDAR connection/status
    - Live polar LiDAR scan
    - distance / angle / X / Y / tilt
    - LiDAR point rate
    - CAN0/CAN1 RX FPS
    - CAN TX FPS
    - CAN bus load
    - safety latency/jitter
    - deadline misses
    - queue occupancy
    - wrong/correct prediction counts
    - error archive/overflow
    - CPU utilisation
    - START / STOP LiDAR control
    - CLEAR LiDAR display

This intentionally does NOT require a 2D floor map.
"""

import argparse
import json
import math
import socket
import threading
import time
from collections import deque

from flask import Flask, jsonify, render_template_string


# ============================================================================
# CONFIGURATION
# ============================================================================

LIDAR_HOST = "127.0.0.1"
LIDAR_PORT = 5555

UDP_HOST = "0.0.0.0"
UDP_PORT = 8080

WEB_HOST = "0.0.0.0"
WEB_PORT = 5000

MAX_LIDAR_POINTS = 2500
MAX_RECENT = 12

# Browser polar display range.
MAX_RANGE_CM = 800.0


# ============================================================================
# SHARED STATE
# ============================================================================

state_lock = threading.Lock()

lidar = {
    "connected": False,
    "error": "",
    "scanning": True,

    "total_points": 0,
    "rate": 0.0,

    "angle": 0.0,
    "motor_angle": 0.0,
    "distance": 0.0,

    "tilt1": None,
    "tilt2": None,

    "x": 0.0,
    "y": 0.0,

    "max_distance": 0.0,

    "last_update": 0.0,

    "sweep": deque(maxlen=MAX_LIDAR_POINTS),

    "recent": deque(maxlen=MAX_RECENT),
}


can = {
    "connected": False,
    "last_update": 0.0,
    "source": "NONE",

    "can0_rx_fps": 0.0,
    "can1_rx_fps": 0.0,
    "tx_fps": 0.0,

    "busload": 0.0,

    "safety_latency_ms": 0.0,
    "safety_jitter_ms": 0.0,
    "deadline_misses": 0,

    "q_safety": 0,
    "q_normal": 0,
    "q_noise": 0,
    "q_errors": 0,

    "wrong_predictions": 0,
    "correct_predictions": 0,

    "errors_archived": 0,
    "error_overflow": 0,

    "cpu0": 0.0,
    "cpu1": 0.0,
    "cpu2": 0.0,
    "cpu3": 0.0,

    "event": "WAITING FOR QNX TELEMETRY",
}


can_history = {
    "time": deque(maxlen=120),
    "busload": deque(maxlen=120),
    "tx_fps": deque(maxlen=120),
    "safety_latency": deque(maxlen=120),
}


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def number(data, *keys, default=0.0):
    for key in keys:
        if key in data:
            try:
                return float(data[key])
            except (TypeError, ValueError):
                pass
    return float(default)


def integer(data, *keys, default=0):
    return int(round(number(data, *keys, default=default)))


def text(data, *keys, default=""):
    for key in keys:
        if key in data:
            return str(data[key])
    return default


def clamp(value, low, high):
    return max(low, min(high, value))


# ============================================================================
# LIDAR TCP CLIENT
# ============================================================================

class LidarClient(threading.Thread):

    def __init__(self, host, port):
        super().__init__(daemon=True)

        self.host = host
        self.port = port

        self.sock = None
        self.sock_lock = threading.Lock()

        self.stop_event = threading.Event()

        self.count_total = 0
        self.count_previous = 0
        self.rate_time = time.time()

    # ------------------------------------------------------------------------

    def run(self):

        print("=" * 72)
        print("LiDAR TCP CLIENT")
        print("=" * 72)
        print(f"Target : {self.host}:{self.port}")
        print()

        while not self.stop_event.is_set():

            try:
                self.session()

            except Exception as exc:

                with state_lock:
                    lidar["connected"] = False
                    lidar["error"] = str(exc)

                print(f"[LIDAR] connection error: {exc}")

            if self.stop_event.is_set():
                break

            time.sleep(1.0)

    # ------------------------------------------------------------------------

    def session(self):

        print(
            f"[LIDAR] connecting to "
            f"{self.host}:{self.port}..."
        )

        sock = socket.create_connection(
            (self.host, self.port),
            timeout=5
        )

        sock.settimeout(1.0)

        with self.sock_lock:
            self.sock = sock

        with state_lock:
            lidar["connected"] = True
            lidar["error"] = ""

        print(
            f"[LIDAR] CONNECTED to "
            f"{self.host}:{self.port}"
        )

        buffer = b""

        try:

            while not self.stop_event.is_set():

                try:
                    chunk = sock.recv(8192)

                except socket.timeout:
                    self.update_rate()
                    continue

                if not chunk:
                    break

                buffer += chunk

                while b"\n" in buffer:

                    raw, buffer = buffer.split(
                        b"\n",
                        1
                    )

                    raw = raw.strip()

                    if not raw:
                        continue

                    try:

                        msg = json.loads(
                            raw.decode("utf-8")
                        )

                        self.handle_message(msg)

                    except (
                        json.JSONDecodeError,
                        UnicodeDecodeError,
                        ValueError,
                        TypeError
                    ):
                        continue

                self.update_rate()

        finally:

            with state_lock:
                lidar["connected"] = False

            with self.sock_lock:
                self.sock = None

            try:
                sock.close()
            except Exception:
                pass

            print("[LIDAR] disconnected")

    # ------------------------------------------------------------------------

    def handle_message(self, msg):

        kind = msg.get("t")

        if kind == "scan":

            distance = number(
                msg,
                "d",
                default=0
            )

            angle = number(
                msg,
                "a",
                default=0
            )

            motor_angle = number(
                msg,
                "am",
                default=angle
            )

            x = number(
                msg,
                "x",
                default=0
            )

            y = number(
                msg,
                "y",
                default=0
            )

            tilt1 = msg.get("t1")
            tilt2 = msg.get("t2")

            if distance <= 0:
                return

            # Ignore impossible readings for display.
            if distance > MAX_RANGE_CM:
                return

            now = time.time()

            point = {
                "angle": angle,
                "distance": distance,
                "x": x,
                "y": y,
                "tilt1": tilt1,
                "tilt2": tilt2,
            }

            with state_lock:

                lidar["total_points"] += 1

                lidar["angle"] = angle
                lidar["motor_angle"] = motor_angle
                lidar["distance"] = distance

                lidar["x"] = x
                lidar["y"] = y

                lidar["tilt1"] = tilt1
                lidar["tilt2"] = tilt2

                if distance > lidar["max_distance"]:
                    lidar["max_distance"] = distance

                lidar["sweep"].append(point)

                lidar["recent"].append({
                    "time": time.strftime("%H:%M:%S"),
                    "angle": angle,
                    "distance": distance,
                    "x": x,
                    "y": y,
                    "tilt1": tilt1,
                })

                lidar["last_update"] = now

            self.count_total += 1

    # ------------------------------------------------------------------------

    def update_rate(self):

        now = time.time()

        elapsed = now - self.rate_time

        if elapsed >= 1.0:

            rate = (
                self.count_total -
                self.count_previous
            ) / elapsed

            with state_lock:
                lidar["rate"] = rate

            self.count_previous = self.count_total
            self.rate_time = now

    # ------------------------------------------------------------------------

    def send_command(self, command):

        if command not in ("start", "stop"):
            return False

        with self.sock_lock:

            if self.sock is None:
                return False

            try:

                self.sock.sendall(
                    (
                        command + "\n"
                    ).encode("utf-8")
                )

                with state_lock:
                    lidar["scanning"] = (
                        command == "start"
                    )

                print(
                    f"[LIDAR] command -> {command}"
                )

                return True

            except Exception as exc:

                with state_lock:
                    lidar["error"] = str(exc)

                return False

    # ------------------------------------------------------------------------

    def stop(self):

        self.stop_event.set()

        with self.sock_lock:

            if self.sock is not None:

                try:
                    self.sock.shutdown(
                        socket.SHUT_RDWR
                    )
                except Exception:
                    pass


# ============================================================================
# CAN UDP RECEIVER
# ============================================================================

class CanTelemetryReceiver(threading.Thread):

    def __init__(self, host, port):

        super().__init__(daemon=True)

        self.host = host
        self.port = port

        self.stop_event = threading.Event()

        self.sock = None

    # ------------------------------------------------------------------------

    def run(self):

        print("=" * 72)
        print("CAN UDP TELEMETRY RECEIVER")
        print("=" * 72)
        print(
            f"Listening : UDP {self.host}:{self.port}"
        )
        print()

        try:

            sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_DGRAM
            )

            sock.setsockopt(
                socket.SOL_SOCKET,
                socket.SO_REUSEADDR,
                1
            )

            sock.bind(
                (
                    self.host,
                    self.port
                )
            )

            sock.settimeout(1.0)

            self.sock = sock

            print(
                f"[CAN] UDP telemetry listening "
                f"on {self.host}:{self.port}"
            )

        except Exception as exc:

            print(
                f"[CAN] Cannot bind UDP {self.port}: "
                f"{exc}"
            )

            return

        while not self.stop_event.is_set():

            try:

                packet, address = sock.recvfrom(
                    8192
                )

                try:

                    data = json.loads(
                        packet.decode(
                            "utf-8"
                        )
                    )

                except (
                    json.JSONDecodeError,
                    UnicodeDecodeError
                ):

                    continue

                self.update_can(
                    data,
                    f"{address[0]}:{address[1]}"
                )

            except socket.timeout:

                self.check_timeout()

            except Exception as exc:

                print(
                    f"[CAN] receiver error: {exc}"
                )

                time.sleep(0.2)

        try:
            sock.close()
        except Exception:
            pass

    # ------------------------------------------------------------------------

    def update_can(self, data, source):

        now = time.time()

        with state_lock:

            can["connected"] = True
            can["last_update"] = now
            can["source"] = source

            can["can0_rx_fps"] = number(
                data,
                "can0_rx_fps",
                "can0_fps"
            )

            can["can1_rx_fps"] = number(
                data,
                "can1_rx_fps",
                "can1_fps"
            )

            can["tx_fps"] = number(
                data,
                "tx_fps",
                "fps",
                "throughput"
            )

            can["busload"] = clamp(
                number(
                    data,
                    "busload",
                    "bus_load",
                    "bus_utilization"
                ),
                0.0,
                100.0
            )

            can["safety_latency_ms"] = number(
                data,
                "safety_latency_ms",
                "latency",
                "avg_latency"
            )

            can["safety_jitter_ms"] = number(
                data,
                "safety_jitter_ms",
                "jitter"
            )

            can["deadline_misses"] = integer(
                data,
                "deadline_misses"
            )

            can["q_safety"] = integer(
                data,
                "q_safety",
                "safety_queue"
            )

            can["q_normal"] = integer(
                data,
                "q_normal",
                "normal_queue"
            )

            can["q_noise"] = integer(
                data,
                "q_noise",
                "noise_queue"
            )

            can["q_errors"] = integer(
                data,
                "q_errors",
                "error_queue"
            )

            can["wrong_predictions"] = integer(
                data,
                "wrong_predictions",
                "wrong"
            )

            can["correct_predictions"] = integer(
                data,
                "correct_predictions",
                "correct"
            )

            can["errors_archived"] = integer(
                data,
                "errors_archived",
                "archived"
            )

            can["error_overflow"] = integer(
                data,
                "error_overflow",
                "overflow"
            )

            can["cpu0"] = number(data, "cpu0")
            can["cpu1"] = number(data, "cpu1")
            can["cpu2"] = number(data, "cpu2")
            can["cpu3"] = number(data, "cpu3")

            can["event"] = text(
                data,
                "event",
                default="NORMAL"
            )

            can_history["time"].append(
                time.strftime("%H:%M:%S")
            )

            can_history["busload"].append(
                can["busload"]
            )

            can_history["tx_fps"].append(
                can["tx_fps"]
            )

            can_history["safety_latency"].append(
                can["safety_latency_ms"]
            )

    # ------------------------------------------------------------------------

    def check_timeout(self):

        with state_lock:

            if (
                can["last_update"] > 0 and
                time.time() - can["last_update"] > 5
            ):

                can["connected"] = False

    # ------------------------------------------------------------------------

    def stop(self):

        self.stop_event.set()

        if self.sock is not None:

            try:
                self.sock.close()
            except Exception:
                pass


# ============================================================================
# FLASK APPLICATION
# ============================================================================

app = Flask(__name__)


# ============================================================================
# SNAPSHOT
# ============================================================================

def get_snapshot():

    with state_lock:

        lidar_copy = {
            "connected": lidar["connected"],
            "error": lidar["error"],
            "scanning": lidar["scanning"],

            "total_points": lidar["total_points"],
            "rate": lidar["rate"],

            "angle": lidar["angle"],
            "motor_angle": lidar["motor_angle"],
            "distance": lidar["distance"],

            "tilt1": lidar["tilt1"],
            "tilt2": lidar["tilt2"],

            "x": lidar["x"],
            "y": lidar["y"],

            "max_distance": lidar["max_distance"],

            "last_update": lidar["last_update"],

            "sweep": list(lidar["sweep"]),
            "recent": list(lidar["recent"]),
        }

        can_copy = dict(can)

        history_copy = {
            "time": list(can_history["time"]),
            "busload": list(can_history["busload"]),
            "tx_fps": list(can_history["tx_fps"]),
            "safety_latency": list(
                can_history["safety_latency"]
            ),
        }

    return {
        "lidar": lidar_copy,
        "can": can_copy,
        "history": history_copy,
    }


# ============================================================================
# API
# ============================================================================

@app.route("/")
def index():

    return render_template_string(
        DASHBOARD_HTML
    )


@app.route("/api/data")
def api_data():

    return jsonify(
        get_snapshot()
    )


@app.route("/api/command", methods=["POST"])
def api_command():

    from flask import request

    data = request.get_json(
        silent=True
    ) or {}

    command = data.get(
        "command"
    )

    if LIDAR_CLIENT is None:

        return jsonify({
            "success": False,
            "error": "LiDAR client not started"
        }), 503

    success = LIDAR_CLIENT.send_command(
        command
    )

    return jsonify({
        "success": success
    })


@app.route("/api/clear", methods=["POST"])
def api_clear():

    with state_lock:

        lidar["sweep"].clear()
        lidar["recent"].clear()

        lidar["total_points"] = 0
        lidar["rate"] = 0.0
        lidar["max_distance"] = 0.0

    return jsonify({
        "success": True
    })


# ============================================================================
# HTML DASHBOARD
# ============================================================================

DASHBOARD_HTML = r"""
<!DOCTYPE html>

<html>

<head>

<meta charset="UTF-8">

<meta name="viewport"
      content="width=device-width, initial-scale=1.0">

<title>QNX Vehicle Gateway</title>

<style>

* {
    box-sizing: border-box;
}

body {
    margin: 0;
    background: #0b1120;
    color: #f8fafc;
    font-family: Arial, Helvetica, sans-serif;
}

.header {
    padding: 20px 28px;
    background: #111827;
    border-bottom: 1px solid #334155;
}

.header h1 {
    margin: 0;
    font-size: 26px;
}

.header p {
    margin: 6px 0 0;
    color: #94a3b8;
}

.container {
    max-width: 1500px;
    margin: auto;
    padding: 20px;
}

.section-title {
    margin: 25px 0 12px;
    font-size: 20px;
}

.cards {
    display: grid;
    grid-template-columns:
        repeat(auto-fit, minmax(170px, 1fr));
    gap: 12px;
}

.card {
    background: #111827;
    border: 1px solid #334155;
    border-radius: 12px;
    padding: 16px;
}

.label {
    font-size: 12px;
    color: #94a3b8;
    text-transform: uppercase;
    margin-bottom: 8px;
}

.value {
    font-size: 24px;
    font-weight: bold;
}

.online {
    color: #22c55e;
}

.offline {
    color: #ef4444;
}

.warning {
    color: #f59e0b;
}

.safe {
    color: #22c55e;
}

.danger {
    color: #ef4444;
}

.main-grid {
    display: grid;
    grid-template-columns:
        minmax(420px, 1fr)
        minmax(420px, 1fr);
    gap: 18px;
}

.panel {
    background: #111827;
    border: 1px solid #334155;
    border-radius: 12px;
    padding: 18px;
}

.panel h2 {
    margin-top: 0;
    font-size: 18px;
}

canvas {
    width: 100%;
    max-width: 620px;
    display: block;
    margin: auto;
    background: #070d18;
    border-radius: 10px;
}

.controls {
    display: flex;
    gap: 10px;
    margin-top: 15px;
    flex-wrap: wrap;
}

button {
    border: 0;
    border-radius: 8px;
    padding: 11px 22px;
    font-size: 14px;
    font-weight: bold;
    cursor: pointer;
}

.start {
    background: #22c55e;
    color: #052e16;
}

.stop {
    background: #f59e0b;
    color: #451a03;
}

.clear {
    background: #ef4444;
    color: white;
}

button:hover {
    opacity: 0.85;
}

.data-grid {
    display: grid;
    grid-template-columns:
        repeat(2, 1fr);
    gap: 10px;
}

.data-item {
    background: #0b1120;
    border-radius: 8px;
    padding: 12px;
}

.data-item .name {
    color: #94a3b8;
    font-size: 12px;
}

.data-item .number {
    font-size: 20px;
    font-weight: bold;
    margin-top: 5px;
}

.event {
    margin-top: 15px;
    padding: 13px;
    background: #0b1120;
    border-radius: 8px;
    border-left: 4px solid #22c55e;
}

table {
    width: 100%;
    border-collapse: collapse;
    font-size: 13px;
}

th, td {
    padding: 8px;
    border-bottom: 1px solid #334155;
    text-align: right;
}

th {
    color: #94a3b8;
}

.info {
    margin-top: 12px;
    color: #64748b;
    font-size: 12px;
}

@media(max-width: 900px) {

    .main-grid {
        grid-template-columns: 1fr;
    }

}

</style>

</head>


<body>


<div class="header">

    <h1>
        QNX Vehicle Gateway Dashboard
    </h1>

    <p>
        LiDAR + CAN + Safety Telemetry
    </p>

</div>


<div class="container">


<!-- ========================================================= -->
<!-- SYSTEM CONNECTION -->
<!-- ========================================================= -->

<div class="section-title">
    System Status
</div>

<div class="cards">

    <div class="card">
        <div class="label">LiDAR</div>
        <div id="lidarConnection"
             class="value offline">
            OFFLINE
        </div>
    </div>

    <div class="card">
        <div class="label">CAN Telemetry</div>
        <div id="canConnection"
             class="value offline">
            OFFLINE
        </div>
    </div>

    <div class="card">
        <div class="label">LiDAR Scanner</div>
        <div id="scanner"
             class="value warning">
            STOPPED
        </div>
    </div>

    <div class="card">
        <div class="label">LiDAR Point Rate</div>
        <div id="lidarRate"
             class="value">
            0 pt/s
        </div>
    </div>

    <div class="card">
        <div class="label">CAN Bus Load</div>
        <div id="busLoad"
             class="value">
            0 %
        </div>
    </div>

    <div class="card">
        <div class="label">Safety Latency</div>
        <div id="safetyLatency"
             class="value">
            0 ms
        </div>
    </div>

    <div class="card">
        <div class="label">Deadline Misses</div>
        <div id="deadlineMisses"
             class="value">
            0
        </div>
    </div>

    <div class="card">
        <div class="label">Event</div>
        <div id="event"
             class="value"
             style="font-size:15px;">
            WAITING
        </div>
    </div>

</div>


<!-- ========================================================= -->
<!-- LIDAR + CAN -->
<!-- ========================================================= -->

<div class="section-title">
    Live Gateway Data
</div>


<div class="main-grid">


<!-- ========================================================= -->
<!-- LIDAR PANEL -->
<!-- ========================================================= -->

<div class="panel">

    <h2>
        Live LiDAR Scan
    </h2>

    <canvas
        id="polarCanvas"
        width="620"
        height="620">
    </canvas>


    <div class="controls">

        <button
            class="start"
            onclick="command('start')">
            START
        </button>

        <button
            class="stop"
            onclick="command('stop')">
            STOP
        </button>

        <button
            class="clear"
            onclick="clearScan()">
            CLEAR
        </button>

    </div>

    <div class="info">
        Live polar view only. A 2D floor map is intentionally
        not required for this dashboard.
    </div>

</div>


<!-- ========================================================= -->
<!-- LIDAR DATA -->
<!-- ========================================================= -->

<div class="panel">

    <h2>
        LiDAR Measurements
    </h2>

    <div class="data-grid">

        <div class="data-item">
            <div class="name">Angle</div>
            <div id="angle"
                 class="number">
                0°
            </div>
        </div>

        <div class="data-item">
            <div class="name">Distance</div>
            <div id="distance"
                 class="number">
                0 cm
            </div>
        </div>

        <div class="data-item">
            <div class="name">X</div>
            <div id="x"
                 class="number">
                0 cm
            </div>
        </div>

        <div class="data-item">
            <div class="name">Y</div>
            <div id="y"
                 class="number">
                0 cm
            </div>
        </div>

        <div class="data-item">
            <div class="name">Tilt 1</div>
            <div id="tilt1"
                 class="number">
                --
            </div>
        </div>

        <div class="data-item">
            <div class="name">Tilt 2</div>
            <div id="tilt2"
                 class="number">
                --
            </div>
        </div>

        <div class="data-item">
            <div class="name">Total Points</div>
            <div id="totalPoints"
                 class="number">
                0
            </div>
        </div>

        <div class="data-item">
            <div class="name">Maximum Distance</div>
            <div id="maxDistance"
                 class="number">
                0 cm
            </div>
        </div>

    </div>


    <h2 style="margin-top:25px;">
        Recent LiDAR Measurements
    </h2>

    <table>

        <thead>
            <tr>
                <th>Time</th>
                <th>Angle</th>
                <th>Distance</th>
            </tr>
        </thead>

        <tbody id="recentTable">
        </tbody>

    </table>

</div>


</div>


<!-- ========================================================= -->
<!-- CAN METRICS -->
<!-- ========================================================= -->

<div class="section-title">
    CAN / Safety Metrics
</div>


<div class="cards">

    <div class="card">
        <div class="label">CAN0 RX</div>
        <div id="can0rx"
             class="value">
            0 fps
        </div>
    </div>

    <div class="card">
        <div class="label">CAN1 RX</div>
        <div id="can1rx"
             class="value">
            0 fps
        </div>
    </div>

    <div class="card">
        <div class="label">CAN TX</div>
        <div id="cantx"
             class="value">
            0 fps
        </div>
    </div>

    <div class="card">
        <div class="label">Safety Jitter</div>
        <div id="jitter"
             class="value">
            0 ms
        </div>
    </div>

    <div class="card">
        <div class="label">Safety Queue</div>
        <div id="qsafety"
             class="value">
            0
        </div>
    </div>

    <div class="card">
        <div class="label">Normal Queue</div>
        <div id="qnormal"
             class="value">
            0
        </div>
    </div>

    <div class="card">
        <div class="label">Noise Queue</div>
        <div id="qnoise"
             class="value">
            0
        </div>
    </div>

    <div class="card">
        <div class="label">Error Queue</div>
        <div id="qerrors"
             class="value">
            0
        </div>
    </div>

</div>


<!-- ========================================================= -->
<!-- RESOURCE / PREDICTION -->
<!-- ========================================================= -->

<div class="section-title">
    Gateway Resource & Prediction State
</div>


<div class="main-grid">


<div class="panel">

    <h2>
        Prediction / Error Handling
    </h2>

    <div class="data-grid">

        <div class="data-item">
            <div class="name">
                Wrong Predictions
            </div>

            <div id="wrong"
                 class="number">
                0
            </div>
        </div>

        <div class="data-item">
            <div class="name">
                Correct Predictions
            </div>

            <div id="correct"
                 class="number">
                0
            </div>
        </div>

        <div class="data-item">
            <div class="name">
                Errors Archived
            </div>

            <div id="archived"
                 class="number">
                0
            </div>
        </div>

        <div class="data-item">
            <div class="name">
                Error Overflow
            </div>

            <div id="overflow"
                 class="number">
                0
            </div>
        </div>

    </div>


    <div class="event">

        <b>Current Gateway Event</b>

        <div id="eventLarge"
             style="margin-top:7px;">
            WAITING FOR QNX TELEMETRY
        </div>

    </div>

</div>


<div class="panel">

    <h2>
        CPU Utilisation
    </h2>

    <div class="data-grid">

        <div class="data-item">
            <div class="name">CPU 0</div>
            <div id="cpu0"
                 class="number">
                0 %
            </div>
        </div>

        <div class="data-item">
            <div class="name">CPU 1</div>
            <div id="cpu1"
                 class="number">
                0 %
            </div>
        </div>

        <div class="data-item">
            <div class="name">CPU 2</div>
            <div id="cpu2"
                 class="number">
                0 %
            </div>
        </div>

        <div class="data-item">
            <div class="name">CPU 3</div>
            <div id="cpu3"
                 class="number">
                0 %
            </div>
        </div>

    </div>


    <div class="info">
        CAN telemetry source:
        <span id="canSource">NONE</span>
    </div>

</div>


</div>


<div class="info"
     style="margin-top:25px;">

    QNX LiDAR TCP: 5555 |
    CAN UDP telemetry: 8080 |
    Web dashboard: 5000

</div>


</div>


<script>


// =============================================================
// CANVAS
// =============================================================

const canvas =
    document.getElementById(
        "polarCanvas"
    );

const ctx =
    canvas.getContext(
        "2d"
    );


// =============================================================
// POLAR DRAWING
// =============================================================

function drawPolar(points)
{

    const w = canvas.width;
    const h = canvas.height;

    const cx = w / 2;
    const cy = h / 2;

    const radius =
        Math.min(
            cx,
            cy
        ) - 45;


    ctx.clearRect(
        0,
        0,
        w,
        h
    );


    ctx.fillStyle =
        "#070d18";

    ctx.fillRect(
        0,
        0,
        w,
        h
    );


    // ---------------------------------------------------------
    // RANGE RINGS
    // ---------------------------------------------------------

    ctx.strokeStyle =
        "#334155";

    ctx.lineWidth = 1;


    for (
        let i = 1;
        i <= 4;
        i++
    )
    {

        const r =
            radius *
            i / 4;


        ctx.beginPath();

        ctx.arc(
            cx,
            cy,
            r,
            0,
            2 * Math.PI
        );

        ctx.stroke();


        ctx.fillStyle =
            "#64748b";

        ctx.font =
            "12px Arial";

        ctx.fillText(
            Math.round(
                i * 200
            ) + " cm",
            cx + 6,
            cy - r + 15
        );

    }


    // ---------------------------------------------------------
    // CROSS HAIR
    // ---------------------------------------------------------

    ctx.strokeStyle =
        "#475569";


    ctx.beginPath();

    ctx.moveTo(
        cx - radius,
        cy
    );

    ctx.lineTo(
        cx + radius,
        cy
    );

    ctx.stroke();


    ctx.beginPath();

    ctx.moveTo(
        cx,
        cy - radius
    );

    ctx.lineTo(
        cx,
        cy + radius
    );

    ctx.stroke();


    // ---------------------------------------------------------
    // DIRECTION LABELS
    // ---------------------------------------------------------

    ctx.fillStyle =
        "#94a3b8";

    ctx.font =
        "14px Arial";


    ctx.fillText(
        "0°",
        cx + 7,
        cy - radius - 8
    );


    ctx.fillText(
        "90°",
        cx + radius + 5,
        cy + 5
    );


    ctx.fillText(
        "180°",
        cx - 18,
        cy + radius + 22
    );


    ctx.fillText(
        "270°",
        cx - radius - 35,
        cy + 5
    );


    // ---------------------------------------------------------
    // POINTS
    // ---------------------------------------------------------

    ctx.fillStyle =
        "#22c55e";


    const maxRange =
        800.0;


    for (
        const p of points
    )
    {

        const angle =
            Number(
                p.angle
            ) *
            Math.PI /
            180.0;


        const distance =
            Number(
                p.distance
            );


        const r =
            Math.min(
                distance /
                maxRange,
                1
            ) *
            radius;


        // 0 degrees = up
        const px =
            cx +
            Math.sin(angle) *
            r;


        const py =
            cy -
            Math.cos(angle) *
            r;


        ctx.beginPath();

        ctx.arc(
            px,
            py,
            2.8,
            0,
            2 * Math.PI
        );

        ctx.fill();

    }


    // ---------------------------------------------------------
    // SENSOR POSITION
    // ---------------------------------------------------------

    ctx.fillStyle =
        "#ef4444";


    ctx.beginPath();

    ctx.arc(
        cx,
        cy,
        6,
        0,
        2 * Math.PI
    );

    ctx.fill();

}


// =============================================================
// FORMAT
// =============================================================

function fmt(
    value,
    digits = 1
)
{

    const n =
        Number(value);

    if (
        !Number.isFinite(n)
    )
        return "0";


    return n.toFixed(
        digits
    );

}


// =============================================================
// DASHBOARD UPDATE
// =============================================================

function updateDashboard(data)
{

    const l =
        data.lidar;

    const c =
        data.can;


    // ---------------------------------------------------------
    // LIDAR CONNECTION
    // ---------------------------------------------------------

    const lidarConnection =
        document.getElementById(
            "lidarConnection"
        );


    if (l.connected)
    {

        lidarConnection.textContent =
            "ONLINE";

        lidarConnection.className =
            "value online";

    }
    else
    {

        lidarConnection.textContent =
            "OFFLINE";

        lidarConnection.className =
            "value offline";

    }


    // ---------------------------------------------------------
    // CAN CONNECTION
    // ---------------------------------------------------------

    const canConnection =
        document.getElementById(
            "canConnection"
        );


    if (c.connected)
    {

        canConnection.textContent =
            "ONLINE";

        canConnection.className =
            "value online";

    }
    else
    {

        canConnection.textContent =
            "OFFLINE";

        canConnection.className =
            "value offline";

    }


    // ---------------------------------------------------------
    // SCANNER
    // ---------------------------------------------------------

    const scanner =
        document.getElementById(
            "scanner"
        );


    if (l.scanning)
    {

        scanner.textContent =
            "SCANNING";

        scanner.className =
            "value safe";

    }
    else
    {

        scanner.textContent =
            "STOPPED";

        scanner.className =
            "value warning";

    }


    // ---------------------------------------------------------
    // TOP CARDS
    // ---------------------------------------------------------

    document.getElementById(
        "lidarRate"
    ).textContent =
        fmt(l.rate, 0) +
        " pt/s";


    document.getElementById(
        "busLoad"
    ).textContent =
        fmt(c.busload, 1) +
        " %";


    document.getElementById(
        "safetyLatency"
    ).textContent =
        fmt(
            c.safety_latency_ms,
            3
        ) +
        " ms";


    document.getElementById(
        "deadlineMisses"
    ).textContent =
        c.deadline_misses;


    document.getElementById(
        "event"
    ).textContent =
        c.event;


    // ---------------------------------------------------------
    // LIDAR
    // ---------------------------------------------------------

    document.getElementById(
        "angle"
    ).textContent =
        fmt(l.angle, 2) +
        "°";


    document.getElementById(
        "distance"
    ).textContent =
        fmt(l.distance, 2) +
        " cm";


    document.getElementById(
        "x"
    ).textContent =
        fmt(l.x, 2) +
        " cm";


    document.getElementById(
        "y"
    ).textContent =
        fmt(l.y, 2) +
        " cm";


    document.getElementById(
        "tilt1"
    ).textContent =
        l.tilt1 === null
            ? "--"
            : l.tilt1 + "°";


    document.getElementById(
        "tilt2"
    ).textContent =
        l.tilt2 === null
            ? "--"
            : l.tilt2 + "°";


    document.getElementById(
        "totalPoints"
    ).textContent =
        l.total_points;


    document.getElementById(
        "maxDistance"
    ).textContent =
        fmt(
            l.max_distance,
            1
        ) +
        " cm";


    drawPolar(
        l.sweep
    );


    // ---------------------------------------------------------
    // RECENT TABLE
    // ---------------------------------------------------------

    const table =
        document.getElementById(
            "recentTable"
        );


    table.innerHTML = "";


    const recent =
        l.recent
            .slice()
            .reverse();


    for (
        const p of recent
    )
    {

        const row =
            document.createElement(
                "tr"
            );


        row.innerHTML =
            "<td>" +
            p.time +
            "</td>" +

            "<td>" +
            fmt(
                p.angle,
                1
            ) +
            "°</td>" +

            "<td>" +
            fmt(
                p.distance,
                1
            ) +
            " cm</td>";


        table.appendChild(
            row
        );

    }


    // ---------------------------------------------------------
    // CAN
    // ---------------------------------------------------------

    document.getElementById(
        "can0rx"
    ).textContent =
        fmt(
            c.can0_rx_fps,
            0
        ) +
        " fps";


    document.getElementById(
        "can1rx"
    ).textContent =
        fmt(
            c.can1_rx_fps,
            0
        ) +
        " fps";


    document.getElementById(
        "cantx"
    ).textContent =
        fmt(
            c.tx_fps,
            0
        ) +
        " fps";


    document.getElementById(
        "jitter"
    ).textContent =
        fmt(
            c.safety_jitter_ms,
            3
        ) +
        " ms";


    document.getElementById(
        "qsafety"
    ).textContent =
        c.q_safety;


    document.getElementById(
        "qnormal"
    ).textContent =
        c.q_normal;


    document.getElementById(
        "qnoise"
    ).textContent =
        c.q_noise;


    document.getElementById(
        "qerrors"
    ).textContent =
        c.q_errors;


    // ---------------------------------------------------------
    // PREDICTION
    // ---------------------------------------------------------

    document.getElementById(
        "wrong"
    ).textContent =
        c.wrong_predictions;


    document.getElementById(
        "correct"
    ).textContent =
        c.correct_predictions;


    document.getElementById(
        "archived"
    ).textContent =
        c.errors_archived;


    document.getElementById(
        "overflow"
    ).textContent =
        c.error_overflow;


    document.getElementById(
        "eventLarge"
    ).textContent =
        c.event;


    // ---------------------------------------------------------
    // CPU
    // ---------------------------------------------------------

    document.getElementById(
        "cpu0"
    ).textContent =
        fmt(
            c.cpu0,
            1
        ) +
        " %";


    document.getElementById(
        "cpu1"
    ).textContent =
        fmt(
            c.cpu1,
            1
        ) +
        " %";


    document.getElementById(
        "cpu2"
    ).textContent =
        fmt(
            c.cpu2,
            1
        ) +
        " %";


    document.getElementById(
        "cpu3"
    ).textContent =
        fmt(
            c.cpu3,
            1
        ) +
        " %";


    document.getElementById(
        "canSource"
    ).textContent =
        c.source;

}


// =============================================================
// POLLING
// =============================================================

async function update()
{

    try
    {

        const response =
            await fetch(
                "/api/data",
                {
                    cache: "no-store"
                }
            );


        const data =
            await response.json();


        updateDashboard(
            data
        );

    }
    catch(error)
    {

        console.log(
            "Dashboard update error:",
            error
        );

    }

}


setInterval(
    update,
    200
);


update();


// =============================================================
// COMMAND
// =============================================================

async function command(
    value
)
{

    try
    {

        const response =
            await fetch(
                "/api/command",
                {
                    method: "POST",

                    headers: {
                        "Content-Type":
                            "application/json"
                    },

                    body:
                        JSON.stringify({
                            command:
                                value
                        })
                }
            );


        const result =
            await response.json();


        if (!result.success)
        {

            console.log(
                "Command failed:",
                result.error
            );

        }

    }
    catch(error)
    {

        console.log(
            "Command error:",
            error
        );

    }

}


// =============================================================
// CLEAR
// =============================================================

async function clearScan()
{

    try
    {

        await fetch(
            "/api/clear",
            {
                method: "POST"
            }
        );

    }
    catch(error)
    {

        console.log(
            "Clear error:",
            error
        );

    }

}

</script>


</body>

</html>
"""


# ============================================================================
# MAIN
# ============================================================================

LIDAR_CLIENT = None
CAN_RECEIVER = None


def main():

    global LIDAR_CLIENT
    global CAN_RECEIVER

    parser = argparse.ArgumentParser(
        description=
        "Unified QNX LiDAR + CAN web dashboard"
    )

    parser.add_argument(
        "--lidar-host",
        default=LIDAR_HOST
    )

    parser.add_argument(
        "--lidar-port",
        type=int,
        default=LIDAR_PORT
    )

    parser.add_argument(
        "--udp-host",
        default=UDP_HOST
    )

    parser.add_argument(
        "--udp-port",
        type=int,
        default=UDP_PORT
    )

    parser.add_argument(
        "--web-host",
        default=WEB_HOST
    )

    parser.add_argument(
        "--web-port",
        type=int,
        default=WEB_PORT
    )

    args = parser.parse_args()

    print()
    print("=" * 72)
    print("             QNX VEHICLE GATEWAY DASHBOARD")
    print("=" * 72)
    print()
    print(
        f"LiDAR TCP      : "
        f"{args.lidar_host}:{args.lidar_port}"
    )
    print(
        f"CAN UDP        : "
        f"{args.udp_host}:{args.udp_port}"
    )
    print(
        f"Web dashboard  : "
        f"{args.web_host}:{args.web_port}"
    )
    print()
    print(
        "Browser:"
    )
    print(
        "    http://172.17.107.235:5000"
    )
    print()
    print(
        "No Matplotlib / NumPy / Tkinter required."
    )
    print("=" * 72)
    print()

    # ------------------------------------------------------------
    # LiDAR TCP client
    # ------------------------------------------------------------

    LIDAR_CLIENT = LidarClient(
        args.lidar_host,
        args.lidar_port
    )

    LIDAR_CLIENT.start()

    # ------------------------------------------------------------
    # CAN UDP receiver
    # ------------------------------------------------------------

    CAN_RECEIVER = CanTelemetryReceiver(
        args.udp_host,
        args.udp_port
    )

    CAN_RECEIVER.start()

    # ------------------------------------------------------------
    # Web server
    # ------------------------------------------------------------

    try:

        print(
            f"[WEB] Starting dashboard on "
            f"{args.web_host}:{args.web_port}"
        )

        app.run(
            host=args.web_host,
            port=args.web_port,
            threaded=True,
            debug=False,
            use_reloader=False
        )

    except KeyboardInterrupt:

        print()
        print(
            "[WEB] stopping..."
        )

    finally:

        if LIDAR_CLIENT is not None:
            LIDAR_CLIENT.stop()

        if CAN_RECEIVER is not None:
            CAN_RECEIVER.stop()


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    main()
