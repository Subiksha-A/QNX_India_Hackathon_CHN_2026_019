#!/usr/bin/env python3
"""
QNX Vehicle Gateway - Real-Time Performance Dashboard
======================================================

Purpose
-------
A lightweight student-friendly dashboard for the QNX CAN gateway demo.

The dashboard can:
1. Receive telemetry from the QNX gateway over UDP.
2. Display real-time bus load, frame rate, dropped frames,
   safety latency, CPU utilisation, queue/buffer occupancy and
   safety-frame status.
3. Keep a short in-memory history for plotting.
4. Work in DEMO MODE when the QNX gateway is not connected.

Expected UDP telemetry
----------------------
The QNX program should ideally send JSON such as:

{
    "bus_load": 92.4,
    "fps": 801,
    "dropped_frames": 407275,
    "safety_latency_ms": 2.31,
    "cpu_util": 1.8,
    "queue_occupancy": 72.0,
    "safety_frames": 100,
    "safety_drops": 0,
    "event": "0x010 BRAKE PREEMPTED"
}

The parser also accepts common alternate key names, so small
changes in the C telemetry format do not require rewriting the
dashboard.

Requirements
------------
Python 3.9+
Flask

Install:
    python -m pip install flask

Run:
    python qnx_gateway_dashboard.py

Open on the development laptop:
    http://127.0.0.1:5000

For another machine on the same network:
    http://<laptop-ip>:5000

UDP listener:
    0.0.0.0:8080

Architecture
------------
QNX Gateway
    |
    | UDP telemetry :8080
    v
Python receiver thread
    |
    v
Thread-safe telemetry state
    |
    v
Flask dashboard :5000
"""

import json
import math
import random
import socket
import threading
import time
from collections import deque
from typing import Any, Dict

from flask import Flask, jsonify, render_template_string


# ============================================================================
# CONFIGURATION
# ============================================================================

UDP_HOST = "0.0.0.0"
UDP_PORT = 8080

WEB_HOST = "0.0.0.0"
WEB_PORT = 5000

# Keep only a small amount of history.
# This prevents the dashboard itself from becoming a memory problem.
HISTORY_LENGTH = 120

# Set True if you want to test the dashboard without the QNX gateway.
DEMO_MODE = True

# Demo update period.
DEMO_PERIOD_SEC = 1.0


# ============================================================================
# GLOBAL TELEMETRY STATE
# ============================================================================

state_lock = threading.Lock()

telemetry = {
    "bus_load": 0.0,
    "fps": 0.0,
    "dropped_frames": 0,
    "safety_latency_ms": 0.0,
    "cpu_util": 0.0,
    "queue_occupancy": 0.0,
    "safety_frames": 0,
    "safety_drops": 0,
    "event": "WAITING FOR QNX TELEMETRY",
    "timestamp": 0.0,
    "source": "NONE",
}

history = {
    "time": deque(maxlen=HISTORY_LENGTH),
    "bus_load": deque(maxlen=HISTORY_LENGTH),
    "fps": deque(maxlen=HISTORY_LENGTH),
    "latency": deque(maxlen=HISTORY_LENGTH),
    "cpu": deque(maxlen=HISTORY_LENGTH),
    "queue": deque(maxlen=HISTORY_LENGTH),
}

app = Flask(__name__)


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def number(data: Dict[str, Any], *keys, default=0.0) -> float:
    """Read a numeric value using several possible key names."""
    for key in keys:
        if key in data:
            try:
                return float(data[key])
            except (TypeError, ValueError):
                pass
    return float(default)


def integer(data: Dict[str, Any], *keys, default=0) -> int:
    """Read an integer using several possible key names."""
    return int(round(number(data, *keys, default=default)))


def text(data: Dict[str, Any], *keys, default="") -> str:
    """Read a string using several possible key names."""
    for key in keys:
        if key in data:
            return str(data[key])
    return default


def clamp(value: float, low: float, high: float) -> float:
    """Keep a number inside a display-safe range."""
    return max(low, min(high, value))


def update_state(data: Dict[str, Any], source: str = "QNX") -> None:
    """
    Update the shared telemetry state.

    This function is deliberately separated from the network receiver
    so that the data processing is easy to explain to the jury.
    """

    now = time.time()

    new_values = {
        "bus_load": clamp(
            number(data, "bus_load", "busload", "bus_utilization"),
            0.0,
            100.0,
        ),

        "fps": max(
            0.0,
            number(data, "fps", "frame_rate", "throughput")
        ),

        "dropped_frames": max(
            0,
            integer(
                data,
                "dropped_frames",
                "dropped_noise_frames",
                "noise_drops",
            ),
        ),

        "safety_latency_ms": max(
            0.0,
            number(
                data,
                "safety_latency_ms",
                "latency_ms",
                "e2e_latency_ms",
            ),
        ),

        "cpu_util": clamp(
            number(data, "cpu_util", "cpu_usage", "cpu_percent"),
            0.0,
            100.0,
        ),

        "queue_occupancy": clamp(
            number(
                data,
                "queue_occupancy",
                "buffer_occupancy",
                "buffer_percent",
            ),
            0.0,
            100.0,
        ),

        "safety_frames": max(
            0,
            integer(
                data,
                "safety_frames",
                "safety_delivered",
                "safety_count",
            ),
        ),

        "safety_drops": max(
            0,
            integer(
                data,
                "safety_drops",
                "dropped_safety_frames",
            ),
        ),

        "event": text(
            data,
            "event",
            "status",
            "last_event",
            default="TELEMETRY RECEIVED",
        ),

        "timestamp": now,
        "source": source,
    }

    with state_lock:
        telemetry.update(new_values)

        history["time"].append(time.strftime("%H:%M:%S"))
        history["bus_load"].append(new_values["bus_load"])
        history["fps"].append(new_values["fps"])
        history["latency"].append(new_values["safety_latency_ms"])
        history["cpu"].append(new_values["cpu_util"])
        history["queue"].append(new_values["queue_occupancy"])


def parse_telemetry(raw: bytes) -> Dict[str, Any]:
    """
    Parse telemetry received from QNX.

    Primary format:
        JSON

    Example:
        {"bus_load":92.4,"fps":801,"safety_latency_ms":2.31}

    Fallback:
        key=value,key=value,key=value

    Example:
        bus_load=92.4,fps=801,safety_latency_ms=2.31
    """

    message = raw.decode("utf-8", errors="replace").strip()

    if not message:
        return {}

    # ---- JSON format -------------------------------------------------------
    try:
        parsed = json.loads(message)
        if isinstance(parsed, dict):
            return parsed
    except json.JSONDecodeError:
        pass

    # ---- key=value fallback ------------------------------------------------
    result = {}

    for item in message.split(","):
        if "=" not in item:
            continue

        key, value = item.split("=", 1)
        key = key.strip()
        value = value.strip()

        try:
            result[key] = float(value)
        except ValueError:
            result[key] = value

    return result


# ============================================================================
# UDP RECEIVER
# ============================================================================

def udp_receiver() -> None:
    """
    Receive telemetry from the QNX gateway.

    A dedicated Python thread is used so that UDP reception never
    blocks Flask's web-server thread.
    """

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Reuse the socket after restarting the dashboard.
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    sock.bind((UDP_HOST, UDP_PORT))

    print(f"[UDP] Listening on {UDP_HOST}:{UDP_PORT}")

    while True:
        try:
            packet, address = sock.recvfrom(8192)

            data = parse_telemetry(packet)

            if data:
                update_state(data, source=f"{address[0]}:{address[1]}")

        except Exception as exc:
            print(f"[UDP] Receiver error: {exc}")
            time.sleep(0.2)


# ============================================================================
# DEMO DATA GENERATOR
# ============================================================================

def demo_generator() -> None:
    """
    Generate realistic-looking data for presentation development.

    IMPORTANT:
    This is only for dashboard development/demo testing.
    Jury measurements should come from the actual QNX gateway.
    """

    phase = 0

    while True:
        phase += 1

        # Slowly move between normal and congested conditions.
        cycle = phase % 30

        if cycle < 10:
            # Nominal
            bus_load = 15.0 + random.uniform(-2.0, 2.0)
            fps = 40.0 + random.uniform(-5.0, 5.0)
            latency = 1.15 + random.uniform(-0.15, 0.15)
            queue = random.uniform(5.0, 20.0)
            event = "NORMAL TRAFFIC"

        elif cycle < 20:
            # Flood / congestion
            bus_load = 92.0 + random.uniform(-1.0, 1.0)
            fps = 800.0 + random.uniform(-30.0, 30.0)
            latency = 2.15 + random.uniform(-0.20, 0.20)
            queue = 70.0 + random.uniform(-5.0, 8.0)
            event = "0x450 FLOOD - LOAD SHEDDING ACTIVE"

        else:
            # Safety preemption
            bus_load = 92.0 + random.uniform(-1.0, 1.0)
            fps = 800.0 + random.uniform(-30.0, 30.0)
            latency = 2.30 + random.uniform(-0.15, 0.15)
            queue = 65.0 + random.uniform(-5.0, 5.0)
            event = "0x010 BRAKE PREEMPTED"

        current_drops = int(max(0, (phase - 10) * 18000))

        update_state(
            {
                "bus_load": bus_load,
                "fps": fps,
                "dropped_frames": current_drops,
                "safety_latency_ms": latency,
                "cpu_util": 1.8 + random.uniform(-0.3, 0.3),
                "queue_occupancy": queue,
                "safety_frames": phase,
                "safety_drops": 0,
                "event": event,
            },
            source="DEMO",
        )

        time.sleep(DEMO_PERIOD_SEC)


# ============================================================================
# FLASK API
# ============================================================================

@app.route("/api/telemetry")
def api_telemetry():
    """Return current telemetry as JSON."""
    with state_lock:
        return jsonify(dict(telemetry))


@app.route("/api/history")
def api_history():
    """Return short historical telemetry for dashboard plots."""
    with state_lock:
        return jsonify(
            {
                "time": list(history["time"]),
                "bus_load": list(history["bus_load"]),
                "fps": list(history["fps"]),
                "latency": list(history["latency"]),
                "cpu": list(history["cpu"]),
                "queue": list(history["queue"]),
            }
        )


# ============================================================================
# DASHBOARD HTML
# ============================================================================

HTML = r"""
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">

<title>QNX Vehicle Gateway Dashboard</title>

<style>
    * {
        box-sizing: border-box;
    }

    body {
        margin: 0;
        background: #0f141a;
        color: #e9eef3;
        font-family: Arial, Helvetica, sans-serif;
    }

    header {
        padding: 18px 28px;
        border-bottom: 1px solid #2b343e;
        background: #151b22;
    }

    header h1 {
        margin: 0;
        font-size: 24px;
    }

    header p {
        margin: 6px 0 0;
        color: #98a5b3;
        font-size: 13px;
    }

    .container {
        max-width: 1400px;
        margin: auto;
        padding: 22px;
    }

    .status {
        display: flex;
        justify-content: space-between;
        align-items: center;
        margin-bottom: 18px;
        padding: 12px 16px;
        border: 1px solid #2b343e;
        border-radius: 10px;
        background: #151b22;
    }

    .status-left {
        display: flex;
        gap: 10px;
        align-items: center;
    }

    .dot {
        width: 10px;
        height: 10px;
        border-radius: 50%;
        background: #35d07f;
    }

    .grid {
        display: grid;
        grid-template-columns: repeat(4, 1fr);
        gap: 14px;
    }

    .card {
        padding: 18px;
        border: 1px solid #2b343e;
        border-radius: 12px;
        background: #151b22;
        min-height: 125px;
    }

    .label {
        color: #8f9baa;
        font-size: 12px;
        text-transform: uppercase;
        letter-spacing: 0.08em;
    }

    .value {
        margin-top: 10px;
        font-size: 30px;
        font-weight: 700;
    }

    .unit {
        color: #8f9baa;
        font-size: 13px;
        margin-left: 4px;
    }

    .wide {
        grid-column: span 2;
    }

    .event {
        grid-column: span 2;
        min-height: 125px;
    }

    .event-text {
        margin-top: 12px;
        font-family: Consolas, monospace;
        font-size: 15px;
        word-break: break-word;
    }

    .safe {
        color: #35d07f;
    }

    .warning {
        color: #f2c94c;
    }

    .danger {
        color: #ff6b6b;
    }

    .chart {
        margin-top: 18px;
        padding: 18px;
        border: 1px solid #2b343e;
        border-radius: 12px;
        background: #151b22;
    }

    .chart h2 {
        margin: 0 0 12px;
        font-size: 15px;
    }

    canvas {
        width: 100%;
        height: 240px;
        background: #10161d;
        border-radius: 8px;
    }

    .footer {
        margin-top: 15px;
        color: #6f7b87;
        font-size: 11px;
    }

    @media (max-width: 900px) {
        .grid {
            grid-template-columns: repeat(2, 1fr);
        }
    }

    @media (max-width: 550px) {
        .grid {
            grid-template-columns: 1fr;
        }

        .wide,
        .event {
            grid-column: span 1;
        }
    }
</style>
</head>

<body>

<header>
    <h1>QNX Vehicle Gateway — Real-Time Performance Monitor</h1>
    <p>
        Safety-critical CAN scheduling • QNX telemetry •
        deterministic routing demonstration
    </p>
</header>

<div class="container">

    <div class="status">
        <div class="status-left">
            <div class="dot" id="statusDot"></div>
            <strong id="source">Waiting for telemetry...</strong>
        </div>
        <div id="timestamp">--:--:--</div>
    </div>

    <div class="grid">

        <div class="card">
            <div class="label">CAN Bus Load</div>
            <div class="value" id="busLoad">0.0<span class="unit">%</span></div>
        </div>

        <div class="card">
            <div class="label">Frame Throughput</div>
            <div class="value" id="fps">0<span class="unit">FPS</span></div>
        </div>

        <div class="card">
            <div class="label">Safety Latency</div>
            <div class="value" id="latency">0.00<span class="unit">ms</span></div>
        </div>

        <div class="card">
            <div class="label">CPU Utilisation</div>
            <div class="value" id="cpu">0.0<span class="unit">%</span></div>
        </div>

        <div class="card">
            <div class="label">Queue / Buffer</div>
            <div class="value" id="queue">0.0<span class="unit">%</span></div>
        </div>

        <div class="card">
            <div class="label">Dropped Noise Frames</div>
            <div class="value" id="drops">0</div>
        </div>

        <div class="card">
            <div class="label">Safety Drops</div>
            <div class="value" id="safetyDrops">0</div>
        </div>

        <div class="card">
            <div class="label">Safety Frames</div>
            <div class="value" id="safetyFrames">0</div>
        </div>

        <div class="event">
            <div class="label">Latest Gateway Event</div>
            <div class="event-text" id="event">
                WAITING FOR QNX TELEMETRY
            </div>
        </div>

    </div>

    <div class="chart wide">
        <h2>Real-Time Bus Load / Queue Occupancy</h2>
        <canvas id="loadChart" width="1200" height="240"></canvas>
    </div>

    <div class="chart wide">
        <h2>Safety Latency / CPU Utilisation</h2>
        <canvas id="latencyChart" width="1200" height="240"></canvas>
    </div>

    <div class="footer">
        Dashboard history is intentionally bounded to a short rolling window.
        This keeps visualization memory usage predictable and does not replace
        persistent QNX safety logging.
    </div>

</div>

<script>
let historyData = {
    time: [],
    bus_load: [],
    fps: [],
    latency: [],
    cpu: [],
    queue: []
};


function formatNumber(value, digits = 1) {
    return Number(value || 0).toFixed(digits);
}


function setClass(element, value, warning, danger) {
    element.classList.remove("safe", "warning", "danger");

    if (value >= danger) {
        element.classList.add("danger");
    } else if (value >= warning) {
        element.classList.add("warning");
    } else {
        element.classList.add("safe");
    }
}


function updateDashboard(data) {
    document.getElementById("busLoad").innerHTML =
        formatNumber(data.bus_load, 1) + '<span class="unit">%</span>';

    document.getElementById("fps").innerHTML =
        formatNumber(data.fps, 0) + '<span class="unit">FPS</span>';

    document.getElementById("latency").innerHTML =
        formatNumber(data.safety_latency_ms, 2) + '<span class="unit">ms</span>';

    document.getElementById("cpu").innerHTML =
        formatNumber(data.cpu_util, 1) + '<span class="unit">%</span>';

    document.getElementById("queue").innerHTML =
        formatNumber(data.queue_occupancy, 1) + '<span class="unit">%</span>';

    document.getElementById("drops").textContent =
        Math.round(data.dropped_frames || 0);

    document.getElementById("safetyDrops").textContent =
        Math.round(data.safety_drops || 0);

    document.getElementById("safetyFrames").textContent =
        Math.round(data.safety_frames || 0);

    document.getElementById("event").textContent =
        data.event || "NO EVENT";

    document.getElementById("source").textContent =
        "Telemetry source: " + (data.source || "UNKNOWN");

    if (data.timestamp) {
        const date = new Date(data.timestamp * 1000);
        document.getElementById("timestamp").textContent =
            date.toLocaleTimeString();
    }

    // Colour-code the important engineering metrics.
    setClass(document.getElementById("busLoad"),
              data.bus_load, 70, 90);

    setClass(document.getElementById("latency"),
              data.safety_latency_ms, 4, 5);

    setClass(document.getElementById("queue"),
              data.queue_occupancy, 70, 85);

    setClass(document.getElementById("safetyDrops"),
              data.safety_drops, 1, 1);
}


function drawChart(canvasId, seriesA, seriesB, labelA, labelB,
                   maxValue, unit) {

    const canvas = document.getElementById(canvasId);
    const ctx = canvas.getContext("2d");

    const width = canvas.width;
    const height = canvas.height;

    ctx.clearRect(0, 0, width, height);

    // Background.
    ctx.fillStyle = "#10161d";
    ctx.fillRect(0, 0, width, height);

    const margin = 40;
    const plotW = width - 2 * margin;
    const plotH = height - 2 * margin;

    // Grid.
    ctx.strokeStyle = "#27313b";
    ctx.lineWidth = 1;

    for (let i = 0; i <= 4; i++) {
        const y = margin + plotH * i / 4;

        ctx.beginPath();
        ctx.moveTo(margin, y);
        ctx.lineTo(width - margin, y);
        ctx.stroke();
    }

    function drawSeries(values, lineType) {
        if (!values || values.length < 2) {
            return;
        }

        ctx.strokeStyle = lineType;
        ctx.lineWidth = 2;
        ctx.beginPath();

        values.forEach((value, index) => {
            const x = margin +
                (index / Math.max(1, values.length - 1)) * plotW;

            const safeValue = Math.max(0, Math.min(maxValue, Number(value)));

            const y = margin + plotH -
                (safeValue / maxValue) * plotH;

            if (index === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        });

        ctx.stroke();
    }

    // Use different line patterns rather than relying on colour alone.
    drawSeries(seriesA, "#e9eef3");
    drawSeries(seriesB, "#8f9baa");

    ctx.fillStyle = "#8f9baa";
    ctx.font = "12px Arial";

    ctx.fillText("0 " + unit, 8, height - margin);
    ctx.fillText(maxValue + " " + unit, 8, margin + 4);

    ctx.fillStyle = "#e9eef3";
    ctx.fillText("A: " + labelA, margin, 18);

    ctx.fillStyle = "#8f9baa";
    ctx.fillText("B: " + labelB, margin + 160, 18);
}


async function refresh() {

    try {
        const telemetryResponse =
            await fetch("/api/telemetry");

        const data =
            await telemetryResponse.json();

        updateDashboard(data);

        const historyResponse =
            await fetch("/api/history");

        historyData =
            await historyResponse.json();

        drawChart(
            "loadChart",
            historyData.bus_load,
            historyData.queue,
            "Bus Load",
            "Queue Occupancy",
            100,
            "%"
        );

        drawChart(
            "latencyChart",
            historyData.latency,
            historyData.cpu,
            "Safety Latency",
            "CPU Utilisation",
            100,
            "%"
        );

    } catch (error) {
        document.getElementById("source").textContent =
            "Dashboard connection error";
    }
}


setInterval(refresh, 1000);
refresh();

</script>

</body>
</html>
"""


@app.route("/")
def dashboard():
    return render_template_string(HTML)


# ============================================================================
# MAIN
# ============================================================================

def main() -> None:
    print("=" * 72)
    print(" QNX VEHICLE GATEWAY — PERFORMANCE DASHBOARD")
    print("=" * 72)
    print(f"[WEB]  http://127.0.0.1:{WEB_PORT}")
    print(f"[UDP]  Listening on {UDP_HOST}:{UDP_PORT}")
    print(f"[DEMO] {'ENABLED' if DEMO_MODE else 'DISABLED'}")
    print("=" * 72)

    # UDP reception is independent from the Flask web server.
    receiver = threading.Thread(
        target=udp_receiver,
        name="udp_receiver",
        daemon=True,
    )
    receiver.start()

    if DEMO_MODE:
        demo = threading.Thread(
            target=demo_generator,
            name="demo_generator",
            daemon=True,
        )
        demo.start()

    # threaded=True allows the browser request and telemetry APIs
    # to be served independently.
    app.run(
        host=WEB_HOST,
        port=WEB_PORT,
        debug=False,
        threaded=True,
    )


if __name__ == "__main__":
    main()
