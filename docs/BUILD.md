# Build and Run

## QNX gateway

The historical sources contain their own build comments. Typical examples are:

```sh
qcc -Wall -Wextra -O2 QNX_GATEWAY_MASTER_CORRECTED.c -o qnx_gateway -lsocket
```

or, for the Momentics master:

```sh
qcc -Wall -Wextra -O2 qnx_master_momentics.c -o qnx_master -lsocket -lm
```

Use the build command that belongs to the exact version being tested. Do not assume all versions have identical dependencies.

## Dashboard

The dashboard uses Python + Flask:

```sh
python -m pip install flask
python qnx_gateway_dashboard.py
```

It listens for UDP telemetry on port 8080 and serves the dashboard on port 5000. The dashboard includes a DEMO_MODE; demo values are not experimental evidence. fileciteturn11file0L48-L80

## Important

The repository does not include the QNX SDP, Momentics, NXP MCUXpresso SDK or vendor BSPs. Install the appropriate vendor toolchains separately.
