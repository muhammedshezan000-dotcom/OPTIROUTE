# OPTIROUTE

OPTIROUTE is an intelligent warehouse management and autonomous navigation
simulation for the Classes 9–11 STEM project. Its control center includes
warehouse editing, Dijkstra/A*/BFS comparison, weighted routing, animated
digital-twin movement, delivery orders with optimized multiple stops, obstacle
rerouting, historical congestion estimates, analytics, route reports, and
controller diagnostics. The Warehouse Simulation workspace runs discrete
one-junction ticks, injects obstacles, and replans around them with A*.

Navigation and movement run in simulation. System Diagnostics can open a USB
serial connection to request `STATUS`, `STOP`, and raw `SENSORS` readings. The
firmware and Pi serial client reject movement commands.

## Run it

Python 3.10 or newer is recommended.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python main.py
```

Open the local URL printed by Streamlit. Use the sidebar to switch among
Overview, Warehouse Map, Route Optimisation, Warehouse Simulation, Robot
Control, Delivery Management, Analytics, and System Diagnostics. In Warehouse
Simulation, select endpoints and speed, optionally set a random obstacle rate,
and start a run. Pause, resume, inject an obstacle, emergency-stop, and reset
the simulated robot from the workspace. Simulation events, reroutes, distance,
and estimated travel time are shown during the run. The robot animates smoothly
along each corridor, including intermediate steps at higher simulation speeds.
Route history, delivery
orders, and simulated corridor traversals are saved in a local SQLite
database at `data/optiroute.db`.

Run the mathematical engine tests without installing the dashboard dependency:

```bash
python -m unittest discover -s tests -v
```

## Warehouse model

The warehouse has 25 nodes numbered row-major from 0 to 24. Node `n` is at
`(row, column) = (n // 5, n % 5)`. Each node connects to its north, south, east,
and west neighbours where those neighbours exist. The default start is node 0
and the default destination is node 24.

The route objective is the weighted sum of normalized distance, travel time,
and congestion. The sidebar weights are normalized to sum to 1. A blocked
corridor is removed in both directions. Restricted nodes cannot be used as
transit locations. The map editor supports pickup/delivery nodes, zones,
corridor distance, speed limits, manual congestion, and blockages. A* uses
Manhattan distance multiplied by a lower bound on per-step route cost, so it
remains admissible for these costs. BFS minimizes corridor count and ignores
weights. Exact multiple-delivery stop ordering uses dynamic programming and
supports up to eight stops.

The congestion estimate is a relative-frequency statistical baseline based on
observed simulated traversals; it is not machine learning or a forecast of real
future traffic. The digital twin advances one junction per second when its
animation is enabled. The dedicated Warehouse Simulation can advance faster
and optionally injects deterministic per-tick obstacles to demonstrate
automatic rerouting. It is a software simulation only and does not issue motor
commands. Route history, orders, and traversals persist locally; robot position
and the active simulation mission are session state.
If the app restarts during a delivery, that order is marked interrupted and
must be requeued after checking the robot position.

## Student work split

- **Class 9:** Build the physical grid and robot; ensure the physical
  intersections match the simulator's row/column numbering.
- **Class 10:** Study the graph generator, Dijkstra, A*, and obstacle tests.
- **Class 11:** Experiment with cost weights and congestion; record route
  length, estimated time, turns, and algorithm search statistics.

## Hardware integration boundary

The `robot/` folder contains a serial communication helper and route-to-command
converter. The System Diagnostics page can inspect an Arduino over USB serial
and send `STATUS`, `STOP`, or `SENSORS`. The latter reports raw analog values
from A0–A4 and an HC-SR04 range reading from D12/D11; `NO_ECHO` means no echo
was received before timeout. These pin assignments are provisional and
readings are unverified until the sensors are connected and tested. The
Arduino sketch and Pi serial client intentionally reject movement commands.

The robot has four motors, but their exact voltage and current ratings have not
been identified. The L298N module's capacity and the battery's actual
specifications also need verification; two motors per driver channel may exceed
its current rating. Do not connect motor power or enable motion yet. A physical
motor-power emergency stop and calibration are required before physical
movement. The digital twin remains simulation only; physical position is not
yet synchronized to it.

Before enabling motion, verify motor voltage/current ratings, use a regulated
supply for the Raspberry Pi, keep a physical emergency stop, and test motor
direction with the wheels lifted. Never connect motors directly to
microcontroller GPIO pins or motor voltage to Raspberry Pi GPIO.

Multi-robot coordination and machine-learning congestion prediction are
intentional future extensions; the current congestion estimate is statistical
and based on observed routes.

