# Multi Agent Traffic Simulation with Adaptive Traffic Lights

This project is a multi agent traffic simulation built with [SPADE](https://github.com/javipalanca/spade).  
It models urban traffic on a grid network with:

- traffic light agents that control intersections
- car agents that choose routes based on travel cost and congestion
- priority agents that represent emergency vehicles and influence signal timing

The main goal is to study how different traffic light control policies affect traffic flow, travel times and fairness between roads.

---

## Features

- Grid based road network generator with nodes, roads and lanes (only the option 2 lanes is available, connections were hard coded)
- One agent per traffic light, one per vehicle
- Route planning with K shortest paths on the lane graph
- Congestion aware routing, cars balance distance and road occupancy
- Support for priority vehicles, for example ambulances, that can request green waves
- Pluggable traffic light control policies, for example:
  - cyclic policy 1 2 3 4 1 2 3 4
  - demand based policy using proposals between traffic lights
- Experiment scripts to run controlled scenarios and collect metrics

---

## Project structure

The core modules are:

```text
network.py              Road network model (with functions of pathfinding that aren't used)
trafficLight_agent.py   Traffic light agent logic and signal control
car_agent.py            Car agent behaviour and routing
priority_agent.py       Priority vehicle behaviour and interaction with lights
simulation.py           Simple infinite simulation runner (Ctrl + C to stop simulation)
experimentation.py      Experiment runner with configurable scenarios
incidentReported.py     Stub for future incident reporting agent
````

Short description of each.

### `network.py`

* Builds a grid network of intersections and roads
* Models:

  * `Node`  intersections
  * `Road`  directed roads between nodes
  * `Lane`  individual lanes inside each road
* Each lane knows:

  * its parent road
  * which lanes feed into it
  * which lanes it can lead to
* Includes pathfinding:

  * Dijkstra on the lane graph
  * Yen K shortest paths for alternative routes

### `trafficLight_agent.py`

Traffic light agent for a single incoming road at an intersection.

Responsibilities:

* keep the state of the signal: red, yellow, green
* track how many cars are in each lane
* decide how long to stay green between a minimum and a maximum, based on road occupancy
* choose which traffic light at the same intersection gets the next green, depending on the selected control policy
* coordinate with priority vehicles

Key behaviours:

* `ReceiveMessageBehaviour`

  * handles all messages from cars, priority vehicles and other traffic lights
  * answers:

    * `get-signal` with the current signal state
    * `get-lane-capacity` with current and total capacity
    * `get-routes` with K shortest routes to a destination node
  * processes:

    * notifications when cars enter or leave lanes
    * messages about priority vehicles entering or leaving the road
    * messages to yield green for priority

* `SignalTimeBehaviour`

  * runs one full signal cycle for a traffic light
  * sets the signal to green
  * computes how long to stay green between `green_light_min_time` and `green_light_max_time` based on
    `sum(lane_capacities) / (n_lanes * road_capacity)`
  * extends green while a priority vehicle is present
  * switches to yellow and red and waits a clear time
  * chooses the next traffic light for green according to `selection_method`

Traffic light control policies are selected through:

```python
selection_method="cyclic"     # cyclic order 1 2 3 4 1 2 3 4
selection_method="proposals"  # original proposal based method
```

You can add more methods if needed.

### `car_agent.py`

Car agent that:

* spawns at a given starting road and aims at a destination node
* requests lane capacity and routes from the traffic light on its starting road
* maintains a list of possible routes
* chooses a route that minimises a weighted combination of:

  * travel cost (distance or time)
  * congestion cost (lane occupancy along the route)
* moves periodically, asking the current traffic light for the signal state
* respects red and yellow lights and lane capacities
* notifies traffic lights when entering or leaving a lane

This agent is used to simulate normal traffic.

### `priority_agent.py`

Priority vehicle agent, for example an ambulance.

* Similar to `CarAgent` but with extra powers:

  * it can notify upcoming roads that a priority vehicle is approaching
  * traffic lights mark `priority_incoming` and can extend or force green
  * cars and traffic lights can react to open a path
* Reports when it leaves a road so traffic lights know the priority situation has ended

This is used to test priority handling and emergency strategies.

### `simulation.py`

Single run simulation script. Typical responsibilities:

* create a `Network` with a grid topology
* create one `TrafficLightAgent` per road
* group traffic lights per intersection
* start an initial green cycle at one traffic light in each intersection
* periodically spawn car agents with:

  * random origin from spawn nodes
  * random destination node
* run until interrupted

Good entry point for quick tests and visual logging.

### `experimentation.py`

Script for controlled experiments.

* defines sets of parameters for:

  * network size
  * number of lanes and capacities
  * car spawn rate and weights for routing
  * priority vehicle patterns
  * traffic light timing
  * traffic light control policy
* runs multiple independent simulations, collects metrics like:

  * average travel time
  * total number of completed trips
  * average green time per traffic light
* can be extended to log data to file for later analysis

### `incidentReported.py`

Placeholder for an `IncidentReporterAgent`.

The idea is to later support:

* dynamic incidents
* road closures or capacity reductions
* notifications to traffic lights and vehicles
* dynamic re routing

At the moment this file is a stub, ready to be implemented.

---

## Requirements

* Python 3.10 or later
* SPADE
* A running XMPP server (for example on `localhost`)
* Other Python dependencies

Install dependencies with:

```bash
pip install -r requirements.txt
```

Check `requirements.txt` for the exact list of libraries and versions.

You also need to configure SPADE to connect to your XMPP server.
By default the code uses JIDs like:

* `tl{road_id}@localhost` for traffic lights
* `car{counter}@localhost` for cars
* `priority{counter}@localhost` for priority vehicles

If your XMPP server is not `localhost`, you must adjust the JIDs and SPADE settings.

---

## Installation

Clone the repository and install dependencies.

```bash
git clone <your_repo_url>.git
cd <your_repo_name>

python -m venv .venv
source .venv/bin/activate   # on Windows use: .venv\Scripts\activate

pip install -r requirements.txt
```

Make sure your XMPP server is up and reachable before starting the simulation.

---

## Configuration

Most configuration lives at the top of `simulation.py` and `experimentation.py`.

Common parameters:

### Network

* `GRID` number of nodes per side, creates GRID x GRID intersections
* `N_LANES` number of lanes per directed road (must be 2)
* `ROAD_CAPACITY` capacity units per lane, used in timing and occupancy

### Traffic lights

* `GREEN_LIGHT_MIN_TIME`
* `GREEN_LIGHT_MAX_TIME`
* `YELLOW_LIGHT_TIME`
* `CLEAR_INTERSECTION_TIME`

These times are given in seconds and control the signal cycle.
The actual green time is computed dynamically between min and max based on how full the road is.

### Cars

* `SPAWN_TIMER` time between new car spawns
* `TRAVEL_WEIGHT` weight for travel cost in route selection
* `CAPACITY_WEIGHT` weight for congestion cost in route selection
* `MOVEMENT_TIMER` period between movement updates in `CarAgent`

### Priority vehicles

If enabled in the scripts:

* spawn frequency of priority agents
* rules for their origin and destination
* behaviour when entering roads and intersections

---

## Running the simulation

From the project root, after activating the virtual environment and starting your XMPP server "spade run":

### Simple simulation

```bash
python simulation.py
```

This will:

* build a network
* start all traffic light agents
* start an initial green cycle at each intersection
* spawn cars over time

You should see logs in the console about:

* traffic lights starting and changing signal
* cars spawning, moving and completing trips
* any priority events if configured

### Experiments

```bash
python experimentation.py
```

This runs controlled experiments.
Check the script to see:

* how events are generated
* how many runs are performed
* where results are stored or printed

You can change parameters there to compare different traffic light control methods.

---

## Traffic light control policies

Traffic light behaviour is controlled by the `selection_method` parameter of `TrafficLightAgent`.

Currently implemented:

### 1. Cyclic policy

```python
selection_method="cyclic"
```

* At each intersection, all incoming roads have one traffic light agent
* Traffic lights at a given intersection are ordered in a fixed sequence, for example:

  * TL for road 1
  * TL for road 2
  * TL for road 3
  * TL for road 4
* After each full green yellow red cycle, the current traffic light:

  * computes the next one in the sequence
  * hands control over by sending a `start-cycle` message
* The green time within each cycle is still dynamic:

  * it varies between minimum and maximum based on how many cars are on the road

This is good for baseline experiments, since the order is fixed and predictable.

### 2. Proposal based policy

```python
selection_method="proposals"
```

* The traffic light that just finished a cycle broadcasts a `asking-for-proposal` message to the other traffic lights at the same intersection
* Each neighbour replies with a proposal that includes:

  * its current tolerance value
  * how many cars it has on the road
* The coordinator uses `choose_best_proposal` to select the winner, usually by choosing:

  * the highest tolerance
  * or another scoring function if you customise it
* The winner receives an `accept-proposal` message and starts its own `SignalTimeBehaviour`

This policy is more demand driven and can be extended with additional metrics.

---

## Extending the project

Some ideas to extend and experiment.

### New traffic light policies

You can add new control methods by:

* adding a new value for `selection_method`
* implementing another branch in `SignalTimeBehaviour` to decide the next controller
* optionally extending `choose_best_proposal` to use more metrics

Examples:

* policies that minimise average delay at the intersection
* fairness based policies that limit maximum waiting time per road
* reinforcement learning based controllers

### Incident handling (not implemented yet)

`incidentReported.py` is a natural place to:

* report accidents or road works
* reduce capacity of specific roads or lanes
* inform traffic lights and vehicles to re route around blocked roads

### Data collection

You can extend `experimentation.py` and the agent code to log:

* per vehicle:

  * start time, end time, total travel time, path taken
* per traffic light:

  * list of green times
  * number of cars served per phase

Then export to CSV or JSON and analyse with tools like pandas or notebooks.

---

## Reproducibility

To make experiments reproducible you should:

* fix a random seed at the beginning of your scripts
* generate and store a list of events:

  * spawn times
  * origin and destination pairs
* run different policies on the same event list

This makes it possible to compare policies fairly under identical traffic demand.