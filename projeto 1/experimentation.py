"""
Batch simulation runner for the multi-agent traffic system.

This script:
  - Builds a fixed-size road network.
  - Spawns one TrafficLightAgent per road (using the "proposals" selection method).
  - Pre-generates a deterministic list of car spawn events.
  - Runs multiple simulations varying (travel_weight, capacity_weight) pairs.
  - Collects metrics from each car and global statistics.
  - Saves all metrics as JSON files inside the `simulation_results` folder.
"""
# import sys

# Redirect all print() output to a file
# sys.stdout = open("simulation_output.txt", "w", encoding="utf-8")
# sys.stderr = open("simulation_errors.txt", "w", encoding="utf-8")

import asyncio
import random
import json
import os
import time
from spade import run
import threading

from network import Network, Node, Road, Lane
from car_agent import CarAgent
from trafficLight_agent import TrafficLightAgent
from priority_agent import PriorityAgent
import interface

# ----------------------
# Network parameters
# ----------------------
WITH_INTERFACE = True
GRID = 3
N_LANES = 2
ROAD_CAPACITY = 25
INTERFACE_ROAD_LENGTH = 200

# ----------------------
# Car parameters
# ----------------------
SPAWN_TIMER = 0.025
MOVEMENT_TIMER = 0.1
NUM_CARS = 200

# ----------------------
# Traffic light parameters
# ----------------------
GREEN_LIGHT_MIN_TIME = 1
GREEN_LIGHT_MAX_TIME = 5
YELLOW_LIGHT_TIME = 0.3
CLEAR_INTERSECTION_TIME = 0.3

# ----------------------
# Deterministic behaviour
# ----------------------
RANDOM_SEED = 42

# ----------------------
# JSON results folder
# ----------------------
RESULTS_FOLDER = "simulation_results"
os.makedirs(RESULTS_FOLDER, exist_ok=True)

# ----------------------
# Deterministic spawn events
# ----------------------
def generate_spawn_events(network, num_events):
    """
    Generate a deterministic list of car spawn events.

    Each event contains:
        (start_node_id, start_road_id, end_node_id)

    The randomness is controlled using RANDOM_SEED so that the spawn pattern
    is reproducible across runs.

    Parameters
    ----------
    network : Network
        Network instance from which spawn nodes are taken.
    num_events : int
        Number of spawn events (cars) to generate.

    Returns
    -------
    list[tuple[int, int, int]]
        List of (start_node_id, start_road_id, end_node_id).
    """
    random.seed(RANDOM_SEED)
    events = []
    nodes = list(network.spawn_nodes)
    for _ in range(num_events):
        temp = list(nodes)
        start_node = random.choice(temp)
        start_road_id = start_node.outRoads[0].ID
        start_lane_id = random.choice([0,1])
        temp.remove(start_node)
        end_node_id = random.choice(temp).ID
        events.append((start_node.ID ,start_road_id, start_lane_id, end_node_id))
    return events


# ----------------------
# Helper function to stop agents
# ----------------------
async def stop_agents(agents):
    """
    Gracefully stop all SPADE agents in the given iterable.

    Parameters
    ----------
    agents : iterable
        Iterable of SPADE agent instances whose `stop()` coroutine
        will be awaited.
    """
    for agent in agents:
        if agent.finished_callback:
            agent.finished_callback(agent.metrics, agent)

        await agent.stop()

# ----------------------
# Weights simulation
# ----------------------
async def weights_simulation(travel_weight, capacity_weight, type_of_tl, changed_network, adaptive_green):
    """
    Run a single simulation with the given routing weights.

    The simulation:
      - Creates a network and starts all traffic lights.
      - Pre-generates a set of car spawn events using `generate_spawn_events`.
      - Spawns NUM_CARS CarAgent instances with the given weights.
      - Waits until all active cars finish.
      - Stops all traffic light agents.
      - Collects and saves metrics for every car and the total simulation time.

    Parameters
    ----------
    travel_weight : float
        Weight given to travel time in the agents' route choice.
    capacity_weight : float
        Weight given to capacity in the agents' route choice.

    Returns
    -------
    dict
        Dictionary of collected metrics, including per-car metrics and
        "total_simulation_duration".
    """
    #print(f"\n[SIM] Starting simulation with TRAVEL_WEIGHT={travel_weight}, CAPACITY_WEIGHT={capacity_weight}\n")

    collected_metrics = {}  # <---- store metrics here

    # Create network
    network = Network()
    network.create_network(GRID, N_LANES, INTERFACE_ROAD_LENGTH, ROAD_CAPACITY)

    if changed_network:
        network.roads[1].length = 3*network.roads[1].length
        network.roads[11].length = 3*network.roads[11].length
        network.roads[21].length = 3*network.roads[21].length

    def tl_finished_callback(agent_metrics, tl_agent):
        """
        Callback executed when a tl finishes its job.

        Parameters
        ----------
        metrics : dict
            Metrics collected by the tl agent.
        agent
            Reference to the agent that has finished.
        """
        # remove and store metrics
        collected_metrics[str(tl_agent.jid)] = agent_metrics

    def car_finished_callback(metrics, agent):
        """
        Callback executed when a car finishes its route.

        Parameters
        ----------
        metrics : dict
            Metrics collected by the car agent.
        agent
            Reference to the agent that has finished.
        """
        # remove and store metrics
        collected_metrics[str(agent.jid)] = metrics

    # Start traffic lights
    traffic_lights = {}
    traffic_light_groups = {}

    for road_id, road in network.roads.items():
        tl_jid = f"tl{road_id}@localhost"

        if adaptive_green:
            tl_agent = TrafficLightAgent(
                jid=tl_jid,
                password="tl",
                road_id=road_id,
                network=network,
                green_light_min_time=GREEN_LIGHT_MIN_TIME,
                green_light_max_time=GREEN_LIGHT_MAX_TIME,
                yellow_light_time=YELLOW_LIGHT_TIME,
                clear_intersection_time=CLEAR_INTERSECTION_TIME,
                selection_method=type_of_tl,    # proposals or cyclic
            )
        else:
            tl_agent = TrafficLightAgent(
                jid=tl_jid,
                password="tl",
                road_id=road_id,
                network=network,
                green_light_min_time=3,
                green_light_max_time=3,
                yellow_light_time=YELLOW_LIGHT_TIME,
                clear_intersection_time=CLEAR_INTERSECTION_TIME,
                selection_method=type_of_tl,    # proposals or cyclic
            )

        tl_agent.finished_callback = tl_finished_callback

        await tl_agent.start()
        traffic_lights[road_id] = tl_agent
        traffic_light_groups.setdefault(road.endNode.ID, []).append(tl_agent)

        print(f"[SIM] Started TrafficLightAgent for road {road_id}")

    # For each intersection, start the first (lowest road_id) TL's cycle
    for _, tl_group in traffic_light_groups.items():
        tl_group_sorted = sorted(tl_group, key=lambda tl: tl.road_id)
        first_tl = tl_group_sorted[0]

        behaviour = first_tl.SignalTimeBehaviour()
        first_tl.signal_time_behaviour = behaviour
        first_tl.add_behaviour(behaviour)

    # In case changed_network == True
    # if changed_network:
    #     closed_roads = [26,25,27,28,29,30,31,32,33,34,39,40,41,42,44,43,48,47,45,46,3,14,22,12,2]
    #     network.closed_roads = closed_roads

    #     for tl in traffic_lights.values():
    #         tl.closed_roads = closed_roads
    #     for road in network.roads.values():
    #         if road.ID in closed_roads:
    #             road.closed = True
    #     traffic_lights[35].priority_incoming = True
    #     traffic_lights[4].priority_incoming = True
    #     traffic_lights[13].priority_incoming = True

    closed_roads = [26,25,27,28,31,32,33,34,39,40,41,42,48,47,45,46,3,14,22,12,2]
    network.closed_roads = closed_roads

    for tl in traffic_lights.values():
        tl.closed_roads = closed_roads
    for road in network.roads.values():
        if road.ID in closed_roads:
            road.closed = True
    traffic_lights[35].priority_incoming = True
    traffic_lights[4].priority_incoming = True
    traffic_lights[13].priority_incoming = True

    # Pre-generate cars
    # spawn_events = generate_spawn_events(network, NUM_CARS)
    active_cars = set()

    def car_remove_callback(agent):
        """
        Callback executed if a car is removed.

        Parameters
        ----------
        agent
            Reference to the agent being removed.
        """
        active_cars.discard(agent)

    # Start visualization (Pygame) in a separate daemon thread
    interface_stop_event = threading.Event()

    if WITH_INTERFACE:
        threading.Thread(
            target=interface.game_loop, 
            args=(network, traffic_lights, active_cars, interface_stop_event),
            daemon=True  # encerra automaticamente ao fechar o programa
        ).start()

    #print("\n[SIM] Spawning cars...\n")

    start_time = time.time()
    counter = 1

    # Spawn cars
    # for i, (start_node_id, start_road_id, start_lane_id, end_node_id) in enumerate(spawn_events):
        # temp = list(network.spawn_nodes)
        # starting_node = random.choice(temp).ID
        # starting_road = network.nodes[starting_node].outRoads[0].ID
        # starting_lane = 0
        # temp.remove(network.nodes[starting_node])
        # end_node = random.choice(temp).ID

    def choose_lane(starting_road, end_node):
        routes_combined = network.generate_routes(starting_road, 0, end_node, 20)
        routes_combined.extend(network.generate_routes(starting_road, 1, end_node, 20))
        routes_combined = list(routes_combined)

        for i, route in enumerate(routes_combined):
            route = list(route)
            temp_capacity = 0
            path = route[0]
            for road_lane in path:
                temp_capacity += traffic_lights[road_lane[0]].lane_capacities[road_lane[1]]

            route.append(temp_capacity)
            routes_combined[i] = route   # ← IMPORTANT FIX

        for route in routes_combined:
            travel_cost = route[1]
            capacity_cost = route[2]
            total_cost = (travel_weight*travel_cost) + (capacity_weight*capacity_cost)
            route.append(total_cost)

        # Sort routes by total cost
        routes_combined.sort(key=lambda route: route[3])

        # Choose the first route whose first lane has free capacity
        for route in routes_combined:
            path = route[0]
            first_road_lane = path[0]
            first_lane_current_capacity =  traffic_lights[first_road_lane[0]].lane_capacities[first_road_lane[1]]
            first_lane_total_capacity =  traffic_lights[first_road_lane[0]].road_capacity
            if first_lane_current_capacity < first_lane_total_capacity:
                if first_road_lane[0] == 4:
                    return 1
                else:
                    return 0

    while counter <= NUM_CARS:
        starting_node = 15
        starting_road = network.nodes[starting_node].outRoads[0].ID
        end_node = 16
        starting_lane = choose_lane(starting_road, end_node)

        while traffic_lights[starting_road].lane_capacities[starting_lane] == 25:
            await asyncio.sleep(0.1)

        car_jid = f"car{counter}@localhost"

        #print(f"[SIM] Spawning {car_jid} at road {starting_road} → node {end_node}")

        if counter % 6 == 0:
            car_agent = CarAgent(
                jid=car_jid,
                password="car",
                start_node=12,
                starting_road=29,
                starting_lane=0,
                end_node=19,
                travel_weight=travel_weight,
                capacity_weight=capacity_weight,
                periodic_movement=MOVEMENT_TIMER,
                remove_callback=car_remove_callback
            )
        elif counter % 7 == 0:
            car_agent = CarAgent(
                jid=car_jid,
                password="car",
                start_node=19,
                starting_road=43,
                starting_lane=0,
                end_node=12,
                travel_weight=travel_weight,
                capacity_weight=capacity_weight,
                periodic_movement=MOVEMENT_TIMER,
                remove_callback=car_remove_callback
            )
        else:
            car_agent = CarAgent(
                jid=car_jid,
                password="car",
                start_node=starting_node,
                starting_road=starting_road,
                starting_lane=starting_lane,
                end_node=end_node,
                travel_weight=travel_weight,
                capacity_weight=capacity_weight,
                periodic_movement=MOVEMENT_TIMER,
                remove_callback=car_remove_callback
            )
        
        car_agent.finished_callback = car_finished_callback

        active_cars.add(car_agent)

        await car_agent.start()

        counter += 1
        
        await asyncio.sleep(SPAWN_TIMER)

    # Wait for all cars to finish
    while active_cars:
        await asyncio.sleep(5)

    # Stop traffic lights
    await stop_agents(traffic_lights.values())

    # Allow some time for agents to terminate
    await asyncio.sleep(5)

    simulation_duration = time.time() - start_time
    collected_metrics["total_simulation_duration"] = simulation_duration

    # -----------------------------
    # SAVE METRICS TO JSON
    # -----------------------------
    if changed_network:
        filename = f"sim_travel{travel_weight}_capacity{capacity_weight}_{type_of_tl}_adaptiveGreen{adaptive_green}_changedNetwork.json"
    else:
        filename = f"sim_travel{travel_weight}_capacity{capacity_weight}_{type_of_tl}_adaptiveGreen{adaptive_green}_notChangedNetwork.json"

    filepath = os.path.join(RESULTS_FOLDER, filename)

    with open(filepath, "w") as f:
        json.dump(collected_metrics, f, indent=4, sort_keys=True)

    print(f"[SIM] Metrics saved: {filepath}")

    if WITH_INTERFACE:
        interface_stop_event.set()

    return collected_metrics

# ----------------------
# Run test
# ----------------------
async def weights_test(travel_weight=0.5, capacity_weight=0.5, type_of_tl="cyclic", changed_network=False, adaptive_green=False):
    metrics = await weights_simulation(travel_weight=travel_weight, capacity_weight=capacity_weight, type_of_tl=type_of_tl, changed_network=changed_network, adaptive_green=adaptive_green)

# ----------------------
# Entry point
# ----------------------
if __name__ == "__main__":
    run(weights_test(travel_weight=0.9, capacity_weight=0.1, type_of_tl="proposals", changed_network=True, adaptive_green=False))