"""
Main simulation script for the multi-agent traffic management system.

This module:
  - Builds the road network.
  - Creates and starts all TrafficLightAgent instances.
  - Spawns CarAgent and PriorityAgent agents at regular intervals.
  - Launches the Pygame visualization interface in a separate thread.
"""
import sys

# Redirect all print() output to a file
sys.stdout = open("simulation_output.txt", "w", encoding="utf-8")
sys.stderr = open("simulation_errors.txt", "w", encoding="utf-8")

import asyncio
import random
import traceback
from spade import run

from network import Network, Node, Road, Lane
from car_agent import CarAgent
from trafficLight_agent import TrafficLightAgent
from priority_agent import PriorityAgent
from incident_reporter_agent import IncidentReporterAgent

import interface
import threading
import time

# Network
WITH_INTERFACE = True
GRID = 3
N_LANES = 2
ROAD_CAPACITY = 25
INTERFACE_ROAD_LENGTH = 200

# Road closure system
ROAD_CLOSE_FREQUENCY = 5
ROAD_CLOSE_PROBABILITY = 0.2
ROAD_CLOSED_DURATION = 50

# Car constant attributes
SPAWN_TIMER = 0.05
MOVEMENT_TIMER = 0.1
TRAVEL_WEIGHT = 0.5
CAPACITY_WEIGHT = 0.5

# Traffic light constant attributes
GREEN_LIGHT_MIN_TIME = 1
GREEN_LIGHT_MAX_TIME = 4
YELLOW_LIGHT_TIME = 0.3
CLEAR_INTERSECTION_TIME = 0.3

# Car vs Priority spawn probabilities
SPAWN_PROBABILITIES=[0.8, 0.2]

# ----------------------
# Helper function to stop agents
# ----------------------
async def stop_agents(agents):
    """
    Gracefully stop all given SPADE agents.

    Parameters
    ----------
    agents : iterable
        Iterable of agent instances whose `stop()` coroutine will be awaited.
    """
    for agent in agents:
        await agent.stop()


async def simulation():
    """
    Main simulation coroutine.

    Responsibilities
    ----------------
    - Create the road network.
    - Start one TrafficLightAgent per road and group them by intersection.
    - Select an initial green light for each intersection.
    - Continuously spawn CarAgent and PriorityAgent instances.
    - Launch the visualization interface in a separate thread.
    """
    # -------------------------------------------------------
    # 1. Load your network
    # -------------------------------------------------------
    network = Network()
    network.create_network(GRID, N_LANES, INTERFACE_ROAD_LENGTH, ROAD_CAPACITY)

    # NEW
    current_closed_road = None 
    closed_road_timer = 0

    # -------------------------------------------------------
    # 2. Start all traffic lights
    # -------------------------------------------------------
    traffic_lights = {}
    traffic_light_groups = {}
    for road_id, road in network.roads.items():
        tl_jid = f"tl{road_id}@localhost"
        tl_pwd = "tl"

        tl_agent = TrafficLightAgent(
            jid=tl_jid,
            password=tl_pwd,
            road_id=road_id,
            network=network,
            green_light_min_time=GREEN_LIGHT_MIN_TIME,
            green_light_max_time=GREEN_LIGHT_MAX_TIME,
            yellow_light_time=YELLOW_LIGHT_TIME,
            clear_intersection_time=CLEAR_INTERSECTION_TIME,
            selection_method="proposals",    # proposals or cyclic
        )

        await tl_agent.start()
        traffic_lights[road_id] = tl_agent
        traffic_light_groups.setdefault(road.endNode.ID, []).append(tl_agent)

        print(f"[SIM] Started TrafficLightAgent for road {road_id}")

    # ------------------------------------------------------------------------------------------
    # 2. Choose a traffic light for each intersection to have green light from start
    # ------------------------------------------------------------------------------------------
    for intersection, tl_group in traffic_light_groups.items():
        chosen_tl = random.choice(tl_group)
        
        chosen_tl.signal_time_behaviour = chosen_tl.SignalTimeBehaviour()
        chosen_tl.add_behaviour(chosen_tl.signal_time_behaviour)

    # ------------------------------------------------------------------------------------------
    # 3. Create Incident Reporter agent
    # ------------------------------------------------------------------------------------------
    incident_reporter_agent = IncidentReporterAgent(
                            jid="incidend_reporter@localhost",
                            password="incidendreporter",
                            road_close_frequency=ROAD_CLOSE_FREQUENCY,
                            road_close_probability=ROAD_CLOSE_PROBABILITY,
                            road_closed_duration=ROAD_CLOSED_DURATION,
                            network=network
                        )

    await incident_reporter_agent.start()

    # ----------------------------
    # 4. Spawn cars
    # ----------------------------
    car_counter = 0
    priority_counter = 0

    print(f"\n[SIM] Starting main loop… Cars spawn every {SPAWN_TIMER} seconds.\n")
    active_vehicles = set()
    
    interface_stop_event = threading.Event()

    # Launch the Pygame interface in a separate thread
    if WITH_INTERFACE:
        threading.Thread(
            target=interface.game_loop, 
            args=(network, traffic_lights, active_vehicles, interface_stop_event),
            daemon=True  # encerra automaticamente ao fechar o programa
        ).start()

    print("[SIM] Interface iniciada em thread separada.")

    collected_metrics = {}

    while True:
        #--------------------------------------
        # ROAD CLOSING / OPENING SYSTEM (NEW)
        #--------------------------------------
        # if current_closed_road is None:
        #     if random.random() < ROAD_CLOSE_PROBABILITY:
        #         candidate_roads = [
        #             r for r in network.roads.values()
        #             if not r.closed and not (r.startNode.type == 'spawn' or r.endNode.type == "spawn")
        #         ]
        #         if candidate_roads:
        #             road_to_close = random.choice(candidate_roads)
        #             Road.close_road(road_to_close)
        #             current_closed_road = road_to_close
        #             closed_road_timer = ROAD_CLOSED_DURATION
        # else:
        #     closed_road_timer -= 1
        #     if closed_road_timer <= 0 or random.random() < ROAD_REOPEN_PROBABILITY:
        #         Road.open_road(current_closed_road)
        #         current_closed_road = None

        #Randomly choose a starting node and a different end node
        temp = list(network.spawn_nodes)
        starting_node = random.choice(temp)
        starting_road = starting_node.outRoads[0].ID
        starting_lane = random.choice([0,1])
        temp.remove(starting_node)
        end_node = random.choice(temp).ID

        def finished_callback(metrics, agent):
            """
            Callback executed when a vehicle finishes its route.

            Parameters
            ----------
            metrics : dict
                Dictionary of metrics collected by the agent.
            agent
                Reference to the agent that has finished.
            """
            #print(f"[SIM] {agent.jid} finished")
            collected_metrics[str(agent.jid)] = metrics

        def remove_callback(agent):
            """
            Callback executed when a vehicle is removed before starting movement.

            Parameters
            ----------
            agent
                Reference to the agent being removed.
            """
            #print(f"[SIM] {agent.jid} despawned before movement")
            active_vehicles.discard(agent)

        # Decide whether to spawn a normal car or a priority vehicle
        vehicle_options = ["car", "priority"]
        probabilities = SPAWN_PROBABILITIES

        choice = random.choices(vehicle_options, weights=probabilities, k=1)[0]

        if choice == "car":
            car_jid = f"car{car_counter}@localhost"
            car_pwd = "car"

            print(f"[SIM] Spawning {car_jid} at road {starting_road} → node {end_node}")

            car_agent = CarAgent(
                jid=car_jid,
                password=car_pwd,
                start_node=starting_node.ID,
                starting_road=starting_road,
                starting_lane=starting_lane,
                end_node=end_node,
                travel_weight=TRAVEL_WEIGHT,
                capacity_weight=CAPACITY_WEIGHT,
                periodic_movement=MOVEMENT_TIMER,
                remove_callback=remove_callback
            )

            car_agent.finished_callback = finished_callback

            active_vehicles.add(car_agent)

            await car_agent.start()

            car_counter += 1

        elif choice == "priority":
            priority_jid = f"priority{priority_counter}@localhost"
            priority_pwd = "priority"

            #print(f"[SIM] Spawning {priority_jid} at road {starting_road} → node {end_node}")

            priority_agent = PriorityAgent(
                jid=priority_jid,
                password=priority_pwd,
                starting_road=starting_road,
                end_node=end_node,
                travel_weight=TRAVEL_WEIGHT,
                capacity_weight=CAPACITY_WEIGHT,
                periodic_movement=MOVEMENT_TIMER,
                remove_callback=remove_callback
            )

            active_vehicles.add(priority_agent)

            await priority_agent.start()

            priority_counter += 1

        # Wait before spawning the next vehicle
        await asyncio.sleep(SPAWN_TIMER)  # spawn interval

    if WITH_INTERFACE:
        interface_stop_event.set()

# Run simulation
if __name__ == "__main__":
    run(simulation())