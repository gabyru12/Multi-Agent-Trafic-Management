from spade.agent import Agent
from spade.behaviour import PeriodicBehaviour, OneShotBehaviour, CyclicBehaviour
from spade.template import Template
from spade.message import Message

from collections import deque
import json
import asyncio
import time

class CarAgent(Agent):
    """
    Agent representing a non-priority car in the traffic simulation.

    The car:
      - Spawns on a given start node and road.
      - Requests lane capacity from the corresponding traffic light.
      - Moves forward along its lane until it reaches the end of the road.
      - Requests possible routes and lane capacities to choose the next road.
      - Interacts with traffic lights based on their signals.
      - Reacts to the presence of priority vehicles by pulling over if needed.
      - Collects various metrics (travel time, distance, waiting times).
    """

    def __init__(self, jid, password, start_node ,starting_road, starting_lane, end_node, travel_weight=0.5, capacity_weight=0.5, periodic_movement=0.1, remove_callback=None):
        """
        Initialize a CarAgent.

        Parameters
        ----------
        jid : str
            XMPP JID of the agent.
        password : str
            Password for the XMPP server.
        start_node : int
            ID of the node where the car starts.
        starting_road : int
            ID of the road where the car initially spawns.
        end_node : int
            ID of the destination node.
        travel_weight : float, optional
            Weight associated with travel time in route cost calculation.
        capacity_weight : float, optional
            Weight associated with lane capacity in route cost calculation.
        periodic_movement : float, optional
            Time interval (seconds) between movement decisions.
        remove_callback : callable, optional
            Function to call when the agent is removed (despawned).
        """
        super().__init__(jid, password)
        # immutable attributes
        self.start_node = start_node
        self.starting_road = starting_road
        self.starting_lane = starting_lane
        self.end_node = end_node
        self.travel_weight = travel_weight
        self.capacity_weight = capacity_weight
        self.periodic_movement = periodic_movement
        self.remove_callback = remove_callback
        self.is_priority = False
        self.can_spawn = None
        
        # Dynamic state
        self.message_buffer = []
        self.priorities_in_road = {}
        self.current_road = self.starting_road
        self.current_lane = self.starting_lane
        self.current_tl_jid = f"tl{self.current_road}@localhost"
        self.current_tl_signal = None
        self.current_lane_capacity = None
        self.total_lane_capacity = 666
        self.position = 1
        self.can_still_move_to = 666
        self.requesting_capacity = False
        self.possible_routes = None
        self.n_received_capacities = 0
        self.received_capacities = {}
        self.priority_in_road = False
        self.pulled_over = False
        self.car_in_front_jid = None
        self.car_from_behind_jid = None
        self.waiting_for_possition_car_in_front = False
        self.position_car_in_front = None

        # Metrics dictionary to store per-car statistics
        self.metrics = {}
        self.finished_callback = None

    async def setup(self):
        """
        SPADE setup hook.

        This method:
          - Adds the ReceiveMessageBehaviour, which listens to all incoming messages.
          - Adds the SpawnCar behaviour, responsible for initial lane setup.
          - Initializes metrics such as start road, movement time, and timers.
        """
        #print(f"[{self.jid}] Car starting...")
        self.add_behaviour(self.ReceiveMessageBehaviour())

        self.add_behaviour(self.SpawnCar())

        self.metrics.setdefault("start_road", self.starting_road)
        self.metrics.setdefault("roads_taken", [(self.starting_road, self.starting_lane)])
        self.metrics.setdefault("periodic_movement_time", self.periodic_movement)
        self.metrics.setdefault("travel_time", time.time())
        self.metrics.setdefault("time_stopped_at_traffic_lights", [0])
        self.metrics.setdefault("total_distance", 0)
        self.metrics.setdefault("waiting_behind_other_cars_time", [0])
        self.metrics.setdefault("free_flow_time", [0])

    class SpawnCar(OneShotBehaviour):
        """One-shot behaviour that sets initial car attributes and registers it in the lane."""

        async def run(self):
            """
            Run the initial spawning routine for the car.

            Steps:
              1. Request lane capacity from the corresponding traffic light.
              2. If there is space, notify the traffic light to update lane capacity.
              3. Start the periodic DriveController behaviour.
            """
            car = self.agent
            await self.signal_to_update_lane_capacity()

            if car.can_spawn == False:
                #print(f"[{car.jid}] does not have space to spawn")

                if car.finished_callback:
                    car.finished_callback(car.metrics, car)

                if car.remove_callback:
                    car.remove_callback(car)

                await car.stop()
                self.kill()
                return
            
            await self.request_next_car_in_road_jid()
            await self.request_lane_capacity()

            car.add_behaviour(car.DriveController(period=car.periodic_movement))

        async def send_message(self, to: str, performative: str, ontology: str, action: str, body: str):
            """
            Helper method to send a SPADE message.

            Parameters
            ----------
            to : str
                Destination agent JID.
            performative : str
                ACL performative (e.g., "inform", "request").
            ontology : str
                Ontology of the message, typically "traffic-management".
            action : str
                Domain-specific action keyword.
            body : str
                String payload (often JSON-encoded).
            """
            car = self.agent

            msg = Message(to=to, metadata={"performative": performative, "ontology": ontology, "action": action}, body=body)
            await self.send(msg)
            print(f"[{car.jid}] → Sent to {to}: ({performative}, {action}) {body}")

        async def request_next_car_in_road_jid(self):
            car = self.agent

            message = json.dumps({"lane": car.current_lane})
            await self.send_message(to=car.current_tl_jid, performative="request", ontology="traffic-management", action="get-car-in-front-jid", body=message)

            while car.car_in_front_jid == None:
                await asyncio.sleep(0.01)

        async def request_lane_capacity(self):
            """
            Ask the current traffic light for lane capacity information.

            Waits until it receives the capacity for (current_road, current_lane)
            and sets:
              - current_lane_capacity
              - total_lane_capacity
              - can_still_move_to

            If there is no space to spawn, the agent is stopped and removed.
            """
            car = self.agent

            car.requesting_capacity = True
            message = json.dumps({"road": car.current_road, "lane": car.current_lane, "changing_roads": True})
            await self.send_message(to=car.current_tl_jid, performative="request", ontology="traffic-management", action="get-lane-capacity", body=message)

            while (car.current_road, car.current_lane) not in car.received_capacities:
                await asyncio.sleep(0.01)
            
            car.current_lane_capacity = car.received_capacities[(car.current_road, car.current_lane)][0]
            car.total_lane_capacity = car.received_capacities[(car.current_road, car.current_lane)][1]

            car.can_still_move_to = car.total_lane_capacity - car.current_lane_capacity + 1
            car.requesting_capacity = False

            car.received_capacities = {}
            car.n_received_capacities = 0

        async def signal_to_update_lane_capacity(self):
            """
            Inform the current traffic light that this car has entered the lane.

            This increments the lane capacity usage at the traffic light side.
            """
            car = self.agent

            message = json.dumps({"car": str(car.jid), "lane": car.current_lane, "event": "+", "spawn": True})
            await self.send_message(to=car.current_tl_jid, performative="inform", ontology="traffic-management", action="update-lane", body=message)

            while car.can_spawn == None:
                await asyncio.sleep(0.01)

    class DriveController(PeriodicBehaviour):
        """Periodic behaviour containing the main car driving logic."""

        async def run(self):
            """
            Execute one driving step for the car.

            Logic overview:
              - If a priority vehicle is in the road, the car may pull over
                and wait depending on its position.
              - If there is space ahead (position < can_still_move_to), it
                moves forward and updates metrics.
              - If at the end of the lane, it interacts with the traffic light:
                  * If green: request routes, capacities and attempt to move
                    to a next road/lane.
                  * If red: wait and accumulate waiting time metrics.
              - If blocked by other cars near the end of the road, it increases
                waiting time behind other cars and informs the traffic light.
            """
            car = self.agent
            print(f"[{car.jid}]: (Road: {car.current_road}, Lane: {car.current_lane}, position: {car.position}, can-still_move_to: {car.can_still_move_to}, car_in_front: {car.car_in_front_jid}, car_from_behind: {car.car_from_behind_jid}")

            if len(car.priorities_in_road) > 0:
                if True in car.priorities_in_road.values():
                    car.pulled_over = True
                else:
                    car.pulled_over = False
            else:
                car.pulled_over = False

            # Priority vehicle management: car might be pulled over or waiting
            if car.pulled_over:
                return

            # Free-flow movement: car can advance within lane capacity
            if car.position < car.can_still_move_to:
                if len(car.priorities_in_road) > 0:
                    if car.car_in_front_jid != "None":
                        await self.send_message(to=car.car_in_front_jid, performative="query", ontology="traffic-management", action="get-position", body="")
                        car.waiting_for_position_car_in_front = True

                        while car.waiting_for_position_car_in_front == True and car.car_in_front_jid != "None":
                            print(f"[{car.jid}] waiting for response of car in front")
                            await asyncio.sleep(0.01)

                        if car.car_in_front_jid == "None":
                            car.waiting_for_possition_car_in_front == False
                            car.metrics["total_distance"] += 1
                            car.metrics["free_flow_time"][-1] += car.periodic_movement
                            car.position += 1
                            return

                        if car.position_car_in_front > car.position + 1:
                            car.metrics["total_distance"] += 1
                            car.metrics["free_flow_time"][-1] += car.periodic_movement
                            car.position += 1
                        else:
                            return
                    else:
                        car.metrics["total_distance"] += 1
                        car.metrics["free_flow_time"][-1] += car.periodic_movement
                        car.position += 1
                else:
                    car.metrics["total_distance"] += 1
                    car.metrics["free_flow_time"][-1] += car.periodic_movement
                    car.position += 1

            # At the very end of the lane (intersection)
            elif car.position == car.total_lane_capacity:
                await self.request_tl_signal()

                if car.current_tl_signal == "green":
                    # Try to move to the next road/lane
                    await self.request_new_routes()
                    await self.request_capacities()

                    entering_road, entering_lane = self.choose_route()

                    if entering_road != None and entering_lane != None:
                        leaving_road, leaving_lane = car.current_road, car.current_lane
                        await self.signal_to_update_lane_capacities(leaving_road, entering_road, leaving_lane, entering_lane)
                        car.n_received_capacities = 0
                        car.received_capacities = {}
                        await self.request_capacity_for_next_lane(entering_road, entering_lane)
                        await self.update_car_attributes(entering_road, entering_lane)

                        car.metrics["roads_taken"].append((entering_road, entering_lane))

                        if car.car_from_behind_jid != None:
                            await self.send_message(to=car.car_from_behind_jid, performative="inform", ontology="traffic-management", action="your-front-car-has-left-lane", body="")

                        car.car_from_behind_jid = None

                        message = json.dumps({"lane": car.current_lane})
                        await self.send_message(to=car.current_tl_jid, performative="request", ontology="traffic-management", action="get-car-in-front-jid", body=message)

                        while car.car_in_front_jid == None:
                            print(f"[{car.jid}] waiting for jid of car in front")
                            await asyncio.sleep(0.01)

                        car.current_tl_signal = None
                        car.possible_routes = None
                        car.n_received_capacities = 0
                        car.received_capacities = {}
                        car.priorities_in_road = {}
                        return
                    else:
                        car.current_tl_signal = None
                        car.possible_routes = None
                        car.n_received_capacities = 0
                        car.received_capacities = {}
                        return

                elif car.current_tl_signal == "red":
                    # Stopped at the traffic light
                    car.metrics["time_stopped_at_traffic_lights"][-1] += car.periodic_movement

                    message = f"{car.periodic_movement}"
                    await self.send_message(to=car.current_tl_jid, performative="inform", ontology="traffic-management", action="car-waiting", body=message)
                    car.current_tl_signal = None
                    return

            # Close to the front of the queue, but blocked
            elif car.position == car.can_still_move_to and car.position >= car.total_lane_capacity - 10:
                await self.request_tl_signal()
                if car.current_tl_signal == "red":
                    car.metrics["time_stopped_at_traffic_lights"][-1] += car.periodic_movement
                    car.metrics["waiting_behind_other_cars_time"][-1] += car.periodic_movement

                    message = f"{car.periodic_movement}"
                    await self.send_message(to=car.current_tl_jid, performative="inform", ontology="traffic-management", action="car-waiting", body=message)

                car.current_tl_signal = None
                return

            # Blocked by other cars somewhere in the lane (not yet at capacity limit)
            elif car.position == car.can_still_move_to and car.position < car.total_lane_capacity:
                car.metrics["waiting_behind_other_cars_time"][-1] += car.periodic_movement

        async def send_message(self, to: str, performative: str, ontology: str, action: str, body: str):
            """
            Helper method to send a SPADE message from DriveController.

            Parameters
            ----------
            to : str
                Destination agent JID.
            performative : str
                ACL performative (e.g., "inform", "request").
            ontology : str
                Ontology identifier, typically "traffic-management".
            action : str
                Domain-specific action keyword.
            body : str
                Body of the message as a string.
            """
            car = self.agent

            msg = Message(to=to, metadata={"performative": performative, "ontology": ontology, "action": action}, body=body)
            await self.send(msg)
            print(f"[{car.jid}] → Sent to {to}: ({performative}, {action}) {body}")

        async def request_capacity_for_next_lane(self, entering_road, entering_lane):
            """
            Request lane capacity for the lane car wants to go to after evaluating possible routes

            Waits for information to arrive at car's received capacities
            """
            car = self.agent

            car.requesting_capacity = True
            send_to = f"tl{entering_road}@localhost"
            message = json.dumps({"road": entering_road, "lane": entering_lane, "changing_roads": True})
            await self.send_message(to=send_to, performative="request", ontology="traffic-management", action="get-lane-capacity", body=message)

            while (entering_road, entering_lane) not in car.received_capacities:
                print(f"[{car.jid}] waiting for capacities")
                await asyncio.sleep(0.01)

        async def request_tl_signal(self):
            """
            Request the current traffic light signal from the TL agent.

            This method blocks (with small sleeps) until `current_tl_signal`
            is filled by the ReceiveMessageBehaviour.
            """
            car = self.agent

            # Request traffic light for current signal
            await self.send_message(to=car.current_tl_jid, performative="request", ontology="traffic-management", action="get-signal", body="")

            # Wait for message to arrive
            while car.current_tl_signal == None:
                print(f"[{car.jid}] waiting for tl signal")
                await asyncio.sleep(0.01)

        async def request_new_routes(self):
            """
            Request possible routes from the current traffic light agent.

            The TL agent will respond with a set of routes, which are stored
            in `car.possible_routes` by the ReceiveMessageBehaviour.
            """
            car = self.agent

            # Request routing agent for possible routes to destination
            message = json.dumps({"current_road": car.current_road, "current_lane": car.current_lane, "end_node": car.end_node})
            await self.send_message(to=car.current_tl_jid, performative="request", ontology="traffic-management", action="get-routes", body=message)

            # Wait for possible routes to arrive
            while car.possible_routes == None:
                print(f"[{car.jid}] waiting for routes")
                print(f"[{car.jid}]: (Road: {car.current_road}, Lane: {car.current_lane}, position: {car.position}, can-still_move_to: {car.can_still_move_to}, car_in_front: {car.car_in_front_jid}, car_from_behind: {car.car_from_behind_jid}")
                await asyncio.sleep(0.01)

        async def request_capacities(self):
            """
            Request lane capacities for all lanes that appear in the possible routes.

            After receiving all capacities, this function computes an aggregate
            capacity cost for each route and appends it to the route data:
              route[2] = total capacity cost along the route.
            """
            car = self.agent

            next_road_lanes = set()
            for route in car.possible_routes:
                path = route[0]
                cost = route[1]
                for road_lane in path:
                    if road_lane not in next_road_lanes:
                        next_road_lanes.add(road_lane)

            length_next_road_lanes = len(next_road_lanes)

            # Ask every relevant traffic light for lane capacity
            for next_road_lane in next_road_lanes:
                road = next_road_lane[0]
                lane = next_road_lane[1]
                send_to = f"tl{road}@localhost"
                message = json.dumps({"road": road, "lane": lane, "changing_roads": False})
                await self.send_message(to=send_to, performative="request", ontology="traffic-management", action="get-lane-capacity", body=message)

            # Wait for all capacity responses
            while car.n_received_capacities != length_next_road_lanes:
                print(f"[{car.jid}] waiting for road capacities")
                await asyncio.sleep(0.01)

            # For each route, compute total capacity cost
            for route in car.possible_routes:
                temp_capacity = 0
                path = route[0]
                for road_lane in path:
                    temp_capacity += car.received_capacities[road_lane][0]
                route.append(temp_capacity)

        def choose_route(self):
            """
            Select the best route based on travel and capacity costs.

            The route structure is:
              route[0] = path (deque of (road, lane))
              route[1] = travel cost
              route[2] = capacity cost (sum of lane occupancies along the path)

            This method computes:
              total_cost = travel_weight * travel_cost + capacity_weight * capacity_cost

            The routes are sorted by this total cost, and the first route whose
            first lane has available capacity is chosen.

            Returns
            -------
            tuple
                (entering_road, entering_lane) for the first hop of the chosen route,
                or (None, None) if no viable route is found.
            """
            car = self.agent

            # Append total cost for each route
            for route in car.possible_routes:
                travel_cost = route[1]
                capacity_cost = route[2]
                total_cost = (car.travel_weight*travel_cost) + (car.capacity_weight*capacity_cost)
                route.append(total_cost)

            # Sort routes by total cost
            car.possible_routes.sort(key=lambda route: route[3])

            # Choose the first route whose first lane has free capacity
            for route in car.possible_routes:
                path = route[0]
                first_road_lane = path[0]
                first_lane_current_capacity =  car.received_capacities[first_road_lane][0]
                first_lane_total_capacity =  car.received_capacities[first_road_lane][1]
                if first_lane_current_capacity < first_lane_total_capacity:
                    return first_road_lane

            return (None, None)

        async def update_car_attributes(self, entering_road, entering_lane):
            """
            Update the car's attributes when moving to a new road/lane.

            Parameters
            ----------
            entering_road : int
                ID of the road being entered.
            entering_lane : int
                Lane index of the road being entered.
            """
            car = self.agent

            car.current_road, car.current_lane = entering_road, entering_lane
            car.current_tl_jid = f"tl{car.current_road}@localhost"
            car.position = 1
            currentCapacity, totalCapacity = car.received_capacities[(car.current_road, car.current_lane)]
            car.current_lane_capacity, car.total_lane_capacity = currentCapacity, totalCapacity
            car.can_still_move_to = car.total_lane_capacity - car.current_lane_capacity + 1
            car.requesting_capacity = False

        async def signal_to_update_lane_capacities(self, leaving_road, entering_road, leaving_lane, entering_lane):
            """
            Notify traffic lights that the car left one lane and entered another.

            Also initializes new metric entries for the next segment.

            Parameters
            ----------
            leaving_road : int
                Road ID the car is leaving.
            entering_road : int
                Road ID the car is entering.
            leaving_lane : int
                Lane index the car is leaving.
            entering_lane : int
                Lane index the car is entering.
            """
            car = self.agent
            car.metrics["time_stopped_at_traffic_lights"].append(0)
            car.metrics["waiting_behind_other_cars_time"].append(0)
            car.metrics["free_flow_time"].append(0)

            # Request traffic light to update lane capacities for the current road and previous road
            previous_road_tl_jid = f"tl{leaving_road}@localhost"
            message = json.dumps({"car": str(car.jid), "lane": leaving_lane, "event": "-"})
            await self.send_message(to=previous_road_tl_jid, performative="inform", ontology="traffic-management", action="update-lane", body=message)

            next_road_tl_jid = f"tl{entering_road}@localhost"
            message = json.dumps({"car": str(car.jid), "lane": entering_lane, "event": "+", "spawn": False})
            await self.send_message(to=next_road_tl_jid, performative="inform", ontology="traffic-management", action="update-lane", body=message)
       
    class ReceiveMessageBehaviour(CyclicBehaviour):
        """
        Cyclic behaviour that handles all incoming messages for the car.

        It processes:
          - Lane capacity updates.
          - Traffic light signals.
          - Routes sent by traffic lights.
          - Destination reached notifications.
          - Priority vehicle warnings and related coordination.
        """

        async def run(self):
            """
            Receive and handle messages according to their performative, ontology and action.
            """
            car = self.agent

            if len(car.message_buffer) > 0:
                msg = car.message_buffer.pop(0)
            else:
                msg = await self.receive(timeout=5)

            if not msg:
                return

            performative = msg.metadata.get("performative")
            ontology = msg.metadata.get("ontology")
            action = msg.metadata.get("action")
            sender = str(msg.sender)

            if performative == "inform" and ontology == "traffic-management" and action == "lane-cleared":
                while car.requesting_capacity:
                    print(f"[{car.jid}] waiting for capacities before updating can_still_move_to")
                    msg = await self.receive(timeout=5)

                    if not msg:
                        continue

                    performative = msg.metadata.get("performative")
                    ontology = msg.metadata.get("ontology")
                    action = msg.metadata.get("action")
                    sender = str(msg.sender)

                    if performative == "inform" and ontology == "traffic-management" and action == "send-lane-capacity":
                        # Received lane capacity information for a specific (road, lane)
                        data = json.loads(msg.body)
                        road = data["road"]
                        lane = data["lane"]
                        current_capacity = data["current_capacity"]
                        total_capacity = data["total_capacity"]
                        car.received_capacities[(road,lane)] = (current_capacity, total_capacity)
                        car.n_received_capacities += 1
                        return
                    else:
                        car.message_buffer.append(msg)

                # Another car left the lane, so this car can move one step further
                car.can_still_move_to = min(car.total_lane_capacity, car.can_still_move_to + 1)

            elif performative == "inform" and ontology == "traffic-management" and action == "send-lane-capacity":

                # Received lane capacity information for a specific (road, lane)
                data = json.loads(msg.body)
                road = data["road"]
                lane = data["lane"]
                current_capacity = data["current_capacity"]
                total_capacity = data["total_capacity"]
                car.received_capacities[(road,lane)] = (current_capacity, total_capacity)
                car.n_received_capacities += 1

            elif performative == "inform" and ontology == "traffic-management" and action == "send-signal":
                # Current traffic light signal for this car's road
                signal = msg.body
                car.current_tl_signal = signal

            elif performative == "inform" and ontology == "traffic-management" and action == "send-routes":
                # Possible routes received from the traffic light
                routes = json.loads(msg.body)

                for route in routes:
                    route[0] = deque(tuple(x) for x in route[0])
                    route = tuple(route)

                car.possible_routes = routes

            elif performative == "inform" and ontology == "traffic-management" and action == "reached-destination":
                # Car has reached its destination
                #print(f"[{car.jid}] has reached it's destination")

                start_time = car.metrics["travel_time"]
                car.metrics["travel_time"] = time.time() - start_time

                if car.car_from_behind_jid == None:
                    while True:
                        msg = await self.receive(timeout=0.1)

                        if not msg:
                            break

                        performative = msg.metadata.get("performative")
                        ontology = msg.metadata.get("ontology")
                        action = msg.metadata.get("action")
                        sender = str(msg.sender)

                        if performative == "inform" and ontology == "traffic-management" and action == "send-car-from-behind-jid":
                            data = json.loads(msg.body)
                            car.car_from_behind_jid = data["car_from_behind"]
                            break
                        else:
                            car.message_buffer.append(msg)

                # Inform TL that this car left its current lane
                message = json.dumps({"car": str(car.jid), "lane": car.current_lane, "event": "-"})
                await self.send_message(to=car.current_tl_jid, performative="inform", ontology="traffic-management", action="update-lane", body=message)

                await self.send_message(to=car.car_from_behind_jid, performative="inform", ontology="traffic-management", action="your-front-car-has-left-lane", body="")

                if car.finished_callback:
                    car.finished_callback(car.metrics, car)

                if car.remove_callback:
                    car.remove_callback(car)

                await car.stop()
                self.kill()
                return

            elif performative == "inform" and ontology == "traffic-management" and action == "can-spawn":
                if msg.body == "yes":
                    car.can_spawn = True
                else:
                    car.can_spawn = False

            elif performative == "inform" and ontology == "traffic-management" and action == "warn-priority-in-road":
                if sender != car.current_tl_jid:
                    return
                    
                data = json.loads(msg.body)
                priority_in_road = data["priority_jid"]
                priority_position = data["position"]

                # Decide whether this car should pull over based on its position
                if car.position >= priority_position and car.position < priority_position + 5:
                    car.priorities_in_road[priority_in_road] = True
                else:
                    car.priorities_in_road[priority_in_road] = False

            elif performative == "inform" and ontology == "traffic-management" and action == "priority-has-left":
                data = json.loads(msg.body)
                priority_that_left_road = data["priority_jid"]

                if priority_that_left_road in car.priorities_in_road.keys():
                    del car.priorities_in_road[priority_that_left_road]

            elif performative == "inform" and ontology == "traffic-management" and action == "send-car-in-front-jid":
                data = json.loads(msg.body)
                car.car_in_front_jid = data["car_in_front"]

            elif performative == "inform" and ontology == "traffic-management" and action == "send-car-from-behind-jid":
                data = json.loads(msg.body)
                car.car_from_behind_jid = data["car_from_behind"]

            elif performative == "inform" and ontology == "traffic-management" and action == "your-front-car-has-left-lane":
                car.car_in_front_jid = "None"

            elif performative == "query" and ontology == "traffic-management" and action == "get-position":
                message = json.dumps({"position": car.position})
                await self.send_message(to=sender, performative="inform", ontology="traffic-management", action="send-position", body=message)

            elif performative == "inform" and ontology == "traffic-management" and action == "send-position":
                data = json.loads(msg.body)
                car.position_car_in_front = data["position"]
                car.waiting_for_position_car_in_front = False
        
        async def send_message(self, to: str, performative: str, ontology: str, action: str, body: str):
            """
            Helper method to send a SPADE message from ReceiveMessageBehaviour.

            Parameters
            ----------
            to : str
                Destination agent JID.
            performative : str
                ACL performative.
            ontology : str
                Ontology identifier, typically "traffic-management".
            action : str
                Domain-specific action keyword.
            body : str
                Message body as a string.
            """
            car = self.agent

            msg = Message(to=to, metadata={"performative": performative, "ontology": ontology, "action": action}, body=body)
            await self.send(msg)
            print(f"[{car.jid}] → Sent to {to}: ({performative}, {action}) {body}")
