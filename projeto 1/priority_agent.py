"""
Priority vehicle agent.

This module defines the PriorityAgent, which represents a priority vehicle
(e.g., ambulance, firetruck) in the traffic simulation. The agent:
  - Requests routes from traffic lights to reach a given destination.
  - Moves along roads with priority handling at intersections.
  - Continuously notifies the current traffic light of its position so that
    traffic signals can be adjusted to favor its movement.
"""

from spade.agent import Agent
from spade.behaviour import PeriodicBehaviour, OneShotBehaviour, CyclicBehaviour
from spade.template import Template
from spade.message import Message

from collections import deque
import json
import asyncio
import sys


class PriorityAgent(Agent):
    """
    Agent that models a priority vehicle within the traffic system.

    Attributes
    ----------
    starting_road : int
        Initial road where the priority vehicle spawns.
    starting_lane : str or int
        Initial lane identifier ("any" means the TL can choose).
    end_node : int
        Destination node that the agent wants to reach.
    remove_callback : callable or None
        Optional function to be called when the agent is removed.
    is_priority : bool
        Flag indicating that this is a priority vehicle.
    periodic_movement : float
        Time interval (seconds) between movement updates.

    current_road : int
        Road where the vehicle is currently located.
    current_lane : str or int
        Lane where the vehicle is currently located.
    current_tl_jid : str
        JID of the traffic light agent controlling the current road.
    current_tl_signal : str or None
        Current signal ("red", "yellow", "green") received from the TL.
    current_lane_capacity : int or None
        Unused here; kept for consistency with car agent logic.
    total_road_capacity : int or None
        Maximum position / capacity for the current lane.
    position : int
        Current position of the vehicle in the lane (1..capacity).
    can_still_move_to : int or None
        Limit position the vehicle can move to within the lane.
    route : deque or None
        Remaining route as a deque of (road_id, lane_id) tuples.
    n_received_capacities : int
        Number of capacity responses received.
    received_capacities : dict
        Mapping from road_id to lane total capacity.
    wasted_fuel : float
        Placeholder metric for fuel consumption.
    fuel_consumption : dict
        Fuel consumption rates for different states.
    """

    def __init__(self, jid, password, starting_road, end_node, travel_weight, capacity_weight, periodic_movement, remove_callback=None):
        """
        Initialize a PriorityAgent.

        Parameters
        ----------
        jid : str
            XMPP JID of the agent.
        password : str
            Password for the XMPP server.
        starting_road : int
            Road where the priority vehicle starts.
        end_node : int
            Target destination node ID.
        periodic_movement : float
            Interval in seconds between movement steps.
        remove_callback : callable, optional
            Optional callback executed when the agent is removed.
        """
        super().__init__(jid, password)
        # immutable atributes
        self.starting_road = starting_road
        self.starting_lane = "any"
        self.end_node = end_node
        self.remove_callback = remove_callback
        self.is_priority = True
        self.periodic_movement = periodic_movement
        self.travel_weight = travel_weight
        self.capacity_weight = capacity_weight
        
        # dynamic attributes
        self.message_buffer = []
        self.can_spawn = None
        self.current_road = self.starting_road
        self.current_lane = self.starting_lane
        self.current_tl_jid = f"tl{self.current_road}@localhost"
        self.current_tl_signal = None
        self.requesting_capacity = False
        self.current_road_capacity = None
        self.total_road_capacity = 666
        self.position = 1
        self.can_still_move_to = 666
        self.possible_routes = None
        self.n_received_capacities = 0
        self.received_capacities = {}

    async def setup(self):
        """
        SPADE setup method.

        This is called once when the agent starts. It:
          - Prints a startup message.
          - Adds the cyclic behaviour to receive messages.
          - Adds the one-shot SpawnPriority behaviour to initialize movement.
        """
        #print(f"[{self.jid}] Car starting...")
        self.add_behaviour(self.ReceiveMessageBehaviour())

        self.add_behaviour(self.SpawnPriority())

    class SpawnPriority(OneShotBehaviour):
        """Set initial attributes for the priority vehicle and initialize its state."""

        async def run(self):
            """
            Initialize the priority vehicle before starting its periodic movement.

            Steps:
              1. Request lane capacity from the current traffic light.
              2. Request a route to the destination node.
              3. Start the DriveController periodic behaviour.
            """
            priority = self.agent

            await self.signal_to_update_priority_capacity()

            if priority.can_spawn == False:
                #print(f"[{priority.jid}] does not have space to spawn")

                if priority.remove_callback:
                    priority.remove_callback(priority)

                await priority.stop()
                self.kill()
                return

            await self.request_road_capacity()

            priority.add_behaviour(priority.DriveController(period=priority.periodic_movement))

        async def send_message(self, to: str, performative: str, ontology: str, action: str, body: str):
            """
            Helper to send a SPADE Message with the given metadata.

            Parameters
            ----------
            to : str
                Destination agent JID.
            performative : str
                ACL performative, e.g., "inform", "request".
            ontology : str
                Ontology identifier, e.g., "traffic-management".
            action : str
                Domain-specific action identifier.
            body : str
                Message payload (usually JSON-encoded).
            """
            priority = self.agent

            msg = Message(to=to, metadata={"performative": performative, "ontology": ontology, "action": action}, body=body)
            await self.send(msg)
            #print(f"[{priority.jid}] → Sent to {to}: ({performative}, {action}) {body}")

        async def signal_to_update_priority_capacity(self):
            """
            Inform the current traffic light that this priority vehicle is planning to enter the road.

            This increments the priority capacity usage at the traffic light side.
            """
            priority = self.agent

            message = json.dumps({"priority": str(priority.jid), "spawn": True})
            await self.send_message(to=priority.current_tl_jid, performative="inform", ontology="traffic-management", action="priority-entering-road", body=message)

            while priority.can_spawn == None:
                await asyncio.sleep(0.01)

        async def request_road_capacity(self):
            """
            Request the total road capacity for the current road from the traffic light.

            The agent waits until it receives a capacity message for the current
            road and then sets:
              - total_road_capacity
              - can_still_move_to
            """
            priority = self.agent

            priority.requesting_capacity = True
            message = json.dumps({"road": priority.current_road, "lane": priority.current_lane})
            await self.send_message(to=priority.current_tl_jid, performative="request", ontology="traffic-management", action="get-lane-capacity", body=message)

            while priority.current_road not in priority.received_capacities:
                await asyncio.sleep(0.01)
            
            priority.current_road_capacity, priority.total_road_capacity = priority.received_capacities[priority.current_road]
            priority.can_still_move_to = priority.total_road_capacity - priority.current_road_capacity + 1
            priority.requesting_capacity = False

            priority.received_capacities = {}
            priority.n_received_capacities = 0

    class DriveController(PeriodicBehaviour):
        """Main driving logic executed periodically."""

        async def run(self):
            """
            Perform one driving step for the priority vehicle.

            Logic overview:
              - Signal presence to the current traffic light.
              - If there is still space in the lane, move forward.
              - If at the end of the lane, check the traffic light:
                  * If green, try to move to the next road according to the route.
                  * If red, notify the traffic light that the priority vehicle is waiting.
              - If close to the end of the lane and blocked, request signal information.
            """
            priority = self.agent
            #print(f"[{priority.jid}] | road: {priority.current_road} | position: {priority.position} | can_still_move: {priority.can_still_move_to}")

            await self.signal_presence()

            if priority.position < priority.can_still_move_to:
                priority.position += 1
                return

            elif priority.position == priority.total_road_capacity:
                await self.request_tl_signal()

                if priority.current_tl_signal == "green":
                    await self.request_new_routes()
                    await self.request_capacities()

                    entering_road = self.choose_route()

                    if entering_road != None:
                        leaving_road = priority.current_road
                        await self.signal_to_update_priority_capacity(entering_road)
                        priority.received_capacities = {}
                        priority.n_received_capacities = 0
                        await self.request_capacity_for_next_road(entering_road)
                        await self.update_priority_attributes(entering_road)
                        await self.signal_left_road(leaving_road)
                        priority.current_tl_signal = None
                        priority.possible_routes = None
                        priority.received_capacities = {}
                        priority.n_received_capacities = 0
                        return
                    else:
                        return

                elif priority.current_tl_signal == "red":
                    await self.send_message(to=priority.current_tl_jid, performative="inform", ontology="traffic-management", action="priority-waiting", body="")
                    priority.current_tl_signal = None
                    return

            elif priority.position == priority.can_still_move_to and priority.position >= priority.total_road_capacity - 10:
                await self.request_tl_signal()
                if priority.current_tl_signal == "red":
                    await self.send_message(to=priority.current_tl_jid, performative="inform", ontology="traffic-management", action="priority-waiting", body="")
                    priority.current_tl_signal = None
                return

        async def send_message(self, to: str, performative: str, ontology: str, action: str, body: str):
            """
            Helper to send a SPADE Message from the DriveController behaviour.

            Parameters
            ----------
            to : str
                Destination agent JID.
            performative : str
                ACL performative, e.g., "inform", "request".
            ontology : str
                Ontology identifier, e.g., "traffic-management".
            action : str
                Domain-specific action identifier.
            body : str
                Message payload (usually JSON).
            """
            priority = self.agent

            msg = Message(to=to, metadata={"performative": performative, "ontology": ontology, "action": action}, body=body)
            await self.send(msg)
            #print(f"[{priority.jid}] → Sent to {to}: ({performative}, {action}) {body}")

        async def signal_presence(self):
            """
            Inform the current traffic light of the priority vehicle's position.

            This allows the traffic light to extend or adjust green phases
            to accommodate the priority vehicle.
            """
            priority = self.agent

            message = f"{priority.position}"
            await self.send_message(to=priority.current_tl_jid, performative="inform", ontology="traffic-management", action="priority-signaling-presence", body=message)

        async def request_tl_signal(self):
            """
            Request the current signal state from the traffic light.

            The method blocks (with small sleeps) until `current_tl_signal`
            is filled by the ReceiveMessageBehaviour.
            """
            priority = self.agent

            # Request traffic light for current signal
            await self.send_message(to=priority.current_tl_jid, performative="request", ontology="traffic-management", action="get-signal", body="")

            # Wait for message to arrive
            while priority.current_tl_signal == None:
                await asyncio.sleep(0.01)

        async def request_new_routes(self):
            """
            Request possible routes from the current traffic light agent.

            The TL agent will respond with a set of routes, which are stored
            in `priority.possible_routes` by the ReceiveMessageBehaviour.
            """
            priority = self.agent

            # Request routing agent for possible routes to destination
            message = json.dumps({"current_road": priority.current_road, "current_lane": priority.current_lane, "end_node": priority.end_node})
            await self.send_message(to=priority.current_tl_jid, performative="request", ontology="traffic-management", action="get-routes", body=message)

            # Wait for possible routes to arrive
            while priority.possible_routes == None:
                await asyncio.sleep(0.01)

        async def request_capacities(self):
            """
            Request priority capacities for all roads that appear in the possible routes.

            After receiving all capacities, this function computes an aggregate
            capacity cost for each route and appends it to the route data:
              route[2] = total capacity cost along the route.
            """
            priority = self.agent
            
            next_roads = set()
            for route in priority.possible_routes:
                path = route[0]
                cost = route[1]
                for road_lane in path:
                    road = road_lane[0]
                    if road not in next_roads:
                        next_roads.add(road)

            length_next_roads = len(next_roads)

            # Ask every relevant traffic light for lane capacity
            for next_road in next_roads:
                send_to = f"tl{next_road}@localhost"
                message = json.dumps({"road": next_road, "lane": "any"})
                await self.send_message(to=send_to, performative="request", ontology="traffic-management", action="get-lane-capacity", body=message)

            # Wait for all capacity responses
            while priority.n_received_capacities != length_next_roads:
                await asyncio.sleep(0.01)

            # For each route, compute total capacity cost
            for route in priority.possible_routes:
                temp_capacity = 0
                path = route[0]
                for road_lane in path:
                    road = road_lane[0]
                    temp_capacity += priority.received_capacities[road][0]
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
            int
                entering_road for the first hop of the chosen route,
                or None if no viable route is found.
            """
            priority = self.agent

            # Append total cost for each route
            for route in priority.possible_routes:
                travel_cost = route[1]
                capacity_cost = route[2]
                total_cost = (priority.travel_weight*travel_cost) + (priority.capacity_weight*capacity_cost)
                route.append(total_cost)

            # Sort routes by total cost
            priority.possible_routes.sort(key=lambda route: route[3])

            # Choose the first route whose first lane has free capacity
            for route in priority.possible_routes:
                path = route[0]
                first_road_lane = path[0]
                first_road = first_road_lane[0]
                first_road_current_capacity =  priority.received_capacities[first_road][0]
                first_road_total_capacity =  priority.received_capacities[first_road][1]
                if first_road_current_capacity < first_road_total_capacity:
                    return first_road

            return None

        async def signal_to_update_priority_capacity(self, entering_road):
            """
            Inform the current traffic light that this priority vehicle is planning to enter the road.

            This increments the priority capacity usage at the traffic light side.
            """
            priority = self.agent

            receiving_tl = f"tl{entering_road}@localhost"
            message = json.dumps({"priority": str(priority.jid), "spawn": False})
            await self.send_message(to=receiving_tl, performative="inform", ontology="traffic-management", action="priority-entering-road", body=message)

        async def request_capacity_for_next_road(self, entering_road):
            """
            Request the total road capacity for the current road from the traffic light.

            The agent waits until it receives a capacity message for the current
            road and then sets:
              - total_road_capacity
              - can_still_move_to
            """
            priority = self.agent

            priority.requesting_capacity = True
            message = json.dumps({"road": entering_road, "lane": priority.current_lane})
            receiving_tl = f"tl{entering_road}@localhost"
            await self.send_message(to=receiving_tl, performative="request", ontology="traffic-management", action="get-lane-capacity", body=message)

            while entering_road not in priority.received_capacities:
                await asyncio.sleep(0.01)
            
            priority.current_road_capacity, priority.total_road_capacity = priority.received_capacities[entering_road]
            priority.can_still_move_to = priority.total_road_capacity - priority.current_road_capacity + 1
            priority.requesting_capacity = False

            priority.received_capacities = {}
            priority.n_received_capacities = 0

        async def update_priority_attributes(self, entering_road):
            """
            Update internal attributes after successfully moving to a new road.

            Parameters
            ----------
            entering_road : int
                Identifier of the road just entered by the priority vehicle.
            """
            priority = self.agent

            priority.current_road = entering_road
            priority.current_tl_jid = f"tl{priority.current_road}@localhost"
            priority.position = 1

        async def signal_left_road(self, leaving_road):
            """
            Inform the previous traffic light that the priority vehicle left its road.

            Parameters
            ----------
            leaving_road : int
                Road identifier that the vehicle has just left.
            """
            send_to = f"tl{leaving_road}@localhost"
            await self.send_message(to=send_to, performative="inform", ontology="traffic-management", action="priority-left-road", body="")

    class ReceiveMessageBehaviour(CyclicBehaviour):
        """
        Behaviour responsible for receiving and handling messages sent to the priority agent.

        This includes:
          - Lane capacity information.
          - Traffic light signal state.
          - Route suggestions.
          - Destination reached notifications.
        """

        async def run(self):
            """
            Process incoming messages according to their performative, ontology and action.
            """
            priority = self.agent

            if len(priority.message_buffer) > 0:
                msg = priority.message_buffer.pop(0)
            else:
                msg = await self.receive(timeout=5)

            if not msg:
                return

            performative = msg.metadata.get("performative")
            ontology = msg.metadata.get("ontology")
            action = msg.metadata.get("action")
            sender = str(msg.sender)

            if performative == "inform" and ontology == "traffic-management" and action == "send-lane-capacity":
                data = json.loads(msg.body)
                road = data["road"]
                priority_capacity = data["priority_capacity"]
                total_capacity = data["total_capacity"]
                priority.received_capacities[road] = (priority_capacity, total_capacity)
                priority.n_received_capacities += 1

            elif performative == "inform" and ontology == "traffic-management" and action == "send-signal":
                signal = msg.body
                priority.current_tl_signal = signal

            elif performative == "inform" and ontology == "traffic-management" and action == "send-routes":
                routes = json.loads(msg.body)

                for route in routes:
                    route[0] = deque(tuple(x) for x in route[0])
                    route = tuple(route)

                priority.possible_routes = routes

            elif performative == "inform" and ontology == "traffic-management" and action == "can-spawn":
                if msg.body == "yes":
                    priority.can_spawn = True
                else:
                    priority.can_spawn = False

            elif performative == "inform" and ontology == "traffic-management" and action == "priority-left":
                while priority.requesting_capacity:
                    msg = await self.receive(timeout=5)

                    if not msg:
                        continue

                    performative = msg.metadata.get("performative")
                    ontology = msg.metadata.get("ontology")
                    action = msg.metadata.get("action")
                    sender = str(msg.sender)

                    if performative == "inform" and ontology == "traffic-management" and action == "send-lane-capacity":
                        # Received road capacity information for priority vehicles for a specific road
                        data = json.loads(msg.body)
                        road = data["road"]
                        priority_capacity = data["priority_capacity"]
                        total_capacity = data["total_capacity"]
                        priority.received_capacities[road] = (priority_capacity, total_capacity)
                        priority.n_received_capacities += 1
                        return
                    else:
                        priority.message_buffer.append(msg)

                # Another priority left the lane, so this priority can move one step further
                priority.can_still_move_to = min(priority.total_road_capacity, priority.can_still_move_to + 1)

            elif performative == "inform" and ontology == "traffic-management" and action == "reached-destination":
                # Priority has reached its destination
                #print(f"[{priority.jid}] has reached it's destination")

                # Inform TL that this priority left its current lane
                await self.send_message(to=priority.current_tl_jid, performative="inform", ontology="traffic-management", action="priority-left-road", body="")

                if priority.remove_callback:
                    priority.remove_callback(priority)

                await priority.stop()
                self.kill()
                return

        async def send_message(self, to: str, performative: str, ontology: str, action: str, body: str):
            """
            Helper to send a SPADE Message from the DriveController behaviour.

            Parameters
            ----------
            to : str
                Destination agent JID.
            performative : str
                ACL performative, e.g., "inform", "request".
            ontology : str
                Ontology identifier, e.g., "traffic-management".
            action : str
                Domain-specific action identifier.
            body : str
                Message payload (usually JSON).
            """
            priority = self.agent

            msg = Message(to=to, metadata={"performative": performative, "ontology": ontology, "action": action}, body=body)
            await self.send(msg)
            print(f"[{priority.jid}] → Sent to {to}: ({performative}, {action}) {body}")