from spade.agent import Agent
from spade.behaviour import OneShotBehaviour, CyclicBehaviour
from spade.template import Template
from spade.message import Message

from collections import deque
import json
import asyncio
import heapq
import time


class TrafficLightAgent(Agent):
    """
    Traffic light agent responsible for controlling a single road segment.

    This agent:
      - Manages lane capacities and the current signal state (red/yellow/green).
      - Responds to cars' requests about signal state, lane capacity and routing.
      - Coordinates with neighbouring traffic lights to decide green cycles.
      - Supports two selection methods: cyclic rotation and proposal-based.
    """

    def __init__(
        self,
        jid: str,
        password: str,
        road_id: int,
        network,
        green_light_min_time,
        green_light_max_time,
        yellow_light_time,
        clear_intersection_time,
        selection_method: str = "proposals",  # cyclic or proposals
    ):
        """
        Initialize a TrafficLightAgent.

        Parameters
        ----------
        jid : str
            XMPP JID of the agent.
        password : str
            Password for the XMPP server.
        road_id : int
            Identifier of the road segment controlled by this traffic light.
        network
            Reference to the overall road network object.
        green_light_min_time
            Minimum allowed time for a green light interval (seconds).
        green_light_max_time
            Maximum allowed time for a green light interval (seconds).
        yellow_light_time
            Duration of the yellow light phase (seconds).
        clear_intersection_time
            Additional time after red to let the intersection clear (seconds).
        selection_method : str, optional
            Method to select the next traffic light with green; either
            "cyclic" (fixed order) or "proposals" (based on CFP/proposals).
        """
        super().__init__(jid, password)
        self.road_id = road_id
        self.network = network
        self.green_light_min_time = green_light_min_time
        self.green_light_max_time = green_light_max_time
        self.yellow_light_time = yellow_light_time
        self.clear_intersection_time = clear_intersection_time

        self.selection_method = selection_method

        self.intersection = None
        self.road_capacity = None
        self.n_lanes = None

        self.TL_group = []
        self.intersection_tl_order = []     

        self.closed_roads = []
        self.lane_capacities = None
        self.cars_in_lane = None
        self.priority_capacity = 0
        self.priorities_in_road = deque()
        self.signal = "red"
        self.tolerance = 0
        self.received_proposals = {}
        self.signal_time_behaviour = None
        self.priority_incoming = False
        self.car_positions = {}

        self.metrics = {}
        self.finished_callback = None

    def init_properties(self):
        """
        Initialize agent properties derived from the network.

        This loads:
          - Intersection ID (end node of the road).
          - Road capacity and lane count.
          - Per-lane capacity counters and queues of cars.
          - List of traffic lights in this intersection and their cyclic order.
        """
        road = self.network.roads[self.road_id]
        self.intersection = road.endNode.ID
        self.road_capacity = road.length
        self.n_lanes = road.laneCount

        self.lane_capacities = {i: 0 for i in range(self.n_lanes)}
        self.cars_in_lane = {i: deque() for i in range(self.n_lanes)}

        node = self.network.nodes[self.intersection]

        # Fixed (cyclic) order of incoming roads' traffic lights in this intersection
        road_ids_in_intersection = sorted(r.ID for r in node.inRoads)
        self.intersection_tl_order = [
            f"tl{r_id}@localhost" for r_id in road_ids_in_intersection
        ]

        # TL_group = only the other traffic lights (used for yielding, proposals, etc.)
        self.TL_group = [
            jid for jid in self.intersection_tl_order
            if jid != str(self.jid)
        ]

        for i in range(self.n_lanes):
            self.car_positions.setdefault(i, dict())

        self.metrics.setdefault("n_leaving_cars_per_cycle", [])
        self.metrics.setdefault("total_green_time_per_cycle", [])
        self.metrics.setdefault("total_time_lost", 0)
        self.metrics.setdefault("tolerance_built_up", [])
        self.metrics.setdefault("queue_length_per_cycle", [])

    def dijkstra(self, start_road_ID: int, start_lane_UID: int, end_node_ID: int):
        """
        Run Dijkstra's shortest-path algorithm on lanes.

        Parameters
        ----------
        start_road_ID : int
            Road where the car is currently located.
        start_lane_UID : int
            UID of the lane where the car currently is.
        end_node_ID : int
            Target node the car wants to reach.

        Returns
        -------
        tuple
            (path, path_cost)
            path : collections.deque
                Deque of tuples (road_id, lane_uid) representing the lane sequence.
                Returns an empty deque if no path exists.
            path_cost : float or None
                Total path cost (e.g. length). None if no path was found.
        """
        routes = []
        startRoad = self.network.roads[start_road_ID]
        startLane = self.network.lanes[start_lane_UID]

        # Priority queue: (cost, counter, lane)
        pq = []
        parent = {}        # lane → previous lane
        dist = {}          # lane → distance
        visited = set()
        counter = 0        # unique increasing ID (tie-breaker)

        # Initialize Dijkstra with the car's current lane
        dist[startLane] = 0
        parent[startLane] = None
        heapq.heappush(pq, (0, counter, startLane))
        counter += 1
        goal_lane = None
        path_cost = None

        # ----- Dijkstra search -----
        while pq:
            current_cost, _, lane = heapq.heappop(pq)

            if lane in visited:
                continue
            visited.add(lane)

            road = lane.road

            # Goal condition: the lane arrives at the desired node
            if road.endNode.ID == end_node_ID:
                goal_lane = lane
                path_cost = current_cost
                break

            # Relax neighbors (the lanes you can drive into next)
            for next_lane in lane.outFlowLanes:
                if next_lane.road.ID in self.closed_roads:
                    continue

                road_id = next_lane.road.ID
                road = self.network.roads[road_id]

                new_cost = current_cost + next_lane.length

                if next_lane not in dist or new_cost < dist[next_lane]:
                    dist[next_lane] = new_cost
                    parent[next_lane] = lane
                    heapq.heappush(pq, (new_cost, counter, next_lane))
                    counter += 1

        # ----- No route found -----
        if goal_lane is None:
            return deque(), path_cost

        # ----- Reconstruct the lane-by-lane route -----
        path = deque()
        lane = goal_lane

        while lane is not None:
            path.appendleft((lane.road.ID, lane.UID))
            lane = parent[lane]

        return path, path_cost

    def get_next_tl_in_cycle(self) -> str:
        """
        Get the JID of the next traffic light in the cyclic order for this intersection.

        Returns
        -------
        str
            JID of the next traffic light in the cycle. If this traffic light is not
            in the known order, it falls back to returning its own JID.
        """
        my_jid = str(self.jid)

        if my_jid not in self.intersection_tl_order:
            return my_jid   # fallback

        idx = self.intersection_tl_order.index(my_jid)
        next_idx = (idx + 1) % len(self.intersection_tl_order)
        return self.intersection_tl_order[next_idx]

    def generate_routes(self, start_road_ID, start_lane_ID, end_node_ID, K):
        """
        Compute the K-shortest loopless paths using Yen’s algorithm.

        The method uses `self.dijkstra()` as the subroutine for finding the
        current shortest path.

        Parameters
        ----------
        start_road_ID
            Road ID where the car starts.
        start_lane_ID
            Lane index within the starting road.
        end_node_ID
            Destination node ID.
        K
            Number of shortest loopless paths to compute.

        Returns
        -------
        list
            List of tuples (path, cost), where each path is a deque of
            (road_id, lane_id) pairs. If no path exists, returns an empty list.
        """
        start_lane_UID = self.network.roads[start_road_ID].lanes[start_lane_ID].UID

        # ---- First shortest path (Dijkstra) ----
        first_path, first_cost = self.dijkstra(start_road_ID, start_lane_UID, end_node_ID)
        if not first_path:
            return []   # no path exists
        A = [(first_path, first_cost)]   # shortest paths found
        B = []                           # candidate paths: (cost, path)

        # ---- Loop to find paths 2..K ----
        for k in range(1, K):
            prev_path, prev_cost = A[k-1]

            prev_path_list = list(prev_path)
            num_nodes = len(prev_path_list)

            # Spur at every node except the last
            for i in range(num_nodes - 1):

                # ---- Root path ----
                root_path = prev_path_list[:i]

                if not root_path:
                    spur_road_ID  = start_road_ID
                    spur_lane_UID  = start_lane_UID
                else:
                    spur_road_ID, spur_lane_UID = root_path[-1]

                # ---- Temporarily remove roads that recreate previously found paths ----
                removed_roads = {}
                for p, _ in A:
                    p_list = list(p)
                    if len(p_list) > i and p_list[:i] == root_path:
                        banned_road_ID, banned_lane_UID = p_list[i]
                        lane = self.network.lanes[banned_lane_UID]
                        if lane not in removed_roads:
                            removed_roads[lane] = lane.outFlowLanes
                        lane.outFlowLanes = []

                # ---- Compute spur path ----
                spur_path, spur_cost = self.dijkstra(spur_road_ID, spur_lane_UID, end_node_ID)

                # ---- If spur path exists, build full path candidate ----
                if spur_path:
                    spur_list = list(spur_path)
                    if spur_list and root_path:
                        # drop the first lane of the spur, because it repeats root's last lane
                        spur_list = spur_list[1:]

                    new_full_path = deque(root_path + spur_list)
                    new_full_cost = sum(self.network.lanes[laneUID].length for _, laneUID in new_full_path) - self.network.roads[new_full_path[0][0]].length

                    heapq.heappush(B, (new_full_cost, new_full_path))

                # ---- Restore removed outflows ----
                for lane, original in removed_roads.items():
                    lane.outFlowLanes = original

            # ---- No more candidates ----
            if not B:
                break

            # ---- Choose the shortest remaining candidate ----
            cost, path = heapq.heappop(B)
            A.append((path, cost))

        # Convert internal lane UIDs to lane IDs for output
        for j in range(len(A)):
            A[j] = list(A[j])
            for i in range(len(A[j][0])):
                A[j][0][i] = list(A[j][0][i])
                A[j][0][i][1] = self.network.lanes[A[j][0][i][1]].ID
                A[j][0][i] = tuple(A[j][0][i])
            A[j][0].popleft()
            A[j] = tuple(A[j])
        return A

    async def setup(self):
        """
        SPADE agent setup hook.

        This is called once when the agent is started. It initializes
        properties derived from the network and adds the main cyclic
        behaviour responsible for message handling.
        """
        #print(f"[{self.jid}] Traffic light for road {self.road_id} started.")
        self.init_properties()
        self.add_behaviour(self.ReceiveMessageBehaviour())

    # -------------
    # Behaviours 
    # -------------
    class ReceiveMessageBehaviour(CyclicBehaviour):
        """
        Main cyclic behaviour that receives and processes all messages.

        This behaviour:
          - Replies to cars asking for signal state, capacity, or routes.
          - Handles lane updates (cars entering/leaving).
          - Coordinates priority vehicles and informs cars accordingly.
          - Participates in CFP/proposal negotiation for green light turns.
          - Triggers or reacts to cycle start events depending on the policy.
        """

        async def run(self):
            """
            Periodically receive and process incoming ACL messages.

            The logic branches based on:
              - performative
              - ontology
              - action

            Each combination identifies a specific interaction pattern
            in the traffic management domain.
            """
            tl = self.agent
            msg = await self.receive(timeout=5)

            if not msg:
                return

            performative = msg.metadata.get("performative")
            ontology = msg.metadata.get("ontology")
            action = msg.metadata.get("action")
            sender = str(msg.sender)

            if performative == "request" and ontology == "traffic-management" and action == "get-signal":
                message = tl.signal
                await self.send_message(to=sender, performative="inform", ontology="traffic-management", action="send-signal", body=message)
                
            elif performative == "request" and ontology == "traffic-management" and action == "get-lane-capacity":
                data = json.loads(msg.body)

                if data["lane"] != "any":
                    current_capacity = tl.lane_capacities[data["lane"]]
                    if data["changing_roads"] == True:
                        current_capacity = tl.cars_in_lane[data["lane"]].index(sender) + 1
                    total_capacity = tl.road_capacity
                    message = json.dumps({"road": data["road"], "lane": data["lane"], "current_capacity": current_capacity, "total_capacity": total_capacity})
                    await self.send_message(to=sender, performative="inform", ontology="traffic-management", action="send-lane-capacity", body=message)
                else:
                    priority_capacity = tl.priority_capacity
                    total_capacity = tl.road_capacity
                    message = json.dumps({"road": data["road"], "priority_capacity": priority_capacity, "total_capacity": total_capacity})
                    await self.send_message(to=sender, performative="inform", ontology="traffic-management", action="send-lane-capacity", body=message)

            elif performative == "request" and ontology == "traffic-management" and action == "get-routes":
                data = json.loads(msg.body)
                road_id, lane_id, end_node_id = data["current_road"], data["current_lane"], data["end_node"]

                # If the current road already ends at the destination node, notify arrival
                if tl.network.roads[data["current_road"]].endNode.ID == data["end_node"]:
                    await self.send_message(to=sender, performative="inform", ontology="traffic-management", action="reached-destination", body="You've arrived at your destination")
                    return

                # If a specific lane is requested, generate routes from that lane
                if lane_id != "any":
                    routes = tl.generate_routes(road_id, lane_id, end_node_id, 20)

                else:
                    # If no lane is specified, compute routes from every lane on this road
                    routes = tl.generate_routes(road_id, 0, end_node_id, 5)

                    for lane_id in range(1, tl.n_lanes):
                        routes.extend(tl.generate_routes(road_id, lane_id, end_node_id, 5))

                # Convert routes to JSON-serializable structure
                for i in range(len(routes)):
                        routes[i] = list(routes[i])
                        routes[i][0] = list(routes[i][0])
                        routes[i] = tuple(routes[i])

                message = json.dumps(routes)
                await self.send_message(to=sender, performative="inform", ontology="traffic-management", action="send-routes", body=message)

            elif performative == "inform" and ontology == "traffic-management" and action == "update-lane":
                data = json.loads(msg.body)
                car = str(data["car"])
                affected_lane = data["lane"]
                event = data["event"]

                # Car entering a lane
                if event == "+":
                    if data["spawn"] == True:
                        if tl.lane_capacities[affected_lane] >= tl.road_capacity:
                            await self.send_message(to=sender, performative="inform", ontology="traffic-management", action="can-spawn", body="no")
                            return
                        else:
                            await self.send_message(to=sender, performative="inform", ontology="traffic-management", action="can-spawn", body="yes")

                    tl.lane_capacities[affected_lane] += 1
                    tl.cars_in_lane[affected_lane].append(car)

                # Car leaving a lane
                elif event == "-":
                    tl.metrics["n_leaving_cars_per_cycle"][-1] += 1

                    if car in tl.car_positions[affected_lane]:
                        del tl.car_positions[affected_lane][car]
                    tl.lane_capacities[affected_lane] -= 1
                    tl.cars_in_lane[affected_lane].popleft()
                    await self.inform_car_has_left(affected_lane)

            elif performative == "inform" and ontology == "traffic-management" and action == "car-waiting":
                # Car informs that it is waiting; tolerance accumulates
                tl.tolerance += float(msg.body)                    

            elif performative == "inform" and ontology == "traffic-management" and action == "yielded-green-signal":
                # Another TL has just yielded its green; start this TL's cycle
                template = Template(metadata={"performative": "request","ontology": "traffic-management","action": "yield-green-signal"})
                tl.signal_time_behaviour = tl.SignalTimeBehaviour()
                tl.add_behaviour(tl.signal_time_behaviour, template)

            elif performative == "inform" and ontology == "traffic-management" and action == "priority-signaling-presence":
                # A priority vehicle (e.g., ambulance) is incoming
                tl.priority_incoming = True
                tl.tolerance += 1000

                # Force neighbouring TLs to yield if this TL is not already green
                if tl.signal != "green":
                    for neighbouring_tl in tl.TL_group:
                        await self.send_message(to=neighbouring_tl, performative="request", ontology="traffic-management", action="yield-green-light", body="")

                # Inform all cars in this road about the priority vehicle position
                for lane in tl.cars_in_lane.keys():
                    for car in tl.cars_in_lane[lane]:
                        incoming_priority_pos = int(msg.body)
                        message = json.dumps({"priority_jid": sender, "position": incoming_priority_pos})
                        await self.send_message(to=car, performative="inform", ontology="traffic-management", action="warn-priority-in-road", body=message)

            elif performative == "inform" and ontology == "traffic-management" and action == "priority-entering-road":
                data = json.loads(msg.body)
                priority = data["priority"]
                spawn = data["spawn"]

                if spawn == True:
                    if tl.priority_capacity >= tl.road_capacity:
                        await self.send_message(to=sender, performative="inform", ontology="traffic-management", action="can-spawn", body="no")
                        return
                    else:
                        await self.send_message(to=sender, performative="inform", ontology="traffic-management", action="can-spawn", body="yes")

                tl.priority_capacity += 1
                tl.priorities_in_road.append(priority)

            elif performative == "inform" and ontology == "traffic-management" and action == "priority-left-road":
                # Priority vehicle has left this road
                tl.priority_capacity -= 1
                if tl.priority_capacity == 0:
                    tl.priority_incoming = False
                tl.priorities_in_road.popleft()

                for lane in range(tl.n_lanes):
                    await self.inform_priority_has_left_road(lane, sender)

                for priority_vehicle in tl.priorities_in_road:
                    await self.send_message(to=priority_vehicle, performative="inform", ontology="traffic-management", action="priority-left", body="")

            elif performative == "request" and ontology == "traffic-management" and action == "get-car-in-front-jid":
                # Car is asking if there is a car directly in front of it during priority management
                data = json.loads(msg.body)
                affected_lane = data["lane"]
                car_in_front = None
                
                if len(tl.cars_in_lane[affected_lane]) > 1:
                    temp_index = tl.cars_in_lane[affected_lane].index(sender)
                    car_in_front = tl.cars_in_lane[affected_lane][temp_index-1]
                else:
                    car_in_front = "None"

                message = json.dumps({"car_in_front": car_in_front})
                await self.send_message(to=sender, performative="inform", ontology="traffic-management", action="send-car-in-front-jid", body=message)

                if car_in_front != "None":
                    message = json.dumps({"car_from_behind": sender})
                    await self.send_message(to=car_in_front, performative="inform", ontology="traffic-management", action="send-car-from-behind-jid", body=message)

            elif performative == "cfp" and ontology == "traffic-management" and action == "asking-for-proposal":
                # Another TL is asking for a proposal (how badly this TL needs green)
                n_cars_in_road = 0
                for lane, n_cars_in_lane in tl.lane_capacities.items():
                    n_cars_in_road += n_cars_in_lane

                message = json.dumps({"tolerance": tl.tolerance, "n_cars_in_road": n_cars_in_road})
                await self.send_message(to=sender, performative="propose", ontology="traffic-management", action="send-proposal", body=message)

            elif performative == "propose" and ontology == "traffic-management" and action == "send-proposal":
                # Store received proposals to evaluate later
                proposal_data = json.loads(msg.body)
                tl.received_proposals[sender] = proposal_data

            elif performative == "accept-proposal" and ontology == "traffic-management" and action == "accept-proposal":
                # This TL won the proposal and can start a green cycle
                template = Template(metadata={"performative": "request","ontology": "traffic-management","action": "yield-green-signal"})
                tl.signal_time_behaviour = tl.SignalTimeBehaviour()
                tl.add_behaviour(tl.signal_time_behaviour, template)

            elif performative == "reject-proposal" and ontology == "traffic-management" and action == "reject-proposal":
                # Proposal rejected; nothing to do
                return
            elif performative == "inform" and ontology == "traffic-management" and action == "start-cycle":
                # External trigger to start a new cycle based on the selected method
                if tl.selection_method == "cyclic" and tl.signal_time_behaviour is None:
                    template = Template(metadata={"performative": "request","ontology": "traffic-management","action": "yield-green-signal"})
                    tl.signal_time_behaviour = tl.SignalTimeBehaviour()
                    tl.add_behaviour(tl.signal_time_behaviour, template)

            elif performative == "inform" and ontology == "traffic-management" and action == "close-road":
                data = json.loads(msg.body)
                road_to_close = data["closed_road"]
                tl.closed_roads.append(road_to_close)

            elif performative == "inform" and ontology == "traffic-management" and action == "reopen-road":
                data = json.loads(msg.body)
                road_to_open = data["reopened_road"]
                tl.closed_roads.remove(road_to_open)


        async def send_message(self, to, performative, ontology, action, body):
            """
            Helper to send a SPADE message with the given metadata and body.

            Parameters
            ----------
            to : str
                Destination agent JID.
            performative : str
                ACL performative (e.g., "inform", "request").
            ontology : str
                Ontology string, usually "traffic-management".
            action : str
                Custom action identifier in the ontology.
            body : str
                Message body payload (often JSON-encoded).
            """
            tl = self.agent 

            msg = Message(to=to, metadata={"performative": performative, "ontology": ontology, "action": action}, body=body)
            await self.send(msg)
            
            #print(f"[{tl.jid}] → Sent to {to}: ({performative}, {action}) {body}")

        async def inform_car_has_left(self, affected_lane):
            """
            Inform all cars in the given lane that another car has left it.

            This is typically used to notify cars that there is now more space
            in front of them within the same lane.
            """
            tl = self.agent

            for car in tl.cars_in_lane[affected_lane]:
                await self.send_message(to=car, performative="inform", ontology="traffic-management", action="lane-cleared", body="")

        async def inform_priority_has_left_road(self, affected_lane, priority_jid):
            """
            Inform all cars in the given lane that the priority vehicle has left this road.
            """
            tl = self.agent
            message = json.dumps({"priority_jid": priority_jid})

            for car in tl.cars_in_lane[affected_lane]:
                await self.send_message(to=car, performative="inform", ontology="traffic-management", action="priority-has-left", body=message)

    class SignalTimeBehaviour(CyclicBehaviour):
        """
        Behaviour responsible for a single complete signal cycle.

        The cycle is:
          1. Green for a computed amount of time (possibly extended due to priority).
          2. Yellow for a fixed time.
          3. Red plus clearing time.
          4. Decide which traffic light will be next (cyclic or proposals).

        It also updates metrics such as:
          - Number of cars that left during the cycle.
          - Total green time.
        """

        async def run(self):
            """
            Execute one full signal cycle and coordinate with other TLs afterwards.
            """

            tl = self.agent
            tl.metrics["n_leaving_cars_per_cycle"].append(0)
            tl.metrics["tolerance_built_up"].append(tl.tolerance)
            tl.metrics["queue_length_per_cycle"].append(list(tl.lane_capacities.values()))

            interrupt_template = Template(metadata={"performative": "request","ontology": "traffic-management","action": "yield-green-signal"})

            # GREEN
            green_light_time = self.set_green_light_time()
            tl.signal = "green"
            start_green_light_time = time.time()

            if tl.priority_incoming:
                while tl.priority_incoming:
                    await asyncio.sleep(green_light_time)
                    
            msg = await self.receive(timeout=green_light_time)

            if msg:                    
                #print(f"[{tl.jid}] Green interrupted by request from {msg.sender}!")
                total_green_light_time = time.time() - start_green_light_time
                tl.metrics["total_green_time_per_cycle"].append(total_green_light_time)

                # YELLOW
                tl.signal = "yellow"
                await asyncio.sleep(tl.yellow_light_time)

                # RED + cleaning time
                tl.signal = "red"
                await asyncio.sleep(tl.clear_intersection_time)

                await self.send_message(to=msg.sender, performative="inform", ontology="traffic-management", action="yielded-green-signal", body="")

                self.kill()

            # METRICS
            total_green_light_time = time.time() - start_green_light_time
            tl.metrics["total_green_time_per_cycle"].append(total_green_light_time)

            # YELLOW
            tl.signal = "yellow"
            await asyncio.sleep(tl.yellow_light_time)

            # RED + cleaning time
            tl.signal = "red"
            await asyncio.sleep(tl.clear_intersection_time)

            tl.metrics["total_time_lost"] += tl.yellow_light_time + tl.clear_intersection_time

            # WHO'S NEXT
            if tl.selection_method == "cyclic":
                await self._finish_cycle_cyclic()
            elif tl.selection_method == "proposals":
                await self._finish_cycle_proposals()
            else:
                # fallback: uses cyclic
                await self._finish_cycle_cyclic()

            # Reset state for the next cycle
            tl.signal_time_behaviour = None
            tl.tolerance = 0
            tl.received_proposals.clear()
            self.kill()

        async def send_message(self, to, performative, ontology, action, body):
            """
            Helper to send a SPADE message from this behaviour.

            Parameters
            ----------
            to : str
                Destination agent JID.
            performative : str
                ACL performative (e.g., "inform", "cfp").
            ontology : str
                Ontology string, usually "traffic-management".
            action : str
                Custom action identifier in the ontology.
            body : str
                Message body payload (often JSON-encoded).
            """
            tl = self.agent

            msg = Message(to=to, metadata={"performative": performative, "ontology": ontology, "action": action}, body=body)
            await self.send(msg)
            
            #print(f"[{tl.jid}] → Sent to {to}: ({performative}, {action}) {body}")

        async def _finish_cycle_cyclic(self):
            """
            Finish the cycle using the cyclic selection method.

            Either:
              - Start another cycle on this traffic light if it is the only one
                or if the cycle wraps around.
              - Or send a "start-cycle" message to the next TL in the cyclic order.
            """
            tl = self.agent

            next_tl = tl.get_next_tl_in_cycle()

            if next_tl == str(tl.jid):
                # Only TL in the intersection or cycle returned to this TL
                template = Template(metadata={"performative": "request","ontology": "traffic-management","action": "yield-green-signal"})
                tl.signal_time_behaviour = tl.SignalTimeBehaviour()
                tl.add_behaviour(tl.signal_time_behaviour, template)
            else:
                # Notify the next TL in the cycle to start its cycle
                await self.send_message(
                    to=next_tl,
                    performative="inform",
                    ontology="traffic-management",
                    action="start-cycle",
                    body=""
                )

        async def _finish_cycle_proposals(self):
            """
            Finish the cycle using the proposal-based method.

            This TL sends a CFP to all neighbouring TLs, collects proposals,
            chooses the best one, and sends accept/reject messages accordingly.
            If there are no neighbours, it starts another cycle on itself.
            """
            tl = self.agent

            if not tl.TL_group:
                # No neighbours: immediately start a new cycle on this TL
                template = Template(metadata={"performative": "request","ontology": "traffic-management","action": "yield-green-signal"})
                tl.signal_time_behaviour = tl.SignalTimeBehaviour()
                tl.add_behaviour(tl.signal_time_behaviour, template)
                return

            best_proposal = None

            # Keep asking until a best proposal is found
            while best_proposal is None:
                for neighbouring_tl in tl.TL_group:
                    await self.send_message(
                        to=neighbouring_tl,
                        performative="cfp",
                        ontology="traffic-management",
                        action="asking-for-proposal",
                        body=""
                    )

                if tl.received_proposals:
                    best_proposal = self.choose_best_proposal()

                await asyncio.sleep(0.5)

            winner_jid = best_proposal[0]

            # Send accept / reject to all neighbours
            for neighbouring_tl in tl.TL_group:
                if neighbouring_tl == winner_jid:
                    await self.send_message(
                        to=neighbouring_tl,
                        performative="accept-proposal",
                        ontology="traffic-management",
                        action="accept-proposal",
                        body=""
                    )
                else:
                    await self.send_message(
                        to=neighbouring_tl,
                        performative="reject-proposal",
                        ontology="traffic-management",
                        action="reject-proposal",
                        body=""
                    )

        def set_green_light_time(self):
            """
            Compute the duration of the green phase for this cycle.

            The time is interpolated between the configured minimum and maximum
            green time based on the current occupancy of this road.

            Returns
            -------
            float
                Duration for the green light (seconds), rounded to 1 decimal.
            """
            tl = self.agent

            minimum_time = tl.green_light_min_time
            maximum_time = tl.green_light_max_time
            green_light_time = round(minimum_time + (maximum_time - minimum_time) * (max(tl.lane_capacities.values()) / tl.road_capacity), 1)

            return green_light_time

        def choose_best_proposal(self):
            """
            Choose the best proposal among the received ones.

            Currently the best proposal is the one with the highest accumulated
            tolerance value.

            Returns
            -------
            tuple or None
                (jid, proposal_data) for the best proposal, or None if no
                proposals are available.
            """
            tl = self.agent
            if not tl.received_proposals:
                return None

            sorted_proposals = sorted(
                tl.received_proposals.items(),
                key=lambda item: item[1]["tolerance"],
                reverse=True
            )
            return sorted_proposals[0]