"""
Network model for the multi-agent traffic simulation.

This module defines the basic infrastructure of the traffic network:
  - Network: container for nodes, roads, and lanes, and utilities to build
    a grid and compute routes.
  - Node: intersection or spawn point in the grid.
  - Road: directed connection between two nodes, containing one or more lanes.
  - Lane: atomic unit for vehicle movement, with inflow and outflow connections.

It also provides:
  - Dijkstra-based shortest path computation on lanes.
  - Yen's K-shortest loopless path algorithm on lanes.
"""

from scipy.spatial import distance
import random

import heapq
from collections import deque

class Network:
    """
    Representation of the entire road network.

    Attributes
    ----------
    nodes : dict[int, Node]
        Mapping from node ID to Node instance.
    roads : dict[int, Road]
        Mapping from road ID to Road instance.
    lanes : dict[int, Lane]
        Mapping from lane UID to Lane instance.
    next_lane_uid : int
        Global counter used to assign unique lane UIDs.
    spawn_nodes : list[Node]
        List of spawn nodes around the border of the grid.
    """

    def __init__(self):
        """Initialize an empty network with no nodes, roads or lanes."""
        self.nodes = {}  # nodeID -> Node
        self.roads = {}  # roadID -> Road
        self.lanes = {}  # laneID -> Lane
        self.closed_roads = []
        self.next_lane_uid = 0

        self.spawn_nodes = []

    def create_network(self, grid_size, lane_count, spacing_between_nodes, road_capacity_for_road):
        """
        Create a grid-like road network and corresponding spawn nodes.

        The network is created as a grid of intersections (traffic light nodes)
        connected by bidirectional roads. Around the border of the grid, extra
        spawn nodes are added and connected to the border intersections, which
        serve as entry/exit points for vehicles.

        Parameters
        ----------
        grid_size : int
            Number of nodes per side in the grid (grid_size x grid_size).
        lane_count : int
            Number of lanes per road (per direction).
        spacing_between_nodes : int or float
            Distance between consecutive grid nodes (used for positioning).
        road_capacity_for_road : int
            Logical capacity/length of each road (used as lane length).
        """
        spacing = spacing_between_nodes
        road_capacity = road_capacity_for_road
        grid_size = grid_size
        lane_count = lane_count
        node_id = 0
        road_id = 0

        # Create grid nodes (traffic light intersections)
        for i in range(grid_size):
            for j in range(grid_size):
                node_id += 1
                x = -spacing + j * spacing
                y = spacing - i * spacing
                self.nodes[node_id] = Node(self, node_id, [x, y], "traffic_light", spacing)

        # Create bidirectional roads inside the grid
        for i in range(grid_size):
            for j in range(grid_size):
                node_index = i * grid_size + j + 1

                # Horizontal roads
                if j < grid_size - 1:
                    right_node = node_index + 1
                    # Two directions: node_index → right_node and right_node → node_index
                    for direction in [(self.nodes[node_index], self.nodes[right_node]),
                                      (self.nodes[right_node], self.nodes[node_index])]:
                        road_id += 1
                        e = Road(road_id, lane_count, road_capacity, *direction)
                        self.roads[road_id] = e
                        for lane in e.lanes.values():
                            self.lanes[lane.UID] = lane

                # Vertical roads
                if i < grid_size - 1:
                    below_node = node_index + grid_size
                    # Two directions: node_index → below_node and below_node → node_index
                    for direction in [(self.nodes[node_index], self.nodes[below_node]),
                                      (self.nodes[below_node], self.nodes[node_index])]:
                        road_id += 1
                        e = Road(road_id, lane_count, road_capacity, *direction)
                        self.roads[road_id] = e
                        for lane in e.lanes.values():
                            self.lanes[lane.UID] = lane

        # Create spawn nodes around the border 
        all_x = [node.pos[0] for node in self.nodes.values()]
        all_y = [node.pos[1] for node in self.nodes.values()]
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)

        spawn_offset = spacing  # how far spawn nodes sit from the grid

        # Add spawners around each outer node
        for node in list(self.nodes.values()):
            x, y = node.pos
            is_left = x == min_x
            is_right = x == max_x
            is_top = y == max_y
            is_bottom = y == min_y

            # Left border
            if is_left:
                node_id += 1
                spawn = Node(self, node_id, [x - spawn_offset, y], "spawn", spacing)
                self.spawn_nodes.append((spawn, node))
                self.nodes[node_id] = spawn

            # Right border
            if is_right:
                node_id += 1
                spawn = Node(self, node_id, [x + spawn_offset, y], "spawn", spacing)
                self.spawn_nodes.append((spawn, node))
                self.nodes[node_id] = spawn

            # Top border
            if is_top:
                node_id += 1
                spawn = Node(self, node_id, [x, y + spawn_offset], "spawn", spacing)
                self.spawn_nodes.append((spawn, node))
                self.nodes[node_id] = spawn

            # Bottom border
            if is_bottom:
                node_id += 1
                spawn = Node(self, node_id, [x, y - spawn_offset], "spawn", spacing)
                self.spawn_nodes.append((spawn, node))
                self.nodes[node_id] = spawn

        # Create roads connecting spawn nodes to their corresponding border nodes
        for spawn, main_node in self.spawn_nodes:
            for direction in [(spawn, main_node), (main_node, spawn)]:
                road_id += 1
                e = Road(road_id, lane_count, road_capacity, *direction)
                self.roads[road_id] = e
                for lane in e.lanes.values():
                    self.lanes[lane.UID] = lane

        # Keep only the spawn Node objects in spawn_nodes (drop the pair)
        self.spawn_nodes = [node[0] for node in self.spawn_nodes]

        # Build road connectivity and per-lane connections
        self.update_flow_roads()
        self.make_lane_connections()

        print(f"Grid built with {len(self.nodes)} nodes, {len(self.roads)} roads.")

    def update_flow_roads(self):
        """
        Update inflow and outflow road lists for each road.

        For each road:
          - inFlowRoads: all roads entering the start node of this road.
          - outFlowRoads: all roads leaving the end node of this road.
          - parallelRoad: a road that shares start and end nodes (opposite direction).
        """
        for road in self.roads.values():
            # All roads that go into the start node become inflow roads
            for inFlowRoad in road.startNode.inRoads:
                road.inFlowRoads.append(inFlowRoad)

            # All roads that go out of the end node become outflow roads
            for outFlowRoad in road.endNode.outRoads:
                road.outFlowRoads.append(outFlowRoad)

            # Parallel road is the road that is both inflow and outflow at this node pair
            common_road = [e for e in road.inFlowRoads if e in road.outFlowRoads]
            road.parallelRoad = common_road[0]

    def make_lane_connections(self):
        """
        Build per-lane inflow and outflow connections based on road directions.

        This method populates for each lane:
          - inFlowLanes: lanes from which vehicles can enter this lane.
          - outFlowLanes: lanes to which vehicles can proceed from this lane.

        The connection pattern depends on the relative orientation of the roads
        (straight, left turn, right turn, or opposite direction).
        """
        for road in self.roads.values():
            # Incoming lanes to this road
            for inFlowRoad in road.inFlowRoads:
                if road.vector_direction == inFlowRoad.vector_direction:
                    # Same direction: merge both lanes from inFlowRoad into both lanes of this road
                    road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[0])
                    road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[0])
                elif inFlowRoad.vector_direction == (0,-1):
                    if road.vector_direction == (-1,0):
                        # From bottom to left turn
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[0])
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[0])
                    elif road.vector_direction == (1,0):
                        # From bottom to right turn
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[1])
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[1])
                elif inFlowRoad.vector_direction == (0,1):
                    if road.vector_direction == (1,0):
                        # From top to right turn
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[0])
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[0])
                    elif road.vector_direction == (-1,0):
                        # From top to left turn
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[1])
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[1])
                elif inFlowRoad.vector_direction == (-1,0):
                    if road.vector_direction == (0,1):
                        # From left to up
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[0])
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[1])
                    elif road.vector_direction == (0,-1):
                        # From left to down
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[1])
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[0])
                elif inFlowRoad.vector_direction == (1,0):
                    if road.vector_direction == (0,-1):
                        # From right to down
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[0])
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[1])
                    elif road.vector_direction == (0,1):
                        # From right to up
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[1])
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[0])

                # Opposite direction: road is parallel and can receive from lane 1 of inFlowRoad
                if abs(inFlowRoad.vector_direction[0] - road.vector_direction[0]) == 2 or abs(inFlowRoad.vector_direction[1] - road.vector_direction[1]) == 2:
                    road.parallelRoad = inFlowRoad
                    road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[1]) 
                    road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[1]) 

            # Outgoing lanes from this road
            for outFlowRoad in road.outFlowRoads:
                if road.vector_direction == outFlowRoad.vector_direction:
                    # Straight ahead: from lane 0 to both lanes of outFlowRoad
                    road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[0])
                    road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[1])
                elif outFlowRoad.vector_direction == (0,-1):
                    if road.vector_direction == (-1,0):
                        # From left to down
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[1])
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[0])
                    elif road.vector_direction == (1,0):
                        # From right to down
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[0])
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[1])
                elif outFlowRoad.vector_direction == (0,1):
                    if road.vector_direction == (1,0):
                        # From right to up
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[1])
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[0])
                    elif road.vector_direction == (-1,0):
                        # From left to up
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[0])
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[1])
                elif outFlowRoad.vector_direction == (-1,0):
                    if road.vector_direction == (0,1):
                        # From up to left
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[1])
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[0])
                    elif road.vector_direction == (0,-1):
                        # From down to left
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[0])
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[1])
                elif outFlowRoad.vector_direction == (1,0):
                    if road.vector_direction == (0,-1):
                        # From down to right
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[1])
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[0])
                    elif road.vector_direction == (0,1):
                        # From up to right
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[0])
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[1])

                # Opposite direction: road is parallel and links to outFlowRoad's lanes
                if abs(outFlowRoad.vector_direction[0] - road.vector_direction[0]) == 2 or abs(outFlowRoad.vector_direction[1] - road.vector_direction[1]) == 2:
                    road.parallelRoad = outFlowRoad
                    road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[1])
                    road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[0])

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
        startRoad = self.roads[start_road_ID]
        startLane = self.lanes[start_lane_UID]

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
                road = self.roads[road_id]

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
        start_lane_UID = self.roads[start_road_ID].lanes[start_lane_ID].UID

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
                        lane = self.lanes[banned_lane_UID]
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
                    new_full_cost = sum(self.lanes[laneUID].length for _, laneUID in new_full_path) - self.roads[new_full_path[0][0]].length

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
                A[j][0][i][1] = self.lanes[A[j][0][i][1]].ID
                A[j][0][i] = tuple(A[j][0][i])
            A[j][0].popleft()
            A[j] = tuple(A[j])
        return A


class Node:
    """
    Node in the road network.

    Represents either:
      - A traffic light intersection inside the grid, or
      - A spawn node on the border for cars entering/leaving the grid.

    Attributes
    ----------
    network : Network
        Reference to the owning network.
    ID : int
        Unique identifier for the node.
    pos : list[int, int]
        Position coordinates [x, y].
    type : str
        Type of node, e.g. "traffic_light" or "spawn".
    neighbourNodes : list[Node]
        Adjacent nodes connected by a road.
    inRoads : list[Road]
        Roads ending at this node.
    outRoads : list[Road]
        Roads starting at this node.
    spacing : int or float
        Grid spacing used for directional vector calculations.
    """

    def __init__(self, network, ID: int, pos: list[int, int], type_of_intersection: str, spacing):
        """
        Initialize a Node.

        Parameters
        ----------
        network : Network
            Parent network containing this node.
        ID : int
            Unique node identifier.
        pos : list[int, int]
            [x, y] coordinates of the node.
        type_of_intersection : str
            Node type (e.g., "traffic_light", "spawn").
        spacing : int or float
            Grid spacing value used by roads to compute direction vectors.
        """
        self.network = network
        self.ID = ID
        self.pos = pos  # (x, y)
        self.type = type_of_intersection  # e.g. tl=traffic_light, s=stop, etc.
        self.neighbourNodes = []
        self.inRoads = []
        self.outRoads = []
        self.spacing = spacing


class Road:
    """
    Directed road connecting two nodes in the network.

    Each road has one or more lanes, a fixed length (capacity) and a
    direction vector derived from the start and end node positions.

    Attributes
    ----------
    ID : int
        Unique road identifier.
    laneCount : int
        Number of lanes on this road.
    lanes : dict[int, Lane]
        Mapping from lane index to Lane object.
    startNode : Node
        Node where this road begins.
    endNode : Node
        Node where this road ends.
    nodeSet : list[int]
        Sorted list of [startNode.ID, endNode.ID].
    parallelRoad : Road or None
        Road that has the same two nodes but opposite direction.
    length : int or float
        Logical length/capacity for all lanes in this road.
    angle_sin : float or None
        Reserved for geometric calculations (not used here).
    angle_cos : float or None
        Reserved for geometric calculations (not used here).
    vector_direction : tuple[float, float] or None
        Normalized direction vector from startNode to endNode.
    inFlowRoads : list[Road]
        Roads that feed into this road at the start node.
    outFlowRoads : list[Road]
        Roads reachable from this road at the end node.
    """

    def __init__(self, ID: int, laneCount: int, road_capacity, startNode: "Node", endNode: "Node"):
        """
        Initialize a Road.

        Parameters
        ----------
        ID : int
            Unique road identifier.
        laneCount : int
            Number of lanes in this road.
        road_capacity : int or float
            Logical capacity/length of the road.
        startNode : Node
            Origin node.
        endNode : Node
            Destination node.
        """
        self.ID = ID
        self.laneCount = laneCount
        self.lanes = {}
        self.startNode = startNode
        self.endNode = endNode
        self.nodeSet = sorted([startNode.ID, endNode.ID])
        self.parallelRoad = None
        self.length = road_capacity
        self.angle_sin = None
        self.angle_cos = None
        self.vector_direction = None
        self.inFlowRoads = []
        self.outFlowRoads = []

        # NEW
        self.closed = False
        self._saved_inflows = {}   # para guardar outflows removidos

        self.init_properties()

    # Lane connections between roads
    def init_properties(self):
        """
        Initialize road properties and create its lanes.

        This method:
          - Registers the road in the start and end nodes.
          - Computes the direction vector based on node positions.
          - Creates the lane objects for this road.
        """
        self.network = self.startNode.network
    
        # Update node properties
        self.startNode.outRoads.append(self)
        self.endNode.inRoads.append(self)
        self.startNode.neighbourNodes.append(self.endNode)

        # Road direction vector (normalized using grid spacing)
        self.vector_direction = (
            (self.endNode.pos[0] - self.startNode.pos[0]) / self.startNode.spacing,
            (self.endNode.pos[1] - self.startNode.pos[1]) / self.startNode.spacing
        )

        # Create lanes for this road
        for i in range(self.laneCount):
            laneID = i
            self.lanes[i] = Lane(laneID, self.network.next_lane_uid, self)
            self.network.next_lane_uid += 1

class Lane:
    """
    Single lane on a Road.

    Lanes are the fundamental unit for vehicle movement and routing. Each lane
    has inflow and outflow connections to other lanes, forming a lane-level
    graph used by the routing algorithms.

    Attributes
    ----------
    ID : int
        Local lane index within its road.
    UID : int
        Global unique identifier across the entire network.
    road : Road
        Parent road.
    length : int or float
        Length of the lane (same as road length).
    parallelLanes : list[Lane]
        Other lanes in the same road.
    vehicles : collections.deque
        Queue of vehicles currently on this lane (if used).
    inFlowLanes : list[Lane]
        Lanes from which a vehicle can enter this lane.
    outFlowLanes : list[Lane]
        Lanes a vehicle can move to from this lane.
    """

    def __init__(self, ID: int, UID: int, road: "Road"):
        """
        Initialize a Lane.

        Parameters
        ----------
        ID : int
            Local lane index in the parent road.
        UID : int
            Globally unique lane identifier.
        road : Road
            Reference to the parent road.
        """
        self.ID = ID
        self.UID = UID
        self.road = road
        self.length = self.road.length
        self.parallelLanes = []
        self.vehicles = deque()
        self.inFlowLanes = []
        self.outFlowLanes = []

    def init_properties(self):
        """
        Initialize the list of parallel lanes within the same road.

        This collects all other lanes of the parent road and stores them
        in `parallelLanes`. It is not strictly required for the routing
        logic but can be useful for lane-changing models.
        """
        for i in self.road.lanes.keys():
            laneID = self.road.lanes[i].ID
            if laneID != self.ID:
                self.parallelLanes.append(self.road.lanes[i])


if __name__ == "__main__":
    network = Network()
    network.create_network(3,2,200,25)

    for node in network.nodes.values():
        print(f"{node.ID}: {node.pos}")

    for roadid, road in network.roads.items():
        print(f"{roadid}: {road.startNode.ID} {road.endNode.ID}")