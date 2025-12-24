from spade.agent import Agent
from spade.behaviour import OneShotBehaviour, CyclicBehaviour, PeriodicBehaviour
from spade.template import Template
from spade.message import Message

import random
import json

class IncidentReporterAgent(Agent):
    def __init__(self, jid: str, password: str, road_close_frequency: int, road_close_probability: float, road_closed_duration: int, network: "Network"):
        super().__init__(jid, password)
        self.road_close_frequency = road_close_frequency
        self.road_close_probability = road_close_probability
        self.road_closed_duration = road_closed_duration
        self.network = network

        self.current_closed_roads = dict()

    async def setup(self):
        """
        SPADE agent setup hook.
        """
        #print(f"[{self.jid}] Traffic light for road {self.road_id} started.")
        self.add_behaviour(self.CloseRoadBehaviour(period=self.road_close_frequency))
        self.add_behaviour(self.ReopenRoadBehaviour(period=1))

    class CloseRoadBehaviour(PeriodicBehaviour):
        async def run(self):
            incident_reporter = self.agent

            if random.random() < incident_reporter.road_close_probability:
                candidate_roads = [
                    r for r in incident_reporter.network.roads.values()
                    if r.ID not in incident_reporter.current_closed_roads.keys() and not (r.startNode.type == 'spawn' or r.endNode.type == "spawn")
                ]
                if candidate_roads:
                    road_to_close = random.choice(candidate_roads).ID
                    incident_reporter.network.roads[road_to_close].closed = True
                    closed_road_timer = incident_reporter.road_closed_duration
                    incident_reporter.current_closed_roads[road_to_close] = closed_road_timer
                    for road in range(1, len(incident_reporter.network.roads) + 1):
                        receiving_tl = f"tl{road}@localhost"
                        message = json.dumps({"closed_road": road_to_close})
                        await self.send_message(to=receiving_tl, performative="inform", ontology="traffic-management", action="close-road", body=message)

        async def send_message(self, to, performative, ontology, action, body):
            incident_reporter = self.agent

            msg = Message(to=to, metadata={"performative": performative, "ontology": ontology, "action": action}, body=body)
            await self.send(msg)
            #print(f"[{incident_reporter.jid}] → Sent to {to}: ({performative}, {action}) {body}")

    class ReopenRoadBehaviour(PeriodicBehaviour):
        async def run(self):
            incident_reporter = self.agent

            for closed_road in list(incident_reporter.current_closed_roads.keys()):
                incident_reporter.current_closed_roads[closed_road] -= 1
                if incident_reporter.current_closed_roads[closed_road] == 0:
                    await self.reopen_road(closed_road)
                    incident_reporter.network.roads[closed_road].closed = False

        async def reopen_road(self, road_to_open):
            incident_reporter = self.agent

            del incident_reporter.current_closed_roads[road_to_open]
            for road in range(1, len(incident_reporter.network.roads) + 1):
                receiving_tl = f"tl{road}@localhost"
                message = json.dumps({"reopened_road": road_to_open})
                await self.send_message(to=receiving_tl, performative="inform", ontology="traffic-management", action="reopen-road", body=message)

        async def send_message(self, to, performative, ontology, action, body):
            incident_reporter = self.agent

            msg = Message(to=to, metadata={"performative": performative, "ontology": ontology, "action": action}, body=body)
            await self.send(msg)
            #print(f"[{incident_reporter.jid}] → Sent to {to}: ({performative}, {action}) {body}")

