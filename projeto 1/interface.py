"""
Pygame-based visualization for the traffic simulation.

This module is responsible for:
  - Drawing the static road network (nodes, roads, lanes).
  - Rendering dynamic elements such as traffic lights and vehicles.
  - Running a main Pygame loop that continuously updates the screen.

It does not implement any simulation logic itself; it only reads
the current state from the Network and agent objects and displays it.
"""

import pygame
import time
import random
import threading
import traceback
from network import Node, Road, Network
from trafficLight_agent import TrafficLightAgent

# Cores do Pygame
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
YELLOW = (255,255,0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GRAY = (128, 128, 128)
PURPLE = (128, 0, 128)

# Configurações da tela
SCREEN_WIDTH = 750
SCREEN_HEIGHT = 750
FPS = 10

pygame.font.init()
font = pygame.font.SysFont('Arial', 14)

def centralize_position(network):
    """Calcular o centro da rede e retornar os valores WC e HC."""
    all_x = [node.pos[0] for node in network.nodes.values()]
    all_y = [node.pos[1] for node in network.nodes.values()]
    
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)
    
    # Calculando o centro da rede (para centralizar no Pygame)
    center_x = (max_x + min_x) / 2
    center_y = (max_y + min_y) / 2
    
    # Calculando o deslocamento necessário para centralizar na tela
    WC = SCREEN_WIDTH / 2 - center_x
    HC = SCREEN_HEIGHT / 2 + center_y
    
    return WC, HC

# Static world-center offsets for drawing
WC = 375
HC = 375


def draw_map(screen, network):
    """
    Draw the static map (nodes and roads/lanes) onto the given surface.

    This function:
      - Draws intersections and spawn nodes.
      - Draws each lane as a line segment between two nodes, slightly
        offset depending on the road direction so that lanes are separated.
      - Labels each node and lane with its ID.

    Parameters
    ----------
    screen : pygame.Surface
        Surface where the map should be rendered. Typically a background surface.
    network : Network
        Network instance containing nodes, roads and lanes to be drawn.

    Returns
    -------
    dict
        Dictionary mapping (road_id, lane_index) to a tuple
        ((x_start, y_start), (x_end, y_end)) representing the lane geometry
        in screen coordinates. This mapping is later used to position cars.
    """
    WC, HC = centralize_position(network)  # Calcular a centralização após criar a rede

    lanes = {}

    # Desenho dos nós
    for node in network.nodes.values():
        node_pos = (WC + node.pos[0], HC - node.pos[1])
        
        if node.type == "traffic_light":
            # Tamanho do "cruzamento"
            rect_width, rect_height = 50, 50
            rect_x = node_pos[0] - rect_width // 2
            rect_y = node_pos[1] - rect_height // 2
            pygame.draw.rect(screen, GRAY, (rect_x, rect_y, rect_width, rect_height))

        elif node.type == "spawn":
            pygame.draw.circle(screen, PURPLE, node_pos, 10)
        
        #Escrever o ID do nó
        text_surface = pygame.font.SysFont('Arial', 14).render(str(node.ID), True, BLACK)
        text_rect = text_surface.get_rect(center=(node_pos[0], node_pos[1] - 20))
        screen.blit(text_surface, text_rect)

    # Desenho das estradas com lanes
    for road in network.roads.values():
        vetor = road.vector_direction
        flag = False  # Flag para verificar se a primeira lane já foi desenhada
        
        for lane in road.lanes.keys():
            start_node = road.startNode
            end_node = road.endNode
            
            # Ajuste da posição dependendo da direção
            if vetor == (1, 0):  # Direção para a direita
                if flag:
                    start_line = (WC + start_node.pos[0] + 25*(vetor[0]), HC - start_node.pos[1] + 25*(vetor[1]) + 5)
                    end_line = (WC + end_node.pos[0] - 25*(vetor[0]), HC - end_node.pos[1] - 25*(vetor[1]) + 5)
                else:
                    start_line = (WC + start_node.pos[0] + 25*(vetor[0]), HC - start_node.pos[1] + 25*(vetor[1]) + 25)
                    end_line = (WC + end_node.pos[0] - 25*(vetor[0]), HC - end_node.pos[1] - 25*(vetor[1]) + 25)
                flag = True
                
            elif vetor == (-1, 0):  # Direção para a esquerda
                if flag:
                    start_line = (WC + start_node.pos[0] + 25*(vetor[0]), HC - start_node.pos[1] + 25*(vetor[1]) - 5)
                    end_line = (WC + end_node.pos[0] - 25*(vetor[0]), HC - end_node.pos[1] - 25*(vetor[1]) - 5)
                else:
                    start_line = (WC + start_node.pos[0] + 25*(vetor[0]), HC - start_node.pos[1] + 25*(vetor[1]) - 25)
                    end_line = (WC + end_node.pos[0] - 25*(vetor[0]), HC - end_node.pos[1] - 25*(vetor[1]) - 25)
                flag = True
                
            elif vetor == (0, 1):  # Direção para cima (cartesiana)
                if flag:
                    start_line = (WC + start_node.pos[0] + 25*(vetor[0]) + 5, HC - (start_node.pos[1] + 25*(vetor[1])))
                    end_line = (WC + end_node.pos[0] - 25*(vetor[0]) + 5, HC - (end_node.pos[1] - 25*(vetor[1])))
                else:
                    start_line = (WC + start_node.pos[0] + 25*(vetor[0]) + 25, HC - (start_node.pos[1] + 25*(vetor[1])))
                    end_line = (WC + end_node.pos[0] - 25*(vetor[0]) + 25, HC - (end_node.pos[1] - 25*(vetor[1])))
                flag = True
                
            elif vetor == (0, -1):  # Direção para baixo (cartesiana)
                if flag:
                    start_line = (WC + start_node.pos[0] + 25*(vetor[0]) - 5, HC - (start_node.pos[1] + 25*(vetor[1])))
                    end_line = (WC + end_node.pos[0] - 25*(vetor[0]) - 5, HC - (end_node.pos[1] - 25*(vetor[1])))
                else:
                    start_line = (WC + start_node.pos[0] + 25*(vetor[0]) - 25, HC - (start_node.pos[1] + 25*(vetor[1])))
                    end_line = (WC + end_node.pos[0] - 25*(vetor[0]) - 25, HC - (end_node.pos[1] - 25*(vetor[1])))
                flag = True

            lanes[(road.ID,lane)] = (start_line,end_line)

            # Desenha a linha da lane
            pygame.draw.line(screen, BLACK, start_line, end_line, 3)

            # # --- Desenhar o ID da lane no centro ---
            font = pygame.font.SysFont(None, 18)
            lane_id = f"{road.ID}-{lane}"

            # Calcular posição central da linha
            mid_x = (start_line[0] + end_line[0]) / 2
            mid_y = (start_line[1] + end_line[1]) / 2

            text_surface = font.render(lane_id, True, (0, 0, 0))
            text_rect = text_surface.get_rect(center=(mid_x, mid_y))
            screen.blit(text_surface, text_rect)
    
    return lanes


def draw_network(screen, network, lanes, traffic_lights, cars):
    """
    Draw the dynamic state of the network (traffic lights and cars).

    Parameters
    ----------
    screen : pygame.Surface
        Surface where the elements should be drawn.
    network : Network
        Network instance that provides geometry and road information.
    lanes : dict
        Mapping (road_id, lane_index) -> ((x_start, y_start), (x_end, y_end))
        as returned by draw_map.
    traffic_lights : dict[int, TrafficLightAgent]
        Dictionary mapping road IDs to their corresponding TrafficLightAgent.
    cars : iterable
        Collection (typically a set) of car-like agents with attributes:
          - current_road
          - current_lane
          - position
          - is_priority
          - pulled_over (for non-priority cars)
    """
    # Calcular a centralização após criar a rede
    WC, HC = centralize_position(network)  # Calcular a centralização após criar a rede

    # Desenho dos semáforos
    for road_id in traffic_lights:
        traffic_light = traffic_lights[road_id]
        signal_color = {
                    "red": RED,
                    "yellow": YELLOW,
                    "green": GREEN
                }.get(traffic_light.signal, RED)
        if traffic_light.network.roads[traffic_light.road_id].endNode.type == "traffic_light":      # Embora existam na teoria semáforos nos node spawn points, como vão estar sempre verdes, não os desenhamos
            traffic_light_x = traffic_light.network.roads[traffic_light.road_id].endNode.pos[0]
            traffic_light_y = traffic_light.network.roads[traffic_light.road_id].endNode.pos[1]
            direction = traffic_light.network.roads[traffic_light.road_id].vector_direction
            if direction == (1,0):
                pygame.draw.circle(screen, signal_color, (WC + traffic_light_x - 25, HC - traffic_light_y + 15), 5)
            elif direction == (0,1):
                pygame.draw.circle(screen, signal_color, (WC + traffic_light_x + 15, HC - traffic_light_y + 25), 5)
            elif direction == (-1,0):
                pygame.draw.circle(screen, signal_color, (WC + traffic_light_x + 25, HC - traffic_light_y - 15), 5)
            elif direction == (0,-1):
                pygame.draw.circle(screen, signal_color, (WC + traffic_light_x - 15, HC - traffic_light_y - 25), 5)

    # --- Desenhar roads fechadas por cima das lanes ---
    for road_id, road in network.roads.items():
        if road.closed:
            # Para cada lane dessa estrada, desenhar uma linha vermelha grossa por cima
            for lane_index in road.lanes.keys():
                if (road_id, lane_index) in lanes:
                    start_line, end_line = lanes[(road_id, lane_index)]
                    pygame.draw.line(screen, (255, 0, 0), start_line, end_line, 6)


    # Desenho dos carros
    for car in list(cars):
        road = network.roads[car.current_road]
        direction = road.vector_direction  # importante: direção da estrada
        if car.is_priority:
            # Pega as duas lanes reais da road
            (start0, end0) = lanes[(car.current_road, 0)]
            (start1, end1) = lanes[(car.current_road, 1)]

            # Calcula a lane central onde o prioritário deve andar
            start_line = ((start0[0] + start1[0]) / 2, (start0[1] + start1[1]) / 2)
            end_line   = ((end0[0] + end1[0]) / 2, (end0[1] + end1[1]) / 2)
        else:
            start_line, end_line = lanes[(car.current_road, car.current_lane)]

        # Vetor da lane normalizado (tamanho 1)
        dx = end_line[0] - start_line[0]
        dy = end_line[1] - start_line[1]
        length = road.length
        spaces = (dx / length, dy / length)

        # Posição real do carro ao longo da estrada
        car_x = start_line[0] + car.position * spaces[0]
        car_y = start_line[1] + car.position * spaces[1]

        # Desenho do triângulo conforme direção
        if direction == (1, 0):  # Direita
            triangle = [
                (car_x - 10, car_y - 5),
                (car_x - 10, car_y + 5),
                (car_x, car_y)
            ]
        elif direction == (-1, 0):  # Esquerda
            triangle = [
                (car_x + 10, car_y - 5),
                (car_x + 10, car_y + 5),
                (car_x, car_y)
            ]
        elif direction == (0, 1):  # Cima (eixo cartesiano invertido no Pygame)
            triangle = [
                (car_x - 5, car_y + 10),
                (car_x + 5, car_y + 10),
                (car_x, car_y)
            ]
        elif direction == (0, -1):  # Baixo (eixo cartesiano invertido no Pygame)
            triangle = [
                (car_x - 5, car_y - 10),
                (car_x + 5, car_y - 10),
                (car_x, car_y)
            ]
        else:
            triangle = [(car_x, car_y)]  # fallback

        # Color coding:
        #   - Non-priority + pulled_over: yellow
        #   - Non-priority + normal: blue
        #   - Priority: red
        if not car.is_priority and car.pulled_over:
            pygame.draw.polygon(screen, YELLOW, triangle)
        elif not car.is_priority and not car.pulled_over:
            pygame.draw.polygon(screen, BLUE, triangle)
        elif car.is_priority:
            pygame.draw.polygon(screen, RED, triangle)


# Função para rodar a simulação
def game_loop(network, traffic_lights, cars, stop_event):
    """
    Main Pygame loop for the visualization.

    This loop:
      - Initializes Pygame and the main window.
      - Draws the static background (network layout) once.
      - In each frame:
          * Blits the static background.
          * Draws traffic lights and vehicles on top.
          * Processes quit events.
          * Updates the display at a fixed FPS.

    Parameters
    ----------
    network : Network
        Network instance with the nodes and roads to be visualized.
    traffic_lights : dict[int, TrafficLightAgent]
        Dictionary of traffic light agents keyed by road_id.
    cars : iterable
        Collection (set/list) of car and priority agents to be drawn.
    """
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Traffic Simulation")
    clock = pygame.time.Clock()

    running = True

    # Desenha o mapa base numa superfície separada (fundo fixo)
    background = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
    background.fill(WHITE)
    lanes = draw_map(background, network)  # desenha o mapa no fundo fixo

    while running and not stop_event.is_set():
        try:
            # Limpa a tela com o fundo original (SEM desenhar o mapa de novo)
            screen.blit(background, (0, 0))

            # Desenha carros e semáforos por cima
            draw_network(screen, network, lanes, traffic_lights, cars)

            # Eventos de saída
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Atualiza o ecrã
            pygame.display.flip()
            clock.tick(FPS)

        except Exception as e:
            print("[INTERFACE ERROR]", e)
            traceback.print_exc()
            running = False

    pygame.quit()
    print("[INTERFACE] Finalizada com segurança.")