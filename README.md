# Multi-Agent Traffic Management System

A sophisticated multi-agent traffic simulation system built with SPADE (Smart Python Agent Development Environment) that models urban traffic flow with adaptive traffic lights, intelligent routing, and priority vehicle handling.

## Overview

This project implements a comprehensive traffic management simulation using multi-agent systems. It features autonomous traffic light agents that coordinate signal timing, car agents that make routing decisions based on congestion and distance, and priority agents representing emergency vehicles. The system studies how different traffic control policies affect traffic flow, travel times, and fairness in urban road networks.

## Key Features

### Multi-Agent Architecture
- **Traffic Light Agents**: Autonomous intersection controllers with adaptive timing
- **Car Agents**: Intelligent vehicles that choose optimal routes considering congestion
- **Priority Agents**: Emergency vehicles that can request green wave priority
- **Incident Reporter Agents**: Future extension for dynamic incident reporting

### Traffic Control Policies
- **Cyclic Policy**: Fixed rotation through traffic lights (1-2-3-4 pattern)
- **Proposal-Based Policy**: Traffic lights negotiate green time based on demand
- **Adaptive Green**: Dynamic green light duration based on road occupancy
- **Priority Handling**: Emergency vehicle green wave coordination

### Intelligent Routing
- **K-Shortest Paths**: Multiple route options using Yen's algorithm
- **Congestion-Aware**: Cars balance travel distance vs. road occupancy
- **Dynamic Re-routing**: Agents can change routes based on real-time conditions

### Simulation Capabilities
- **Configurable Grid Networks**: Variable grid sizes and road capacities
- **Real-time Visualization**: Pygame-based graphical interface
- **Batch Experimentation**: Automated parameter sweeps and statistical analysis
- **Performance Metrics**: Travel times, throughput, fairness measures

## Project Structure

```
8.Multi_Agent_Traffic_Management/
└── projeto 1/                          # Main project directory
    ├── README.md                       # This file
    ├── requirements.txt                # Python dependencies
    ├── assumptions.txt                 # Project assumptions and constraints
    ├── simulation_output.txt           # Console output logs
    ├── simulation_errors.txt           # Error logs
    │
    ├── Core Modules/
    │   ├── network.py                  # Road network model and pathfinding
    │   ├── trafficLight_agent.py       # Traffic light agent logic
    │   ├── car_agent.py                # Car agent behavior and routing
    │   ├── priority_agent.py           # Emergency vehicle agent
    │   ├── incident_reporter_agent.py  # Incident reporting (stub)
    │   └── interface.py                # Pygame visualization interface
    │
    ├── Simulation Runners/
    │   ├── simulation.py               # Basic simulation runner
    │   ├── experimentation.py          # Batch experiment runner
    │   └── test.py                     # Unit testing
    │
    ├── Analysis Tools/
    │   ├── results_analysis.ipynb      # Jupyter notebook for result analysis
    │   └── graphics.ipynb              # Visualization and plotting
    │
    ├── Web Interfaces/
    │   ├── simulation.html             # Web-based simulation control
    │   ├── interface.html              # Web interface for visualization
    │   ├── network.html                # Network configuration interface
    │   ├── car_agent.html              # Car agent documentation
    │   ├── trafficLight_agent.html     # Traffic light documentation
    │   ├── priority_agent.html         # Priority agent documentation
    │   ├── incident_reporter_agent.html # Incident reporter docs
    │   └── experimentation.html        # Experiment configuration
    │
    └── simulation_results/             # Experiment output data
        ├── *.json                      # Detailed simulation data
        └── summaries/                  # Aggregated metrics
            └── *_summary.json          # Summary statistics
```

## Technical Architecture

### Agent Communication
- **SPADE Framework**: XMPP-based multi-agent communication
- **Message Types**: Structured messages for coordination
- **Real-time Updates**: Asynchronous agent interactions
- **Fault Tolerance**: Robust error handling and recovery

### Network Model
- **Grid Topology**: Configurable intersection grid
- **Lane-based Routing**: Individual lane modeling with connectivity
- **Capacity Management**: Road capacity and congestion tracking
- **Pathfinding**: Dijkstra and K-shortest paths algorithms

### Simulation Engine
- **Event-Driven**: Asynchronous simulation with timers
- **Configurable Parameters**: Extensive parameterization options
- **Data Collection**: Comprehensive metrics gathering
- **Visualization**: Real-time graphical display

## Installation and Setup

### Prerequisites
- Python 3.8+
- pip package manager

### Dependencies
```bash
pip install -r requirements.txt
```

Required packages:
- `spade==4.1.2`: Multi-agent framework
- `pygame==2.5.2`: Graphics and visualization

### Running the Simulation

#### Basic Simulation
```bash
python simulation.py
```
- Runs an infinite simulation with default parameters
- Use Ctrl+C to stop
- Includes graphical interface

#### Batch Experiments
```bash
python experimentation.py
```
- Runs multiple simulation scenarios
- Varies parameters like travel weights and capacities
- Saves results to `simulation_results/` directory

#### Custom Configuration
Modify parameters in the script files:
- Network size: `GRID = 3` (3x3 intersections)
- Road capacity: `ROAD_CAPACITY = 25` (vehicles per road)
- Car spawn rate: `SPAWN_TIMER = 0.025` (seconds)
- Number of cars: `NUM_CARS = 200`

## Usage Examples

### Running a Single Simulation
```python
from simulation import run_simulation

# Run with default parameters
asyncio.run(run_simulation())
```

### Configuring Network Parameters
```python
from network import Network

network = Network()
network.create_network(
    grid_size=3,      # 3x3 grid
    n_lanes=2,        # 2 lanes per road
    road_length=200,  # pixels
    capacity=25       # vehicles per road
)
```

### Analyzing Results
```python
import json

# Load simulation results
with open('simulation_results/sim_travel0.1_capacity0.9_cyclic_adaptiveGreenTrue_changedNetwork.json', 'r') as f:
    data = json.load(f)

# Extract metrics
travel_times = [car['total_time'] for car in data.values() if isinstance(car, dict) and 'total_time' in car]
avg_travel_time = sum(travel_times) / len(travel_times)
```

## Key Components

### Traffic Light Agent
- **Responsibilities**: Signal timing, congestion monitoring, priority coordination
- **Behaviors**: Message handling, signal cycling, demand-based decisions
- **Policies**: Cyclic, proposal-based, adaptive green duration

### Car Agent
- **Capabilities**: Route planning, congestion avoidance, real-time adaptation
- **Decision Making**: Balances distance vs. congestion using configurable weights
- **Communication**: Queries traffic lights for routes and signal status

### Priority Agent
- **Role**: Emergency vehicles requiring expedited travel
- **Functionality**: Requests green wave priority from traffic lights
- **Impact**: Modifies signal timing to create clear paths

## Experimentation Framework

### Parameter Sweeps
The system supports systematic experimentation with:
- **Travel vs. Capacity Weights**: How much cars prioritize distance vs. congestion
- **Traffic Light Policies**: Different coordination strategies
- **Network Configurations**: Changed vs. unchanged network topologies
- **Adaptive Features**: Green light adaptation on/off

### Metrics Collected
- **Travel Times**: Individual vehicle journey durations
- **Throughput**: Vehicles processed per unit time
- **Fairness**: Distribution of service across roads
- **Congestion Patterns**: Road utilization statistics
- **Priority Performance**: Emergency vehicle travel efficiency

## Analysis and Visualization

### Jupyter Notebooks
- **results_analysis.ipynb**: Statistical analysis of simulation results
- **graphics.ipynb**: Data visualization and plotting

### Key Analyses
- Travel time distributions
- Road utilization heatmaps
- Policy comparison charts
- Congestion pattern analysis
- Priority vehicle performance metrics

## Assumptions and Constraints

Based on `assumptions.txt`:
- Unlimited fuel for all vehicles
- No acceleration/deceleration modeling
- Fixed road closure durations
- Uniform road capacities
- Deterministic vehicle behavior

## Performance Considerations

### Scalability
- Grid size affects computational complexity
- Agent count impacts communication overhead
- Visualization can be disabled for headless runs

### Optimization
- Asynchronous agent execution
- Efficient pathfinding algorithms
- Minimal message passing

## Future Extensions

### Planned Features
- **Incident Reporting**: Dynamic road closure handling
- **Machine Learning**: Adaptive traffic light policies
- **Multi-modal Traffic**: Integration of public transport
- **Weather Effects**: Environmental impact modeling
- **Real-time Adaptation**: Learning from historical data

### Research Directions
- **Reinforcement Learning**: Traffic light optimization
- **Game Theory**: Intersection negotiation strategies
- **Network Theory**: Urban traffic flow optimization
- **Sustainability**: Environmental impact assessment

## Academic Context

This project demonstrates advanced concepts in:
- **Multi-Agent Systems**: Distributed coordination and decision-making
- **Traffic Engineering**: Urban mobility optimization
- **Artificial Intelligence**: Intelligent routing and control
- **Simulation Modeling**: Complex system analysis

## References

- **SPADE Framework**: Smart Python Agent Development Environment
- **Traffic Light Coordination**: Adaptive signal control literature
- **Multi-Agent Traffic Simulation**: Related research in MAS for transportation
- **Pathfinding Algorithms**: Dijkstra, Yen K-shortest paths

## Contributing

This is a research project. Contributions welcome for:
- Algorithm improvements
- Additional agent types
- New traffic policies
- Performance optimizations
- Documentation enhancements

## License

Academic/research use. Please cite appropriately if used in publications.