# GODOT-UAV-SIMULATOR-v2

Godot-based UAV (Unmanned Aerial Vehicle) Simulator for drone flight planning and route optimization.

## Project Overview

This simulator provides a 3D visualization environment for UAV flight planning, featuring:
- GridMap terrain system with 3D elevation mapping
- WebSocket capability for real-time communication
- Drone port visualization
- Camera navigation with coordinate conversion
- Route generation using Cooperative A* algorithm
- Flight plan management and logging

## Features

- **Terrain System**: 3D GridMap-based terrain visualization
- **Drone Management**: Support for multiple drone models and specifications
- **Flight Planning**: CSV-based flight plan import and management
- **Real-time Logging**: Collision detection, distance tracking, and simulation logging
- **WebSocket Integration**: Python-Godot communication for route generation
- **Visualization**: 3D route visualization with coordinate mapping

## Project Structure

```
├── data/                          # Flight plan data and CSV files
├── resources/                     # 3D models and mesh libraries
├── scenes/                        # Godot scene files
│   ├── drones/                   # Drone-specific scenes
│   ├── GridMap/                  # Terrain GridMap scene
│   ├── main/                     # Main simulation scene
│   └── ui/                       # User interface scenes
├── scripts/                       # GDScript and Python scripts
│   ├── core/                     # Core simulation systems
│   ├── drone/                    # Drone behavior scripts
│   ├── Python/                   # Python route generation tools
│   └── ui/                       # UI controller scripts
└── logs/                          # Simulation output logs
```

## Requirements

### Godot Engine
- Godot 4.x or higher

### Python Dependencies
- Python 3.x
- See individual Python script directories for specific requirements

## Getting Started

1. Clone this repository
2. Open the project in Godot Engine 4.x
3. Run the main scene (`scenes/main/main.tscn`)

## Documentation

- `TERRAIN_SYSTEM_README.md` - Detailed terrain system documentation
- `PROJECT_FLOWCHART.txt` - Project flow and architecture overview
- `drone_models_specifications.txt` - Drone model specifications

## License

[Add your license information here]

## Contributors

[Add contributor information here]

