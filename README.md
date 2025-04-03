# SMSL: State Machine Serialization Language

![MIT License](https://img.shields.io/badge/license-MIT-blue.svg)
![Version](https://img.shields.io/badge/version-0.0.1-green.svg)

SMSL is a data language designed specifically for serializing finite state machines. It provides a clean and intuitive syntax for defining state machines, with support for multiple common data formats.

## Features

- **State Machine Serialization**: Read and serialize finite state machines from/to string-based data formats
- **Multiple Format Support**: Works with SMSL, JSON, and YAML formats
- **Graph Integration**: Built-in integration with graph data structures, enabling any graph algorithm for your needs
- **Visualization Tools**: Visualize your state machines using NetworkX and matplotlib
- **Path Finding**: Find shortest paths between states in your state machine

## Installation

```bash
pip install pysmsl
```

## Usage

### Basic Usage

```python
import smsl

# Load a state machine from a file
sm = smsl.smslStateMachine('examples/hanoi.json')

# Access the state machine
state_branch = sm.state_machine[0]

# Find the shortest path between two states
from smsl_state import smslState
path = state_branch.shortest_path(
    smslState('State_aaa'),
    smslState('State_ccc')
)

# Get the operations (edges) along that path
edge_path = state_branch.shortest_edge_path(
    smslState('State_aaa'),
    smslState('State_ccc')
)

# Visualize the state machine
state_branch.plot_sb()
```

## State Machine Definition Formats

### SMSL Format

```
// SMSL format example
{
    # NAME: SB1
    # INITIAL: State000
    # NUM_FACTS: 3
    # SUB_SBS: (SB2: 0)

    State000: [Operation000 -> State100],

    State100: [
        Operation100A -> State110,
        Operation100B -> State111
    ],
        
    State110: [
        Operation110A -> State100,
        Operation110B -> State111
    ],

    State111: [ ]
}
```

### JSON Format

```json
{
    "SB1": {
        "HEADER": {
            "INITIAL": "State000",
            "NUM_FACTS": 3,
            "SUB_SBS": {
                "SB2": 0 
            }
        },
        "State000": {
            "Operation000": "State100"
        },
        "State100": {
            "Operation100A": "State110",
            "Operation100B": "State111"
        },
        "State110": {
            "Operation110A": "State100",
            "Operation110B": "State111"
        },
        "State111": { }
    }
}
```

### YAML Format

```yaml
SB1: [
  HEADER: [
    INITIAL: State000,
    NUM_FACTS: 3,
    SUB_SBS: [
      SB2: 0
    ]
  ],
  State000: [
    Operation000: State100
  ],
  State100: [
    Operation100A: State110,
    Operation100B: State111
  ],
  State110: [
    Operation110A: State100,
    Operation110B: State111
  ],
  State111: [ ]
]
```

## Examples

The repository includes several examples:

- `examples/example.smsl`, `examples/example.json`, `examples/example.yaml`: Simple state machine examples in different formats
- `examples/hanoi.json`: A state machine for solving the Tower of Hanoi puzzle

## Tutorials

### Tower of Hanoi

The repository includes a complete example of using SMSL to solve the Tower of Hanoi puzzle:

- `tutorials/hanoi/hanoi_vtk_sim/`: A VTK simulation of the Tower of Hanoi
- `tutorials/hanoi/hanoi_robot/`: A robot implementation for playing Tower of Hanoi

To run the VTK simulation:

```bash
cd tutorials/hanoi/hanoi_vtk_sim
python hanoi_vtk_sim.py
```

## Design Philosophy

SMSL is designed with the following goals in mind:

1. **Simplicity**: Easy to read and write state machine definitions
2. **Flexibility**: Support for multiple data formats (SMSL, JSON, YAML)
3. **Extensibility**: Integration with graph algorithms for advanced state machine analysis
4. **Visualization**: Tools for visualizing and understanding state machines

## Dependencies

- Python 3.6+
- `networkx`
- `matplotlib`
- `pyyaml`

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Citation

If you use SMSL in your research, please consider citing the following papers:

```bibtex
@article{liu2024roadmap,
  title={A Roadmap Towards Automated and Regulated Robotic Systems},
  author={Liu, Yihao and Armand, Mehran},
  journal={arXiv preprint arXiv:2403.14049},
  year={2024}
}

@article{liu2023toward,
  title={Toward Process Controlled Medical Robotic System},
  author={Liu, Yihao and Kheradmand, Amir and Armand, Mehran},
  journal={arXiv preprint arXiv:2308.05809},
  year={2023}
}
```
