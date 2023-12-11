# State Machine Serialization Language

SMSL is a data language to serialize finite state machine.

## Example

An example script can be seen in [hanoi.py](), a finite state machine for automatically playing Hanoi Tower.

## Design Goals

* *State machine serialization*. The main goal of SMSL is to read / serialize finite state machines from / to string-based data. The SMSL is a new data language similar to JSON that is designed specifically for state machines. The format of the data language can be seen in [example.smsl]()
* *Graph integration*. The data loader integrates graph data structure so any graph algorithm is possible for your needs.
* *Common data language support*. The package supports JSON and YAML. Examples of how to write the state machine can be seen in [example.json]() and [example.yaml]().

## Installation

````
pip install pysmsl
````

## Citation

Please consider to cite the following paper:

Liu, Y., Kheradmand, A., & Armand, M. (2023). Toward Process Controlled Medical Robotic System. arXiv preprint arXiv:2308.05809.

````
@article{liu2023toward,
  title={Toward Process Controlled Medical Robotic System},
  author={Liu, Yihao and Kheradmand, Amir and Armand, Mehran},
  journal={arXiv preprint arXiv:2308.05809},
  year={2023}
}
````