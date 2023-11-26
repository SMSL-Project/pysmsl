import json, yaml, smsl_json, smsl_yaml
import networkx as nx
import matplotlib.pyplot as plt

from pathlib import Path
from smsl_errors import (
    smslFileNotSupportedError,
    smslDataStructNotSupportedError
)
from smsl_constants import (
    smslDict_SB_Statedigit
)

class smslStateMachine:
    """
    State Machine class
    """
    def __init__(self):
        """
        Empty class
        """
        self.state_machine = None
        self._post_process()
    
    def __init__(self, file_path : Path):
        """
        Initialize State Machine by a file
        """
        file_data = None
        if file_path.suffix == '.json':
            with open(file_path, 'r') as file:
                file_data = {"TYPE" : "JSON", "DATA" : json.load(file)}
        elif file_path.suffix == '.yaml':
            with open(file_path, 'r') as file:
                file_data = {"TYPE" : "YAML", "DATA" :yaml.safe_load(file)}
        # TODO finish smsl parser
        # elif file_path.suffix == '.smsl':
        #     with open(file_path, 'r') as file:
        #         file_data = {"TYPE" : "SMSL", "DATA" : smsl.load(file)}
        else:
            raise smslFileNotSupportedError(f"{file_path.suffix} files are not supported.")
        self._post_process(file_data)
         
    def _post_process(self, file_data):
        d_type, d_data = file_data["TYPE"], file_data["DATA"]
        if d_type == "JSON":
            smsl_json.process(d_data)
        elif d_type == "YAML":
            smsl_yaml.process(d_data)
        # TODO finish smsl parser
        # elif file_data["TYPE"] == "SMSL":
        #     smsl.process(file_data["DATA"])
        else:
            raise smslDataStructNotSupportedError(f"{d_type} data structure is not supported.")
    
class smslState:
    """
    The State class
    """
    def __init__(self, 
                 name: str):
        self.state_name = name

class smslOperation:
    """
    The Operation class
    """
    def __init__(self, 
                 name: str):
        self.operation_name = name

class smslStateBranch:
    """
    The SB class
    """
    def __init__(self, 
                 name: str, 
                 initial: smslState = None, 
                 activating: smslState = None, 
                 num_fact: int = 0, 
                 sub_sb: smslDict_SB_Statedigit = {},
                 states: list[smslState] = [],
                 operations: list[smslOperation] = [],
                 sb_level: int = None ):
        """
        Constructor
        """
        self.name = name
        self.initial = initial
        self.activating = activating
        self.num_fact = num_fact
        self.sub_sb = sub_sb
        self.graph = nx.Graph()
        self.add_states(states)
        self.add_operations(operations)
        self._post_process()

    def _post_process(self):
        """
        Post processing
        - Get the SB level
        - ...
        """
        return

    def add_states(self, states : list[smslState]):
        """
        Add states from a list
        """
        self.graph.add_nodes_from(states)
    
    def add_operations(self, operations : list[smslOperation]):
        """
        Add operations from a list
        """
        self.graph.add_edges_from(operations)

    def plot_sb(self, with_labels=True):
        """
        Plot the state branch
        """
        nx.draw(self.graph, with_labels=True)
        plt.show()