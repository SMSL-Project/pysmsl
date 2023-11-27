import networkx as nx
import matplotlib.pyplot as plt
from smsl_state import smslState
from smsl_state import smslState
from smsl_operation import smslOperation
from smsl_constants import (
    smslDict_SB_Statedigit
)

class smslStateBranch:
    """
    The SB class
    - name
    - initial
    - activating
    - num_facts
    - sub_sbs
    - _sub_sbs
        Temperary dictionary contains the name of sub SBs and
        the corresponding state digit. Is always empty after post-
        processing
    - states
    - operations
    - sb_level
    """
    def __init__(self, 
                 name: str, 
                 initial: smslState = None, 
                 activating: smslState = None, 
                 num_facts: int = 0, 
                 sub_sbs: smslDict_SB_Statedigit = {},
                 states: list[smslState] = [],
                 operations: list[smslOperation] = [],
                 sb_level: int = None,
                 _sub_sbs: dict = {}):
        """
        Constructor
        """
        self.name = name
        self.initial = initial
        self.activating = activating
        self.num_facts = num_facts
        self.sub_sbs = sub_sbs
        self._sub_sbs = _sub_sbs
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
        self.graph.add_edges_from([
            operation._to_graph_edge() for operation in operations
        ])

    def plot_sb(self, with_labels=True):
        """
        Plot the state branch
        """
        nx.draw(self.graph, with_labels=True)
        plt.show()
