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
        assert isinstance(initial, smslState), 'Initial state has to be smslState.'
        assert isinstance(activating, smslState), 'Activating state has to be smslState.'
        assert isinstance(num_facts, int), 'Num of facts has to be int.'

        self.name = name
        self.initial = initial
        self.activating = activating
        self.num_facts = num_facts
        self.sub_sbs = sub_sbs
        self._sub_sbs = _sub_sbs
        self.graph = nx.MultiDiGraph()
        self.add_states(states)
        self.add_operations(operations)
        self._post_process()

    def _post_process(self):
        """
        Post processing
        - Get the SB level
        """
        def _assign_levels(node : smslStateBranch, current_level=0):
            node.sb_level = current_level
            for i in node.sub_sbs.keys():
                _assign_levels(i, current_level+1)
        # assign levels
        _assign_levels(self)

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
        # TODO graph plots have issues
        # https://stackoverflow.com/questions/22785849/drawing-multiple-edges-between-two-nodes-with-networkx
        # https://graph-tool.skewed.de
        # TODO plot sub SBs
        pos=nx.spring_layout(self.graph,seed=5)
        nx.draw_networkx_nodes(self.graph, pos)
        nx.draw_networkx_edges(self.graph, pos)
        nx.draw_networkx_labels(self.graph, pos)
        nx.draw_networkx_edge_labels(self.graph, pos)
        
        plt.axis('off')
        axis = plt.gca()
        axis.set_xlim([1.2*x for x in axis.get_xlim()])
        axis.set_ylim([1.2*y for y in axis.get_ylim()])
        plt.tight_layout()
        plt.show()

    def shortest_path(self, start, end):
        """
        Return a list of the states on the shortest path
        """
        return nx.shortest_path(
            self.graph, 
            start, 
            end
        )
    
    def edge_path(self, path : list):
        """
        Return a list of the edges on the shortest path list of states
        """
        return [
            self.graph[path[i]][path[i+1]] \
            for i in range(len(path) - 1)
        ]
    
    def shortest_edge_path(self, start, end):
        """
        Return a list of the edges on the shortest path list of states
        """
        return self.edge_path(self.shortest_path(start, end))