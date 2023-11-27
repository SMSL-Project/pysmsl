from smsl_state import smslState

class smslOperation:
    """
    The Operation class
    """
    def __init__(self, 
                 name : str,
                 start_state : smslState,
                 end_state : smslState):
        self.operation_name = name
        self.start_state = start_state
        self.end_state = end_state

    def _to_graph_edge(self):
        return (self.start_state, self.end_state)
    
    def __repr__(self) -> str:
        return f"smslOperation({repr(self.start_state)} -> {repr(self.end_state)})"