from .smsl_state import smslState

class smslOperation:
    """
    The Operation class
    """
    def __init__(self, 
                 name : str,
                 start_state : smslState,
                 end_state : smslState):
        assert isinstance(start_state, smslState), 'Start state has to be smslState.'
        assert isinstance(end_state, smslState), 'End state has to be smslState.'
        self.operation_name = name
        self.start_state = start_state
        self.end_state = end_state

    def _to_graph_edge(self):
        return (self.start_state, self.end_state, {'OP':self.operation_name})
    
    def __repr__(self) -> str:
        return f"smslOperation({repr(self.start_state)} -> {repr(self.end_state)})"