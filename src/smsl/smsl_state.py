class smslState:
    """
    The State class
    """
    def __init__(self, 
                 name: str):
        """
        Constructor
        """
        self.state_name = name

    def __repr__(self) -> str:
        return f"smslState({self.state_name})"
    
    def __eq__(self, other):
        if isinstance(other, smslState):
            return self.state_name == other.state_name
        return False

    def __hash__(self):
        return hash(self.state_name)
    