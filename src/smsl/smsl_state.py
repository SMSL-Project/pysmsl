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
    