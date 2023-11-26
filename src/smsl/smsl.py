import json, yaml
from pathlib import Path
from smsl_errors import smslFileNotSupportedError

class smslStateMachine:
    """
    State Machine class
    """
    def __init__(self):
        """
        Empty class
        """
        self.state_machine = None
        self.
        self._post_process()
    
    def __init__(self, file_path : Path):
        """
        Initialize State Machine by a file
        """
        file_data = None
        if file_path.suffix == '.json':
            with open(file_path, 'r') as file:
                file_data = json.load(file)
        elif file_path.suffix == '.yaml':
            with open(file_path, 'r') as file:
                file_data = yaml.safe_load(file)
        # TODO finish smsl parser
        # elif file_path.suffix == '.smsl':
        # 
        else:
            raise smslFileNotSupportedError(f"{file_path.suffix} files are not supported.")
        self._post_process()
        
    def _post_process(self):
        if self.state_machine = None: