import json, yaml, smsl_json, smsl_yaml

from pathlib import Path
from smsl_errors import (
    smslFileNotSupportedError,
    smslDataStructNotSupportedError
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
    
    def __init__(self, file_path):
        """
        Initialize State Machine by a file
        """
        file_path = Path(file_path)

        file_data = None
        if file_path.suffix == '.json':
            with open(file_path, 'r') as file:
                file_data = {"TYPE" : "JSON", "DATA" : json.load(file)}
        elif file_path.suffix == '.yaml':
            with open(file_path, 'r') as file:
                file_data = {"TYPE" : "YAML", "DATA" : yaml.safe_load(file)}
        # TODO finish smsl parser
        elif file_path.suffix == '.smsl':
            with open(file_path, 'r') as file:
                file_data = {"TYPE" : "SMSL", "DATA" : smsl.load(file)}
        else:
            raise smslFileNotSupportedError(
                f"{file_path.suffix} files are not supported."
            )
        
        self._process_file_data(file_data)
         
    def _process_file_data(self, file_data):
        d_type, d_data = file_data["TYPE"], file_data["DATA"]
        if d_type == "JSON":
            smsl_json.process_file_data(d_data)
        elif d_type == "YAML":
            smsl_yaml.process_file_data(d_data)
        # TODO finish smsl parser
        elif file_data["TYPE"] == "SMSL":
            smsl.process_file_data(file_data["DATA"])
        else:
            raise smslDataStructNotSupportedError(
                f"{d_type} data structure is not supported."
            )


