import json, yaml
from pathlib import Path
from .smsl_json import process_file_data as json_process_file_data
from .smsl_yaml import process_file_data as yaml_process_file_data
from .smsl_errors import (
    smslFileNotSupportedError,
    smslDataStructNotSupportedError
)

class smslStateMachine:
    """
    State Machine class
    """
    def __init__(self, file_path=None):
        """
        Initialize State Machine by a file
        """
        if file_path is None:
            self.state_machine = None
            return
            
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
                # This part might need more work depending on the actual implementation
                from . import smsl_parser
                file_data = {"TYPE" : "SMSL", "DATA" : smsl_parser.load(file)}
        else:
            raise smslFileNotSupportedError(
                f"{file_path.suffix} files are not supported."
            )
        
        self._process_file_data(file_data)
         
    def _process_file_data(self, file_data):
        d_type, d_data = file_data["TYPE"], file_data["DATA"]
        if d_type == "JSON":
            sb_list = json_process_file_data(d_data)
        elif d_type == "YAML":
            sb_list = yaml_process_file_data(d_data)
        # TODO finish smsl parser
        elif file_data["TYPE"] == "SMSL":
            from . import smsl_parser
            sb_list = smsl_parser.process_file_data(file_data["DATA"])
        else:
            raise smslDataStructNotSupportedError(
                f"{d_type} data structure is not supported."
            )
        self.state_machine = sb_list