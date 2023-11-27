import json
from smsl_state_branch import smslStateBranch
from smsl_state import smslState
from smsl_operation import smslOperation
from smsl_errors import (
    smslJsonSBInitialNotDefined,
    smslJsonSubSBNotFound
)

def process_file_data(json_data : dict):
    """
    Process loaded JSON data structure
    """
    sb_list  = []

    # Load the state branches in the json file
    for new_sb_name, new_sb_entry in json_data.items():
        if new_sb_name.startswith('_'):
            # skip non-data entries
            continue
        else:
            initial, activating, num_facts, _sub_sbs = \
                process_file_data_sb_header(new_sb_entry['HEADER'])
            states, operations = \
                process_file_data_sb_states(new_sb_entry)
            new_sb = smslStateBranch(
                new_sb_name, 
                initial, 
                activating, 
                num_facts, 
                {},
                states,
                operations,
                _sub_sbs
            )
            sb_list.append(new_sb)

    # Post process the state branches
    sb_list = post_process(sb_list)

    return sb_list

def process_file_data_sb_header(header_data : dict):
    """
    Process the JSON data in the HEADER attribute of an SB
    """
    initial, activating, num_facts, sub_sbs = None, None, 0, {}
    if 'INITIAL' in header_data.keys():
        initial = header_data['INITIAL']
    if 'ACTIVATING' in header_data.keys():
        activating = header_data['ACTIVATING']
    if 'NUM_FACTS' in header_data.keys():
        num_facts = header_data['NUM_FACTS']
    if 'SUB_SBS' in header_data.keys():
        sub_sbs = header_data['SUB_SBS']
    if initial == None:
        raise smslJsonSBInitialNotDefined
    return initial, activating, num_facts, sub_sbs

def process_file_data_sb_states(sb_data : dict):
    """
    Process the JSON data in the state attributes of an SB
    """
    states, operations = [], []
    for state, ops in sb_data.items():
        if state == "HEADER" or state.startswith('_'):
            # skip non-data entries
            continue
        states.append(smslState(state))
        operations.extend(
            smslOperation(op, state, end_state) \
                for op, end_state in ops.items()
        )
    return states, operations

def post_process(sb_list : list):
    """
    Post processing
    - Attach sub SBs
    - Get the SB level
    - Remove SBs that has an SB level of >= 1 (starting 0)
    """
    # - Attach sub SBs
    # Basically a problem that:
    #   L is a list of tree nodes, randomized order, no connection
    #   Need to reconstruct the tree from these tree nodes
    _temp_name_lookup = {}
    for sb in sb_list:
        _temp_name_lookup[sb.name] = sb
    for sb in sb_list:
        for sub_sb_name, state_digit in sb._sub_sbs.items():
            if sub_sb_name not in _temp_name_lookup.keys():
                raise smslJsonSubSBNotFound
            sb.sub_sbs[_temp_name_lookup[sub_sb_name]] = state_digit
            del sb._sub_sbs[sub_sb_name]
        if bool(sb._sub_sbs):
            raise smslJsonSubSBNotFound
    # - Get the SB level of each SB

    # - Remove SBs that has an SB level of >= 1 (starting 0)

def print_file_data(json_data : dict):
    """
    Print pretty data
    """
    json_formatted_str = json.dumps(json_data, indent=2)
    print(json_formatted_str)

# TODO: Process multiple files 
# (SB can contain sub SBs from other files)
# def process_files_data(json_data : dict):