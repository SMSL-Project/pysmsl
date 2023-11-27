from smsl_state_branch import smslStateBranch
from smsl_errors import (
    smslSubSBNotFound,
)

def _assign_levels(node : smslStateBranch, current_level=0):
    """
    Helper to assign sb levels recursively
    """
    node.sb_level = current_level
    for i in node.sub_sbs.keys():
        _assign_levels(i, current_level+1)

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
    _temp_list_name_subsb = []
    _temp_name_lookup = {}
    for sb in sb_list:
        _temp_name_lookup[sb.name] = sb
    for sb in sb_list:
        for sub_sb_name, state_digit in sb._sub_sbs.items():
            if sub_sb_name not in _temp_name_lookup.keys():
                raise smslSubSBNotFound
            sb.sub_sbs[_temp_name_lookup[sub_sb_name]] = state_digit
            _temp_list_name_subsb.append(sub_sb_name)
        sb._sub_sbs = {} # clear

    # - Remove SBs that have an SB level of >= 1 (starting 0)
    for sub_sb_name in _temp_list_name_subsb:
        sb_list.remove(_temp_name_lookup[sub_sb_name])

    # - Get the SB level of each SB (starting 0)
    for sb_root in sb_list:
        _assign_levels(sb_root)

    return sb_list