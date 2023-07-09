# Token types "TT"
SMSL_TT_HEADER                 = 'TT_HEADER'
SMSL_TT_SBCONTAINER_LEFT       = 'TT_CONTAINER_SB_LEFT'
SMSL_TT_SBCONTAINER_RIGHT      = 'TT_CONTAINER_SB_RIGHT'
SMSL_TT_STATECONTAINER_LEFT    = 'TT_CONTAINER_STATE_LEFT'
SMSL_TT_STATECONTAINER_RIGHT   = 'TT_CONTAINER_STATE_RIGHT'
SMSL_TT_OPPOINTING             = 'TT_OP_POINTING'
SMSL_TT_COMMENT                = 'TT_COMMENT'
SMSL_TT_ASSIGN                 = 'TT_ASSIGN'
SMSL_TT_IDENTIFIER             = 'TT_IDENTIFIER'

class Token:

    def __init__(self, type, value=None, pos_start=None, pos_end=None):
        self.type = type
        self.value = value
        if pos_start:
            self.pos_start = pos_start.copy()
            self.pos_end = pos_start.copy()
            self.pos_end.advance()
        if pos_end:
            self.pos_end = pos_end

    def __repr__(self):
        if self.value:
            return f'{self.type}:{self.value}'
        return f'{self.type}'