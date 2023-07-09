from smsl_token import *
from smsl_nodes import *

class Parser:

    def __init__(self, tokens):
        self.tokens = tokens
        self.tok_idx = -1
        self.advance()

    def advance(self):
        self.tok_idx += 1
        if self.tok_idx < len(self.tokens):
            self.current_tok = self.tokens[self.tok_idx]
        return self.current_tok
    
    def parse(self):
        # TODO
        return res

    def tt_operation(self):
        tok = self.current_tok
        if tok.type in (SMSL_TT_IDENTIFIER):
            if peek(): # TODO
                self.advance()
                return OperationNode(tok)
    
    def tt_state(self):
        tok = self.current_tok
        if tok.type in (SMSL_TT_IDENTIFIER):
            if peek(): # TODO
                self.advance()
                return StateNode(tok)

    def tt_pointing(self):
        return self.bin_op( \
            self.tt_operation, \
            self.tt_state, \
            (SMSL_TT_OPPOINTING))

    def tt_assignop(self):
        return self.bin_op( \
            self.tt_state, \
            self.tt_endstates, \
            (SMSL_TT_ASSIGN))

    def tt_assignsb(self):
        return self.bin_op( \
            self.tt_state, \
            self.tt_statebranch, \
            (SMSL_TT_ASSIGN))
    
    def bin_op(self, func_l, func_r, ops):
        left = func_l()
        while self.current_tok.type in ops:
            op_tok = self.current_tok
            self.advance()
            right = func_r()
            left = BinOpNode(left, op_tok, right)
        return left

class ParseResult:
    def __init__(self):
        self.error = None
        self.node = None

    def register(self, res):
        if isinstance(res, ParseResult):
            if res.error: self.error = res.error
            return res.node
        return res

    def success(self, node):
        self.node = node
        return self

    def failure(self, error):
        self.error = error
        return self