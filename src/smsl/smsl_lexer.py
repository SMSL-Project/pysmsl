from smsl_token import *
from smsl_position import *
from smsl_errors import *

class Lexer:

    def __init__(self, fn, text):
        self.fn = fn
        self.text = text
        self.pos = Position(-1, 0, -1, fn, text)
        self.current_char = None
        self.advance()

    def advance(self):
        self.pos.advance(self.current_char)
        self.current_char = self.text[self.pos.idx] if self.pos.idx < len(self.text) else None

    def make_tokens(self):
        tokens = []

        while self.current_char != None:

            if self.current_char in ' \t':
                self.advance()
            elif self.current_char == '#':
                tokens.append(Token(SMSL_TT_HEADER))
            elif self.current_char == '{':
                tokens.append(Token(SMSL_TT_SBCONTAINER_LEFT))
            elif self.current_char == '}':
                tokens.append(Token(SMSL_TT_SBCONTAINER_RIGHT))
            elif self.current_char == '[':
                tokens.append(Token(SMSL_TT_STATECONTAINER_LEFT))
            elif self.current_char == ']':
                tokens.append(Token(SMSL_TT_STATECONTAINER_RIGHT))
            # elif self.current_char == '->':
            #     tokens.append( TODO )
            # elif self.current_char == '//':
            #     tokens.append( TODO )
            else:
                pos_start = self.pos.copy()
                char = self.current_char
                self.advance()
                return [], IllegalCharError(pos_start, self.pos, "'" + char + "'")

        return tokens, None