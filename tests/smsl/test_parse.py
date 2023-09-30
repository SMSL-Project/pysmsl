# from src.smsl.smsl_context import *
# from src.smsl.smsl_lexer import *
# from src.smsl.smsl_parser import *
# from src.smsl.smsl_interperter import *

def run(fn, text):
    """
    Test parsing structured text to SMSL data
    """
    
    global_symbol_table = SymbolTable()

    # Generate tokens
    lexer = Lexer(fn, text)
    tokens, error = lexer.make_tokens()
    if error: return None, error

    # Generate AST
    parser = Parser(tokens)
    ast = parser.parse()
    if ast.error: return None, ast.error

    # Run program
    interpreter = Interpreter()
    context = Context('<program>')
    context.symbol_table = global_symbol_table
    result = interpreter.visit(ast.node, context)

    return result.value, result.error