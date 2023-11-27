class smslFileNotSupportedError(Exception):
    def __init__(self, message="File format not supported."):
        self.message = message
        super().__init__(self.message)

class smslDataStructNotSupportedError(Exception):
    def __init__(self, message="Data structure not supported."):
        self.message = message
        super().__init__(self.message)

class smslJsonSBInitialNotDefined(Exception):
    def __init__(self, message="Initial state of the StateBranch is not defined."):
        self.message = message
        super().__init__(self.message)

class smslJsonSubSBNotFound(Exception):
    def __init__(self, message="A sub StateBranch was not found."):
        self.message = message
        super().__init__(self.message)