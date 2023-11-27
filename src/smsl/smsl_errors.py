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

class smslSubSBNotFound(Exception):
    def __init__(self, message="A sub StateBranch was not found."):
        self.message = message
        super().__init__(self.message)

class smslSubSBPostProcessingError(Exception):
    def __init__(self, message="There is an error in the post processing of StateBranch."):
        self.message = message
        super().__init__(self.message)