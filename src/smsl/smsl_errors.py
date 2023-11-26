class smslFileNotSupportedError(Exception):
    def __init__(self, message="File format not supported."):
        self.message = message
        super().__init__(self.message)

class smslDataStructNotSupportedError(Exception):
    def __init__(self, message="Data structure not supported."):
        self.message = message
        super().__init__(self.message)

