class CustomFilter:
    """
    passthrough - A filter that does nothing. For debugging only.
        Arguments: none.
    """
    filtername = "passthrough"

    def __init__(self):
        self.count = 0
        
    def filter(self, pc):
        self.count += 1
        return pc

        
    def statistics(self):
        print(f"{self.filtername}: count={self.count}")
        