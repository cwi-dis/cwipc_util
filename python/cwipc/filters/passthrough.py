class CustomFilter:
    """
    passthrough - A filter that does nothing. For debugging only.
        Arguments: none.
    """
    def __init__(self):
        self.count = 0
        
    def filter(self, pc):
        self.count += 1
        return pc

        
    def statistics(self):
        print(f"passthrough: count={self.count}")
        