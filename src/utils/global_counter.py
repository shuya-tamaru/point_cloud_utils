class Counter:
    def __init__(self):
        self.count = 0

    def next(self):
        self.count += 1
        return self.count
