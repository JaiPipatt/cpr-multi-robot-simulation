class Goal:
    def __init__(self, id, position):
        self.id = id
        self.position = position
        self.found = 0

    def picking(self):
        self.found += 1

    def picked(self): # need two robots to pick the goal
        return self.found >= 2
