class robot:
    def __init__(self, id, position, mode="random"):
        self.id = id
        self.position = position
        self.mode = mode  # "random", "userinput" or "scripted"
        self.orientation = "N"  # Default orientation is North
        self.sensed_data = []  # List to hold sensed data
        self.messages = []  # List to hold messages for this robot
        self.script = []  # Queue of actions consumed one per step in "scripted" mode
    def do(self, action):
        if action == "forward":
            if self.orientation == "N":
                self.position[1] -= 1
            elif self.orientation == "E":
                self.position[0] += 1
            elif self.orientation == "S":
                self.position[1] += 1
            elif self.orientation == "W":
                self.position[0] -= 1
        elif action == "turn_left":
            if self.orientation == "N":
                self.orientation = "W"
            elif self.orientation == "E":
                self.orientation = "N"
            elif self.orientation == "S":
                self.orientation = "E"
            elif self.orientation == "W":
                self.orientation = "S"
        elif action == "turn_right":
            if self.orientation == "N":
                self.orientation = "E"
            elif self.orientation == "E":
                self.orientation = "S"
            elif self.orientation == "S":
                self.orientation = "W"
            elif self.orientation == "W":
                self.orientation = "N"
        elif action == "pick_up":
            # += 1 to value in side
            pass

    def sense(self, target_in_front, robot_id_in_front, robot_id_same_position):
        robot_sensed_data = {
            "target_in_front": target_in_front,
            "robot_id_in_front": robot_id_in_front,
            "robot_id_same_position": robot_id_same_position
        }
        self.sensed_data = robot_sensed_data  # Update the robot's sensed data
        
    def read(self, messages):
        self.messages = messages  # Store the received messages for this robo

    def send(self):
        pass
