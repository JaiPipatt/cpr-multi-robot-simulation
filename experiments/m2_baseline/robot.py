import random

# (dx, dy) of the cell in front, per facing (screen coords: y down)
FRONT = {"N": (0, -1), "E": (1, 0), "S": (0, 1), "W": (-1, 0)}
CLOCKWISE = ["N", "E", "S", "W"]

REPLY_WAIT = 1     # steps to wait for POS replies 
WAIT_TIMEOUT = 30  # steps a robot waits on a goal cell before giving up
CLAIM_TIME = WAIT_TIMEOUT + 5  # steps a robot remembers a goal as already announced


class robot:
    def __init__(self, id, position, mode="random"):
        self.id = id
        self.position = position
        self.mode = mode  # "random", "userinput", "scripted" or "auto"
        self.orientation = "N"  # Default orientation is North
        self.sensed_data = []  # List to hold sensed data
        self.messages = []  # Messages read this step
        self.inbox = []  # Incoming messages as [msg, steps_of_delay_left]
        self.outbox = None  # Message to broadcast at the end of this step
        self.script = []  # Queue of actions consumed one per step in "scripted" mode

        # "auto" mode state machine: exploring, read_pos, heading, waiting
        self.goal = None  # {"id": goal id, "pos": [x, y]} this robot is working on
        self.replies = []  # POS messages collected while in read_pos
        self.reply_wait = 0  # steps left to wait for POS replies
        self.wait_count = 0  # steps spent in waiting
        self.partner = None  # id of the robot I pick up with
        self.ready_sent = False  # sent READY last step (partner was on my cell)
        self.claimed = {}  # goal id -> steps left that I remember it as announced
        self.state = "exploring"  # explorting, waiting, heading, ready
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
        dx, dy = FRONT[self.orientation]
        robot_sensed_data = {
            "target_in_front": target_in_front,
            "target_pos": [self.position[0] + dx, self.position[1] + dy],  # cell in front
            "robot_id_in_front": robot_id_in_front,
            "robot_id_same_position": robot_id_same_position
        }
        self.sensed_data = robot_sensed_data  # Update the robot's sensed data

    def read(self):
        # Messages whose delay is 0 are readable now, the others get one step closer
        self.messages = [msg for msg, delay in self.inbox if delay == 0]
        self.inbox = [[msg, delay - 1] for msg, delay in self.inbox if delay > 0]

    def send(self):
        # Hand the message decided this step to the map, which broadcasts it
        msg, self.outbox = self.outbox, None
        return msg

    # ---- "auto" mode: distributed control ----

    def set_state(self, state):
        print(f"Robot {self.id}: {self.state} -> {state}")
        self.state = state
        self.wait_count = 0
        self.ready_sent = False
        if state == "exploring":
            self.partner = None

    def distance_to(self, pos, goal_pos):
        return abs(pos[0] - goal_pos[0]) + abs(pos[1] - goal_pos[1])  # Manhattan

    def turn_toward(self, target):
        # One turn (or forward when already facing target)
        diff = (CLOCKWISE.index(target) - CLOCKWISE.index(self.orientation)) % 4
        if diff == 0:
            return "forward"
        return "turn_left" if diff == 3 else "turn_right"

    def head_to(self, goal_pos):
        # Fix x first, then y
        if self.position[0] != goal_pos[0]:
            return self.turn_toward("E" if goal_pos[0] > self.position[0] else "W")
        return self.turn_toward("S" if goal_pos[1] > self.position[1] else "N")

    def find_partner(self):
        # Another robot that found the same goal at once: the two lowest ids pair up
        finders = sorted([m["sender"] for m in self.messages
                          if m["type"] == "FOUND" and m["goal_id"] == self.goal["id"]]
                         + [self.id])
        if len(finders) > 1:
            if self.id in finders[:2]:
                self.partner = finders[1] if finders[0] == self.id else finders[0]
            else:
                self.set_state("exploring")
            return
        # Otherwise the helper is the closest robot that answered with POS
        # (same ranking the helpers use, so everybody agrees)
        replies = [(m["pos"], m["sender"]) for m in self.messages
                   if m["type"] == "POS" and m["goal_id"] == self.goal["id"]]
        if replies:
            best = min(replies, key=lambda c: (self.distance_to(c[0], self.goal["pos"]), c[1]))
            self.partner = best[1]

    def decide(self):
        """Return the action for this step, or None to stay idle."""
        # Remember goals someone already announced (even while busy), so a second
        # robot does not announce the same goal again. The memory fades with time.
        self.claimed = {g: t - 1 for g, t in self.claimed.items() if t > 1}
        for m in self.messages:
            if m["type"] == "FOUND":
                self.claimed[m["goal_id"]] = CLAIM_TIME
            elif m["type"] == "PICKED":
                self.claimed.pop(m["goal_id"], None)
        if self.state in ("heading", "waiting") and any(
                m["type"] == "PICKED" and m["goal_id"] == self.goal["id"]
                for m in self.messages):
            self.set_state("exploring")  # someone else already picked my goal
            return None
        if self.state == "exploring":
            found = [m for m in self.messages if m["type"] == "FOUND"]
            # A goal found by two robots at once needs no helper: the finders pair up
            found = [m for m in found
                     if sum(f["goal_id"] == m["goal_id"] for f in found) == 1]
            if found:
                # Someone found a goal: tell everyone where I am (lowest goal id first)
                msg = min(found, key=lambda m: m["goal_id"])
                self.goal = {"id": msg["goal_id"], "pos": msg["goal_pos"]}
                self.partner = msg["sender"]  # the finder is who I will pick with
                self.outbox = {"type": "POS", "goal_id": msg["goal_id"],
                               "sender": self.id, "pos": list(self.position)}
                self.replies = []
                self.reply_wait = REPLY_WAIT
                self.set_state("read_pos")
                return None
            if (self.sensed_data["target_in_front"] is not None
                    and self.sensed_data["target_in_front"] not in self.claimed):
                # Found a goal: announce it, step onto it and wait for a partner
                self.goal = {"id": self.sensed_data["target_in_front"],
                             "pos": self.sensed_data["target_pos"]}
                self.outbox = {"type": "FOUND", "goal_id": self.goal["id"],
                               "goal_pos": self.goal["pos"], "sender": self.id}
                self.set_state("waiting")
                return "forward"
            return random.choice(["forward", "turn_left", "turn_right"])  # no pick_up

        if self.state == "read_pos":
            self.replies += [m for m in self.messages
                             if m["type"] == "POS" and m["goal_id"] == self.goal["id"]]
            self.reply_wait -= 1
            if self.reply_wait > 0:
                return None
            # Closest robot goes (tie: lowest id). The finder is already at the goal.
            candidates = [(m["pos"], m["sender"]) for m in self.replies]
            candidates.append((list(self.position), self.id))
            best = min(candidates,
                       key=lambda c: (self.distance_to(c[0], self.goal["pos"]), c[1]))
            self.set_state("heading" if best[1] == self.id else "exploring")
            return None

        if self.state == "heading":
            if self.position != self.goal["pos"]:
                return self.head_to(self.goal["pos"])
            self.set_state("waiting")  # arrived: same rule as the finder

        if self.state == "waiting":
            if self.partner is None:
                self.find_partner()  # finder side: who is coming?
                if self.state != "waiting":
                    return None  # not part of the pair after all
            # Being on the cell is not enough: the partner may still be busy deciding.
            # Both robots say READY while the other is on their cell, and pick only
            # once each has sent READY and read the other's READY (same step).
            partner_here = self.partner in self.sensed_data["robot_id_same_position"]
            # The partner must also name me as its partner (pairs must agree)
            partner_ready = any(
                m["type"] == "READY" and m["sender"] == self.partner
                and m["goal_id"] == self.goal["id"] and m["partner"] == self.id
                for m in self.messages)
            if partner_here and partner_ready and self.ready_sent:
                self.outbox = {"type": "PICKED", "goal_id": self.goal["id"],
                               "sender": self.id}
                self.set_state("exploring")
                return "pick_up"
            self.ready_sent = partner_here
            if partner_here:
                self.outbox = {"type": "READY", "goal_id": self.goal["id"],
                               "sender": self.id, "partner": self.partner}
            self.wait_count += 1
            if self.wait_count > WAIT_TIMEOUT:
                self.set_state("exploring")  # no partner came, give up
            return None
