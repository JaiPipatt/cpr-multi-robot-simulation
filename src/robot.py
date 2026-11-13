import random

from logger import log

# (dx, dy) of the cell in front, per facing (screen coords: y down)
FRONT = {"N": (0, -1), "E": (1, 0), "S": (0, 1), "W": (-1, 0)}
CLOCKWISE = ["N", "E", "S", "W"]

WAIT_TIMEOUT = 20              # steps a robot waits on a goal cell before giving up
CLAIM_TIME = WAIT_TIMEOUT + 5  # steps a robot remembers a goal as already announced
ROBOT_COUNT = 10               # default N; the map overwrites it with the real robot count


class robot:
    def __init__(self, id, position, mode="random"):
        self.id = id
        self.position = position
        self.mode = mode          # "random", "userinput", "scripted" or "auto"
        self.orientation = "N"
        self.sensed_data = []
        self.messages = []        # messages read this step
        self.inbox = []           # incoming bundles as [bundle, steps_of_delay_left]
        self.outbox = []          # payloads to broadcast at the end of this step
        self.script = []          # actions consumed one per step in "scripted" mode

        # state machine ("auto" mode)
        # exploring -> backoff -> proposing -> accepting -> heading -> waiting
        self.state = "exploring"
        self.goal = None          # {"id": goal id, "pos": [x, y]} I am working on
        self.wait_count = 0       # steps spent in waiting
        self.claimed = {}         # goal id -> steps left that I remember it as announced
        self.picked = set()       # goal ids known to be captured (ignore late messages about them)
        self.t = 0                # my own step counter, for timers
        self.backoff_until = 0

        # Paxos
        self.N = ROBOT_COUNT      # number of robots (majority = more than N // 2)
        self.lamport_clock = 0
        self.inst = None          # Paxos instance I work on: (goal_id, finder's lamport clock)
        self.ballot = None        # my ballot as proposer: (lamport_clock, id)
        self.promises = {}        # proposer: sender -> (ab, av) from its PROMISE
        self.acceptor_mem = {}    # acceptor: inst -> {"prom", "ab", "av"}
        self.vote = {}            # learner: inst -> {(ballot, value): set of senders}
        self.chosen = {}          # learner: inst -> chosen partner id

    # ---- physical robot ----

    def do(self, action):
        if action == "forward":
            dx, dy = FRONT[self.orientation]
            self.position[0] += dx
            self.position[1] += dy
        elif action == "turn_left":
            self.orientation = CLOCKWISE[(CLOCKWISE.index(self.orientation) - 1) % 4]
        elif action == "turn_right":
            self.orientation = CLOCKWISE[(CLOCKWISE.index(self.orientation) + 1) % 4]
        # "pick_up" changes nothing on the robot; the map counts it

    def sense(self, target_in_front, robot_id_in_front, robot_id_same_position):
        dx, dy = FRONT[self.orientation]
        self.sensed_data = {
            "target_in_front": target_in_front,
            "target_pos": [self.position[0] + dx, self.position[1] + dy],  # cell in front
            "robot_id_in_front": robot_id_in_front,
            "robot_id_same_position": robot_id_same_position,
        }

    # ---- communication ----

    def read(self):
        # Bundles whose delay is 0 are readable now, the others get one step closer
        ready = [bundle for bundle, delay in self.inbox if delay == 0]
        self.inbox = [[bundle, delay - 1] for bundle, delay in self.inbox if delay > 0]
        self.messages = []
        for bundle in ready:  # one broadcast = a list of payloads
            for m in bundle:
                self.lamport_clock = max(self.lamport_clock, m["lamport_clock"]) + 1  # Lamport receive rule
                self.messages.append(m)

    def send(self):
        # Hand this step's payloads to the map as one broadcast
        if not self.outbox:
            return None
        self.lamport_clock += 1  # Lamport send rule
        for m in self.outbox:
            m["lamport_clock"] = self.lamport_clock
        bundle, self.outbox = self.outbox, []
        self.inbox.append([bundle, 0])  # I hear my own broadcast (no delay)
        return bundle

    # ---- helpers ----

    def set_state(self, state):
        log("state", f"Robot {self.id}: {self.state} -> {state}")
        self.state = state
        self.wait_count = 0

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

    # ---- Paxos: every robot is always an acceptor and a learner ----

    def paxos_acceptor(self):
        for m in self.messages:
            if m["type"] not in ("PREPARE", "ACCEPT"):
                continue
            a = self.acceptor_mem.setdefault(m["inst"], {"prom": None, "ab": None, "av": None})
            if m["type"] == "PREPARE" and (a["prom"] is None or m["ballot"] > a["prom"]):
                a["prom"] = m["ballot"]  # promise to ignore lower ballots, report what I accepted
                self.outbox.append({"type": "PROMISE", "inst": m["inst"], "goal_id": m["goal_id"],
                                    "to": m["sender"], "ballot": m["ballot"],
                                    "ab": a["ab"], "av": a["av"], "sender": self.id})
            elif m["type"] == "ACCEPT" and (a["prom"] is None or m["ballot"] >= a["prom"]):
                a["prom"] = a["ab"] = m["ballot"]
                a["av"] = m["value"]
                self.outbox.append({"type": "ACCEPTED", "inst": m["inst"], "goal_id": m["goal_id"],
                                    "ballot": m["ballot"], "value": m["value"], "sender": self.id})

    def paxos_learner(self):
        # A value is chosen once a majority accepted the same (ballot, value)
        for m in self.messages:
            if m["type"] != "ACCEPTED":
                continue
            votes = self.vote.setdefault(m["inst"], {})
            votes.setdefault((m["ballot"], m["value"]), set()).add(m["sender"])
            for (b, v), voters in votes.items():
                if len(voters) > self.N // 2 and m["inst"] not in self.chosen:
                    self.chosen[m["inst"]] = v
                    log("consensus", f"Robot {self.id}: goal {m['goal_id']} chosen value {v}")

    # ---- decision ----

    def decide(self):
        """Return the action for this step, or None to stay idle."""
        self.t += 1

        self.paxos_acceptor()
        self.paxos_learner()
        if self.state in ("backoff", "proposing", "accepting") and self.inst in self.chosen:
            if self.chosen[self.inst] == self.id:
                self.set_state("heading")    # I'm the partner
            else:
                self.set_state("exploring")  # someone else got it

        # Remember announced goals for a while, so nobody announces them twice
        self.claimed = {g: t - 1 for g, t in self.claimed.items() if t > 1}
        for m in self.messages:
            if m["type"] == "FOUND":
                self.claimed[m["goal_id"]] = CLAIM_TIME
            elif m["type"] == "PICKED":
                self.claimed.pop(m["goal_id"], None)
                self.picked.add(m["goal_id"])
        if self.state in ("heading", "waiting") and any(
                m["type"] == "PICKED" and m["goal_id"] == self.goal["id"]
                for m in self.messages):
            self.set_state("exploring")  # someone else already picked my goal
            return None
        if self.state in ("backoff", "proposing", "accepting", "heading") and self.goal["id"] in self.picked:
            self.set_state("exploring")  # goal already captured
            return None

        if self.state == "exploring":
            found = [m for m in self.messages
                     if m["type"] == "FOUND" and m["inst"] not in self.chosen
                     and m["goal_id"] not in self.picked]
            # A goal found by two robots at once needs no helper: the finders pair up
            found = [m for m in found
                     if sum(f["goal_id"] == m["goal_id"] for f in found) == 1]
            if found:
                # Someone found a goal: volunteer for the closest one, after a backoff
                msg = min(found, key=lambda m: self.distance_to(self.position, m["goal_pos"]))
                self.goal = {"id": msg["goal_id"], "pos": msg["goal_pos"]}
                self.inst = msg["inst"]
                self.backoff_until = self.t + self.distance_to(self.position, self.goal["pos"])
                self.set_state("backoff")
                return None
            if (self.sensed_data["target_in_front"] is not None
                    and self.sensed_data["target_in_front"] not in self.claimed
                    and not self.sensed_data["robot_id_in_front"]):
                # Found a goal on an empty cell: announce it, step onto it, wait for a partner
                self.goal = {"id": self.sensed_data["target_in_front"],
                             "pos": self.sensed_data["target_pos"]}
                self.inst = (self.goal["id"], self.lamport_clock)
                self.outbox.append({"type": "FOUND", "inst": self.inst, "goal_id": self.goal["id"],
                                    "goal_pos": self.goal["pos"], "sender": self.id})
                self.set_state("waiting")
                return "forward"
            if self.sensed_data["target_in_front"] is not None:
                return random.choice(["turn_left", "turn_right"])  # never step onto a goal
            return random.choice(["forward", "turn_left", "turn_right"])

        if self.state == "backoff":
            # Someone closer already proposes for this instance: let them
            if any(m["type"] == "PREPARE" and m["inst"] == self.inst for m in self.messages):
                self.set_state("exploring")
                return None
            if self.t >= self.backoff_until:
                # Paxos phase 1a: new ballot, no value yet
                self.ballot = (self.lamport_clock, self.id)
                self.outbox.append({"type": "PREPARE", "inst": self.inst, "goal_id": self.goal["id"],
                                    "ballot": self.ballot, "sender": self.id})
                self.promises = {}
                self.set_state("proposing")
            return None

        if self.state == "proposing":
            # Paxos phase 1b: collect promises for my ballot
            for m in self.messages:
                if m["type"] == "PROMISE" and m["to"] == self.id and m["ballot"] == self.ballot:
                    self.promises[m["sender"]] = (m["ab"], m["av"])
            # Paxos phase 2a: with a majority, propose the value accepted under the
            # highest ballot if any acceptor reported one, otherwise myself
            if len(self.promises) > self.N // 2:
                highest_ab = max((ab for ab, av in self.promises.values() if ab is not None), default=None)
                if highest_ab is not None:
                    value = max(av for ab, av in self.promises.values() if ab == highest_ab)
                else:
                    value = self.id
                self.outbox.append({"type": "ACCEPT", "inst": self.inst, "goal_id": self.goal["id"],
                                    "ballot": self.ballot, "value": value, "sender": self.id})
                self.set_state("accepting")
            return None

        if self.state == "accepting":
            return None  # paxos_learner() moves me on once a value is chosen

        if self.state == "heading":
            if self.position != self.goal["pos"]:
                if self.sensed_data["target_pos"] == self.goal["pos"]:          # facing my goal cell
                    if self.sensed_data["target_in_front"] != self.goal["id"]:  # goal already gone
                        self.set_state("exploring")
                        return None
                    if len(self.sensed_data["robot_id_in_front"]) >= 2:         # a pair is already there
                        return None
                return self.head_to(self.goal["pos"])
            self.set_state("waiting")

        if self.state == "waiting":
            # Sensing-only pickup: exactly one other robot on my goal cell is my pair,
            # and it senses me in this same step, so we both pick together
            others = self.sensed_data["robot_id_same_position"]
            if len(others) == 1 and self.position == self.goal["pos"]:
                self.outbox.append({"type": "PICKED", "goal_id": self.goal["id"], "sender": self.id})
                self.picked.add(self.goal["id"])
                self.set_state("exploring")
                return "pick_up"
            self.wait_count += 1
            if self.wait_count > WAIT_TIMEOUT:
                self.set_state("exploring")  # nobody came
            return None
