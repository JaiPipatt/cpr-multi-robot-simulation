import random

from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QPolygonF, QFont
from PySide6.QtCore import QPointF, QRectF, QTimer, Qt

from project.goal import Goal

GRID = 10


class MapWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(400, 400)
        self.robots = []  # List to hold robot instances
        self.goals = []   # List to hold goal instances
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.step)
        self.messages = []  # List to hold messages for robots
        self._next_goal_id = 0  # To assign unique IDs to goals
        self.goals_found = 0  # Cumulative count of goals picked successfully
        self.iteration = 0  # Number of steps run

    def start_iteration(self):
        self.timer.start(500)  # Update every 500 ms

    def stop_iteration(self):
        self.timer.stop()

    def step(self): 
        '''
        while number_of_iterations < max_iterations:
            if there are no more targets:
                generate random targets

            for each robot:
                read input messages

            for each robot:
                sense the environment
                decide which action to perform
                test for illegal behavior here

            for each robot:
                perform the action

            for each robot:
                send output messages
        '''
        print("Step function called")
        self.iteration += 1
        if not self.goals:  # Generate random goals if there are none
            positions = random.sample(
                [(x, y) for x in range(GRID) for y in range(GRID)],
                3
            )
            for i, goal_position in enumerate(positions):
                new_goal = Goal(
                    id=self._next_goal_id,
                    position=list(goal_position)
                )
                self.add_goal(new_goal)
                self._next_goal_id += 1

        for robot in self.robots:

            # Read input messages (not implemented)
            robot.read(self.messages)  # Assign the global messages to each robot

            # Sense the environment (not implemented)
            x, y, orientation = robot.position[0], robot.position[1], robot.orientation
            target_in_front = None
            robot_id_in_front = []
            robot_id_same_position = []
            for goal in self.goals:
                if orientation == "N" and goal.position == [x, y - 1]:
                    target_in_front = goal.id
                elif orientation == "E" and goal.position == [x + 1, y]:
                    target_in_front = goal.id
                elif orientation == "S" and goal.position == [x, y + 1]:
                    target_in_front = goal.id
                elif orientation == "W" and goal.position == [x - 1, y]:
                    target_in_front = goal.id

            for other_robot in self.robots:
                if other_robot.id != robot.id:
                    if orientation == "N" and other_robot.position == [x, y - 1]:
                        robot_id_in_front.append(other_robot.id)
                    elif orientation == "E" and other_robot.position == [x + 1, y]:
                        robot_id_in_front.append(other_robot.id)
                    elif orientation == "S" and other_robot.position == [x, y + 1]:
                        robot_id_in_front.append(other_robot.id)
                    elif orientation == "W" and other_robot.position == [x - 1, y]:
                        robot_id_in_front.append(other_robot.id)
                    if other_robot.position == [x, y]:
                        robot_id_same_position.append(other_robot.id)

            for other_robot in self.robots:
                if other_robot.id != robot.id and other_robot.position == robot.position:
                    robot_id_same_position.append(other_robot.id)
            robot.sense(target_in_front, robot_id_in_front, robot_id_same_position)

    
        robot_pick = []
        for robot in self.robots:
            # Decide which action to perform (random for now)
            if robot.mode == 'random':
                random_action = random.choice(["forward", "turn_left", "turn_right", "pick_up"])
            elif robot.mode == 'userinput':
                # Get user input for the action
                random_action = input(f"Robot {robot.id} at {robot.position} facing {robot.orientation}. Enter action (forward, turn_left, turn_right, pick_up): ")
                while random_action not in ["forward", "turn_left", "turn_right", "pick_up"]:
                    print("Invalid action. Please enter a valid action.")
                    random_action = input(f"Robot {robot.id} at {robot.position} facing {robot.orientation}. Enter action (forward, turn_left, turn_right, pick_up): ")
            else:
                continue  # unknown mode: robot idles this step
            robot.do(random_action)
            # fix for out of bounds
            if robot.position[0] < 0 or robot.position[0] >= GRID:
                robot.position[0] = max(0, min(robot.position[0], GRID - 1))
            if robot.position[1] < 0 or robot.position[1] >= GRID:
                robot.position[1] = max(0, min(robot.position[1], GRID - 1))

            if random_action == "pick_up":
                robot_pick.append((robot.id, robot.position))
                for goal in self.goals:
                    if robot.position == goal.position:
                        goal.picking()

        # check for illegal behavior (one robot pick goal, more than two robot pick goal, more than one robot pick goal but there is no goal in that position)
        for goal in self.goals:
            if goal.found == 2:
                print(f"Goal {goal.id} picked successfully by two robots")
                self.goals.remove(goal)  # Remove the goal from the list
                self.goals_found += 1
                robot_pick.remove((robot.id, robot.position) for robot in self.robots if robot.position == goal.position)
            elif goal.found > 2:
                print(f"Illegal behavior: more than two robots picked goal {goal.id}")
                goal.found = 0
            elif goal.found == 1:
                print(f"Illegal behavior: only one robot picked goal {goal.id}")
                goal.found = 0
        for robot_id, position in robot_pick:
            if not any(goal.position == position for goal in self.goals):
                print(f"Illegal behavior: robot {robot_id} picked a goal at {position} but there is no goal there")


        

        for robot in self.robots:
            # Send output messages (not implemented)
            robot.send()  # Placeholder for sending messages

        self.update()  # Trigger a repaint to show the updated positions

    def add_robot(self, robot):
        self.robots.append(robot)
        self.update()  # Trigger a repaint to show the new robot

    def add_goal(self, goal):
        self.goals.append(goal)
        self.update()

    def update_robot_position(self, robot_id, new_position):
        for robot in self.robots:
            if robot.id == robot_id:
                robot.position = new_position
                break
        self.update()  # Trigger a repaint to show the updated position

    def label_font(self, cell_size):
        font = QFont()
        font.setBold(True)
        font.setPixelSize(max(8, int(cell_size * 0.5)))
        return font

    def paintgoals(self, painter, cell_size):
        painter.setFont(self.label_font(cell_size))
        for goal in self.goals:
            x = goal.position[0] * cell_size
            y = goal.position[1] * cell_size

            painter.setBrush(QColor("green"))
            painter.setPen(QColor("black"))
            painter.drawRect(int(x), int(y), int(cell_size), int(cell_size))

            painter.setPen(QColor("white"))
            painter.drawText(
                QRectF(x, y, cell_size, cell_size),
                Qt.AlignCenter,
                f"G{goal.id}",
            )

    def paintrobots(self, painter, cell_size):
        painter.setBrush(QColor("red"))
        painter.setFont(self.label_font(cell_size))
        for robot in self.robots:
            cx = (robot.position[0] + 0.5) * cell_size
            cy = (robot.position[1] + 0.5) * cell_size
            r = cell_size * 0.4

            # unit vector for facing direction (screen coords: y down)
            dx, dy = {
                "N": (0, -1),
                "E": (1, 0),
                "S": (0, 1),
                "W": (-1, 0),
            }[robot.orientation]

            # tip in facing direction, two base corners perpendicular
            tip = QPointF(cx + dx * r, cy + dy * r)
            back_x = cx - dx * r
            back_y = cy - dy * r
            left = QPointF(back_x - dy * r * 0.7, back_y + dx * r * 0.7)
            right = QPointF(back_x + dy * r * 0.7, back_y - dx * r * 0.7)

            painter.setPen(QColor("red"))
            painter.drawPolygon(QPolygonF([tip, left, right]))

            # id label on the shape
            painter.setPen(QColor("white"))
            painter.drawText(
                QRectF(cx - cell_size / 2, cy - cell_size / 2, cell_size, cell_size),
                Qt.AlignCenter,
                f"R{robot.id}",
            )

    def paintEvent(self, event):
        painter = QPainter(self)

        width = self.width()
        height = self.height()

        cell_size = min(width, height) / GRID

        # Draw grid
        for row in range(GRID):
            for col in range(GRID):
                x = col * cell_size
                y = row * cell_size

                painter.drawRect(
                    int(x),
                    int(y),
                    int(cell_size),
                    int(cell_size)
                )

        # draw goals then robots
        self.paintgoals(painter, cell_size)
        self.paintrobots(painter, cell_size)
