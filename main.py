import sys
import random
import argparse

from PySide6.QtWidgets import (
    QApplication,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QLabel,
    QPushButton
)

from project.map_widget import MapWidget, GRID
from project.robot import robot
from project.goal import Goal


class MainWindow(QWidget):
    def __init__(self, robot_mode="random"):
        super().__init__()
        self.robot_mode = robot_mode  # decision mode for newly added robots
        self.setWindowTitle("Multi Robot Simulation")

        # Main layout: left + right
        main_layout = QHBoxLayout()

        # Map
        self.map = MapWidget()
        main_layout.addWidget(self.map, 3)

        # Status
        status_layout = QVBoxLayout()

        self._next_id = 0
        add_robot_btn = QPushButton("add robot")
        add_robot_btn.clicked.connect(self.on_add_robot)
        status_layout.addWidget(add_robot_btn)

        remove_robot_btn = QPushButton("remove robot")
        remove_robot_btn.clicked.connect(self.on_remove_robot)
        status_layout.addWidget(remove_robot_btn)

        start_iteration_btn = QPushButton("start iteration")
        start_iteration_btn.clicked.connect(self.on_start)
        status_layout.addWidget(start_iteration_btn)

        stop_iteration_btn = QPushButton("stop iteration")
        stop_iteration_btn.clicked.connect(self.on_stop)
        status_layout.addWidget(stop_iteration_btn)

        self.lbl_robots = QLabel()
        self.lbl_targets = QLabel()
        self.lbl_found = QLabel()
        self.lbl_iteration = QLabel()
        self.lbl_mode = QLabel()
        for lbl in (self.lbl_robots, self.lbl_targets, self.lbl_found,
                    self.lbl_iteration, self.lbl_mode):
            status_layout.addWidget(lbl)

        status_layout.addStretch(1)

        main_layout.addLayout(status_layout, 1)

        self.setLayout(main_layout)

        # Refresh status on every simulation step
        self.map.timer.timeout.connect(self.update_status)
        self.update_status()

    def update_status(self):
        running = self.map.timer.isActive()
        self.lbl_robots.setText(f"total robot: {len(self.map.robots)}")
        self.lbl_targets.setText(f"current target: {len(self.map.goals)}")
        self.lbl_found.setText(f"target found: {self.map.goals_found}")
        self.lbl_iteration.setText(f"iteration: {self.map.iteration}")
        self.lbl_mode.setText(f"Mode: {'Running' if running else 'Idle'}")

    def on_start(self):
        self.map.start_iteration()
        self.update_status()

    def on_stop(self):
        self.map.stop_iteration()
        self.update_status()

    def on_remove_robot(self):
        if self.map.robots:
            self.map.robots.pop()
            self.map.update()  # Trigger a repaint to remove the robot from the display
        self.update_status()

    def on_add_robot(self):
        # Random spawn cell + random facing
        pos = [random.randint(0, GRID - 1), random.randint(0, GRID - 1)]
        new_robot = robot(self._next_id, pos, mode=self.robot_mode)
        new_robot.orientation = random.choice(["N", "E", "S", "W"])
        self.map.add_robot(new_robot)
        self._next_id += 1
        self.update_status()


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Multi Robot Simulation")
    parser.add_argument(
        "--mode",
        choices=["random", "userinput"],
        default="random",
        help="decision mode for robots: 'random' (default) or 'userinput' "
             "(prompt on the console for each robot's action every step)",
    )
    return parser.parse_args(argv)


args = parse_args(sys.argv[1:])

app = QApplication([])

window = MainWindow(robot_mode=args.mode)
window.resize(1000, 600)
window.show()

app.exec()
