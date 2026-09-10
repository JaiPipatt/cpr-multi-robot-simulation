"""Predefined test scenarios for the multi-robot simulation.

Each scenario fixes goal positions and robot start states plus an action
script per robot, so behavior can be tested without typing on the console.

Robot facing: "N", "E", "S", "W".
A robot picks the cell it currently stands on when its script yields "pick_up".
"""

SCENARIOS = {
    # Two robots share a target cell and both pick -> valid pick.
    "two_pick": {
        "goals": [(5, 5)],
        "robots": [
            {"id": 0, "pos": [5, 6], "facing": "N", "script": ["forward", "pick_up"]},
            {"id": 1, "pos": [4, 5], "facing": "E", "script": ["forward", "pick_up"]},
        ],
    },
    # One robot picks an empty cell -> illegal (no goal there).
    "empty_pick": {
        "goals": [(0, 0)],
        "robots": [
            {"id": 0, "pos": [3, 3], "facing": "N", "script": ["pick_up"]},
        ],
    },
    # Exactly one robot picks a target -> illegal (needs two).
    "one_pick": {
        "goals": [(2, 2)],
        "robots": [
            {"id": 0, "pos": [2, 2], "facing": "N", "script": ["pick_up"]},
        ],
    },
    # Three robots pick the same target -> illegal (more than two).
    "three_pick": {
        "goals": [(5, 5)],
        "robots": [
            {"id": 0, "pos": [5, 6], "facing": "N", "script": ["forward", "pick_up"]},
            {"id": 1, "pos": [4, 5], "facing": "E", "script": ["forward", "pick_up"]},
            {"id": 2, "pos": [6, 5], "facing": "W", "script": ["forward", "pick_up"]},
        ],
    },
}
