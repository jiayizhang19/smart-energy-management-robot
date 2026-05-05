#!/usr/bin/env python3

import random
import py_trees


class CheckOccupancy(py_trees.behaviour.Behaviour):
    def __init__(self, waypoint: str):
        super().__init__(f'CheckOccupancy({waypoint})')
        self.waypoint = waypoint
        self.bb = py_trees.blackboard.Client(name='CheckOccupancy')
        self.bb.register_key('occupied', access=py_trees.common.Access.WRITE)

    def update(self):
        self.bb.occupied = random.choice([True, False])
        self.logger.info(
            f'[{self.waypoint}] Occupancy: '
            f'{"OCCUPIED" if self.bb.occupied else "UNOCCUPIED"}'
        )
        return py_trees.common.Status.SUCCESS


class ManageLight(py_trees.behaviour.Behaviour):
    def __init__(self, waypoint: str):
        super().__init__(f'ManageLight({waypoint})')
        self.waypoint = waypoint
        self.bb = py_trees.blackboard.Client(name='ManageLight')
        self.bb.register_key('occupied', access=py_trees.common.Access.READ)
        self.bb.register_key('light_on',  access=py_trees.common.Access.WRITE)

    def update(self):
        self.bb.light_on = self.bb.occupied
        state = 'ON' if self.bb.light_on else 'OFF'
        self.logger.info(f'[{self.waypoint}] Light: {state}')
        return py_trees.common.Status.SUCCESS


class ResolveEnergy(py_trees.behaviour.Behaviour):
    def __init__(self, waypoint: str):
        super().__init__(f'ResolveEnergy({waypoint})')
        self.waypoint = waypoint
        self.bb = py_trees.blackboard.Client(name='ResolveEnergy')
        self.bb.register_key('occupied', access=py_trees.common.Access.READ)
        self.bb.register_key('light_on',  access=py_trees.common.Access.READ)

    def update(self):
        if not self.bb.occupied and self.bb.light_on:
            self.logger.warning(
                f'[{self.waypoint}] Energy waste detected — turning light OFF'
            )
        else:
            self.logger.info(f'[{self.waypoint}] Energy OK')
        return py_trees.common.Status.SUCCESS


def create_inspection_tree(waypoint: str) -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(
        name=f'Inspect({waypoint})',
        memory=True
    )
    root.add_children([
        CheckOccupancy(waypoint),
        ManageLight(waypoint),
        ResolveEnergy(waypoint),
    ])
    return root
