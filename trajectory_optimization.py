#!/bin/env python3

from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import SpatialInertia, PlanarJoint

plant = MultibodyPlant(time_step=0)

ctx = plant.CreateDefaultContext()

return plant.num_positions()
