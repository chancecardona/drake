Robot Control Algorithms using the DRAKE solvers

https://drake.mit.edu/pydrake/index.html

The following are example applications using various capabilities of the DRAKE optimization software.

### math_prog.py: Mathematical Program Using Drake
Create a [MathematicalProgram](https://drake.mit.edu/doxygen_cxx/classdrake_1_1solvers_1_1_mathematical_program.html), which is an optimization problem that drake can solve. 
You can then add your variable type to it (continuous or non) for if your x,y,z etc should be 
continuous or discrete variables.
This shows how to add basic constraints and costs.
- Constraint:
    A function with upper and lower bounds. Constraints are mutable.
    Represent physical constraints of your model, etc.
- Cost:
    What the loss function is going to minimize. PositionCost's exist for L2 distance.
    OrientationCost, etc. This shows how to pass in a function of your variable vectors.

### traj_opt.py: Trajectory Optimization
Shows how to create a particle that follows you around as an example of using a Plant.
- Plant:
    A model for a physical system consisting of a simple body or etc.
We use this to create "brick on ice" to model our particle with a prismatic joint.
We then have a 
- QuadraticErrorCost
    Which models the cost as the distance between the goal and the current state quadratically
    so we can use [quadratic numerical optimization techniques](https://en.wikipedia.org/wiki/Quadratic_programming).
We also set some basic constraints to set the boundary conditions (initial states) and dynamic constraints.
We add (newton's method)[https://en.wikipedia.org/wiki/Newton%27s_method] as a constraint so we are really solving the lagrangian with the optimization.

### ik.py: Inverse Kinematics 
Uses MathematicalProgram in a real context. Namely loads a robot arm via SDF as a 
MultibodyPlant.
- MultibodyPlant: 
    A model for a physical system consisting of a collection of interconnected bodies.
    This is what will represent the physical system of our arm. Including its kinematics and 
    dynamics.
    - Can also define frames (for robot joints) within it, weld frames together, calculate Transforms, and set (joint) Positions, etc.
This then shows how to create a controller (aka a MathProg we Solve()) for the robot by using the plants properties and 

### mjc_cart_pole.py: Solving the Cart Pole control problem visually in Mujoco.
Shows how to use the [mujoco](https://mujoco.readthedocs.io/en/stable/overview.html) [visualizer](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer) and use these principles to model and solve the controller using [MPC](https://en.wikipedia.org/wiki/Model_predictive_control) as our control scheme.

