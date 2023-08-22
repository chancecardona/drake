from pydrake.solvers import MathematicalProgram, Solve
import numpy as np
import matplotlib.pyplot as plt

# Create an empty MathematicalProgram named prog (with no decision variables, 
# constraints or cost function)
prog = MathematicalProgram()

x = prog.NewContinuousVariables(2)
print(x)
print(1 + 2*x[0] + 3*x[1] + 4*x[1])

y = prog.NewContinuousVariables(2, "dog")
print(y)
print(y[0] + y[0] + y[1] * y[1] * y[1])

var_matrix = prog.NewContinuousVariables(3, 2, "A")
print(var_matrix)

# Add constraints to x
prog.AddConstraint(x[0] * x[1] == 1)
prog.AddConstraint(x[0] >= 0)
prog.AddConstraint(x[0] - x[1] <= 0)

# Add a cost x(0)**2 + 3 to the total cost. Since prog doesn't have a cost before, now the total cost is x(0)**2 + 3
prog.AddCost(x[0] ** 2 + 3)
prog.AddCost(x[0] + x[1])

result = Solve(prog)

# print out the result.
print("Success? ", result.is_success())
# Print the solution to the decision variables.
print('x* = ', result.GetSolution(x))
# Print the optimal cost.
print('optimal cost = ', result.get_optimal_cost())
# Print the name of the solver that was called.
print('solver is: ', result.get_solver_id().name())
