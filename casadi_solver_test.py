import os
from solver.config import GFoldConfig
from solver.solver_casadi import GFoldSolver
from solver.visualization import plot_results

solver = GFoldSolver(GFoldConfig(n=100))

print("Solving G-FOLD optimization problem...")
solution = solver.solve(verbose=True)
print(f"Final mass: {solution['final_mass']:.2f} kg")
    
# Plot results
save_path = os.path.join(".\\", "gfold_plot.png")
plot_results(solution, save_path=save_path, show=True)