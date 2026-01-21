from solver.GFOLD.solver import GFoldSolver, GFoldConfig
from numpy import array
import os

def generate_c_solver(case):
    print(f"generating c solver for {case}")
    vessel_config = GFoldConfig()
    if "tower_catch" in case:
        vessel_config.spacecraft.target_position = array([0, 0, 500])
        vessel_config.spacecraft.target_velocity = array([0, 0, -50])
        vessel_config.environment.gravity = array([0,0,-9.80665])
        vessel_config.solver.n = 100
    elif "normal_landing" in case:
        vessel_config.environment.gravity = array([0,0,-9.80665])
        vessel_config.solver.n = 100
    # write your case here
    else:
        raise ValueError(f"unknown case {case}")
    
    GFoldSolver(vessel_config).generate_code(case)

if __name__ == "__main__":
    path = os.path.join("solver", "GFOLD", "compiled_solvers")
    case = ["normal_landing", "tower_catch"]
    for c in case:
        try:
            generate_c_solver(os.path.join(path, c))
        except Exception as e:
            print(f"failed to generate c solver for {c}: {e}")