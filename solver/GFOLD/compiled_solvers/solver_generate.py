import os
import sys

# Add parent directory to Python path to import solver module when in compiled_solvers
current_dir_name = os.path.basename(os.getcwd())
if current_dir_name == "compiled_solvers":
    project_root = os.path.dirname(os.path.dirname(os.getcwd()))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

from solver.GFOLD import GFoldSolver, GFoldConfig
from numpy import array
import multiprocessing
import shutil
import subprocess

def generate_c_solver(case):
    print(f"generating c solver for {case}")
    vessel_config = GFoldConfig()
    if case == "tower_catch":
        vessel_config.spacecraft.target_position = array([0, 0, 500])
        vessel_config.spacecraft.target_velocity = array([0, 0, -50])
        vessel_config.environment.gravity = array([0,0,-9.80665])
        vessel_config.solver.n = 100
    elif case == "normal_landing":
        vessel_config.environment.gravity = array([0,0,-9.80665])
        vessel_config.solver.n = 100
    # write your case here
    else:
        raise ValueError(f"unknown case {case}")
    
    GFoldSolver(vessel_config).generate_code(case)
    
    # Fix import statements in generated cpg_solver.py file
    # fix_import_statements(case)

def main():
    # Check if already in compiled_solvers directory
    current_dir = os.path.basename(os.getcwd())
    if current_dir != os.path.join("solver", "GFOLD", "compiled_solvers"):
        # Create compiled_solvers directory
        compiled_solvers_dir = os.path.join("solver", "GFOLD", "compiled_solvers")
        os.makedirs(compiled_solvers_dir, exist_ok=True)
        
        # Copy current script to compiled_solvers directory
        current_script = __file__
        target_script = os.path.join(compiled_solvers_dir, "solver_generate.py")
        shutil.copy2(current_script, target_script)
        
        # Run copied script in compiled_solvers directory
        original_cwd = os.getcwd()
        try:
            os.chdir(compiled_solvers_dir)
            # Add parent directory to Python path for importing solver module
            sys.path.insert(0, original_cwd)
            # Run copied script
            subprocess.run([sys.executable, "solver_generate.py"])
        finally:
            os.chdir(original_cwd)
    else:
        # Already in compiled_solvers directory, generate solvers directly
        multiprocessing.Process(target=generate_c_solver, args=("normal_landing",)).start()
        multiprocessing.Process(target=generate_c_solver, args=("tower_catch",)).start()

if __name__ == "__main__":
    main()
