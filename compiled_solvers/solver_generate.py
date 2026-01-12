import os
import sys

# Add parent directory to Python path to import solver module when in compiled_solvers
if os.path.basename(os.getcwd()) == "compiled_solvers":
    parent_dir = os.path.dirname(os.getcwd())
    if parent_dir not in sys.path:
        sys.path.insert(0, parent_dir)

from solver import GFoldSolver, config
from numpy import array
import multiprocessing
import shutil
import subprocess

def generate_c_solver(case):
    """Generate C solver for a specific case"""
    print(f"\n{'='*50}")
    print(f"Generating C solver for case: {case}")
    print(f"{'='*50}")
    
    vessel_config = config.GFoldConfig()
    if case == "tower_catch":
        vessel_config.spacecraft.target_position = array([0, 0, 500])
        vessel_config.spacecraft.target_velocity = array([0, 0, -50])
        vessel_config.environment.gravity = array([0, 0, -9.80665])
        vessel_config.solver.n = 100
    elif case == "normal_landing":
        vessel_config.environment.gravity = array([0, 0, -9.80665])
        vessel_config.solver.n = 100
    else:
        raise ValueError(f"Unknown case: {case}")
    
    # Generate the solver code
    GFoldSolver(vessel_config).generate_code(case)
    
    # Fix import statements in generated cpg_solver.py file
    fix_import_statements(case)

def fix_import_statements(case):
    """Fix import statements in generated cpg_solver.py file to use relative imports"""
    cpg_solver_path = os.path.join(case, "cpg_solver.py")
    if not os.path.exists(cpg_solver_path):
        print(f"[WARN] File {cpg_solver_path} not found")
        return
    
    try:
        with open(cpg_solver_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        original_content = content
        
        # Replace problematic absolute import with relative import
        old_import = f"from {case} import cpg_module"
        new_import = "from . import cpg_module"
        
        if old_import in content:
            content = content.replace(old_import, new_import)
            print(f"[OK] Fixed import statement in {cpg_solver_path}")
        elif "import cpg_module" in content:
            print(f"[INFO] Found cpg_module import but pattern doesn't match in {case}/cpg_solver.py")
            # Show relevant lines for debugging
            lines = content.split('\n')
            for i, line in enumerate(lines, 1):
                if 'import cpg_module' in line:
                    print(f"  Line {i}: {line.strip()}")
        
        # Write back only if content changed
        if content != original_content:
            with open(cpg_solver_path, 'w', encoding='utf-8') as f:
                f.write(content)
                
    except Exception as e:
        print(f"[ERROR] Error fixing imports for {case}: {e}")

def ensure_package_structure():
    """Ensure compiled_solvers directory has proper package structure"""
    # Create compiled_solvers directory if it doesn't exist
    compiled_solvers_dir = "compiled_solvers"
    os.makedirs(compiled_solvers_dir, exist_ok=True)
    
    # Create __init__.py to make it a proper package
    init_file = os.path.join(compiled_solvers_dir, "__init__.py")
    if not os.path.exists(init_file):
        with open(init_file, 'w', encoding='utf-8') as f:
            f.write("# GFOLD Compiled Solvers Package\n")
        print(f"[OK] Created package initializer: {init_file}")
    
    return compiled_solvers_dir

def main():
    # Ensure we're in the right directory structure
    current_dir = os.path.basename(os.getcwd())
    
    if current_dir != "compiled_solvers":
        # Create compiled_solvers directory and set up package structure
        compiled_solvers_dir = ensure_package_structure()
        
        # Copy current script to compiled_solvers directory
        current_script = __file__
        target_script = os.path.join(compiled_solvers_dir, "solver_generate.py")
        shutil.copy2(current_script, target_script)
        print(f"[OK] Copied script to {target_script}")
        
        # Run copied script in compiled_solvers directory
        original_cwd = os.getcwd()
        try:
            os.chdir(compiled_solvers_dir)
            # Add parent directory to Python path for importing solver module
            if original_cwd not in sys.path:
                sys.path.insert(0, original_cwd)
            
            print(f"\nExecuting in {compiled_solvers_dir} directory...")
            result = subprocess.run([sys.executable, "solver_generate.py"], 
                                  capture_output=True, text=True, encoding='utf-8')
            
            # Print output from subprocess
            if result.stdout:
                print(result.stdout)
            if result.stderr:
                print(f"Errors:\n{result.stderr}", file=sys.stderr)
            
            if result.returncode != 0:
                print(f"[ERROR] Solver generation failed with return code {result.returncode}")
                sys.exit(1)
                
        finally:
            os.chdir(original_cwd)
    else:
        # Already in compiled_solvers directory, generate solvers directly
        print(f"Working in compiled_solvers directory: {os.getcwd()}")
        
        cases = ["normal_landing", "tower_catch"]
        processes = []
        
        # Start generation processes
        for case in cases:
            # Create case directory if it doesn't exist
            os.makedirs(case, exist_ok=True)
            
            p = multiprocessing.Process(target=generate_c_solver, args=(case,))
            p.start()
            processes.append(p)
            print(f"[OK] Started generation process for {case} (PID: {p.pid})")
        
        # Wait for all processes to complete
        print(f"\n{'='*50}")
        print("Waiting for all solver generation processes to complete...")
        print(f"{'='*50}\n")
        
        for p in processes:
            p.join()
            print(f"[OK] Process {p.pid} finished")
        
        print(f"\n{'='*50}")
        print("All solvers generated successfully!")
        print(f"{'='*50}")

if __name__ == "__main__":
    # Verify solver module is available before starting
    try:
        from solver import GFoldSolver, config
    except ImportError as e:
        print(f"[ERROR] Cannot import solver module: {e}")
        print("Make sure the solver module is in the Python path.")
        sys.exit(1)
    
    main()