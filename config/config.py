from krpc import client
import krpc
from Solver import GFoldConfig, GFoldSolver
from Internal import Targets, Targets_JNSQ

class KALConfig:
    def __init__(self, conn, target:Targets or Targets_JNSQ) -> None:
        self.conn = conn
        self.target = target
        
        

        self.solverconfig = GFoldConfig()
        self.solver = GFoldSolver(self.solverconfig)