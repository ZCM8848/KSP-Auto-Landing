# version: 0.2.0
import krpc
from threading import Thread, Lock

from Internal import KAL
from Internal import Targets, Targets_JNSQ

conn = krpc.connect(name='KAL')
space_center = conn.space_center
params = {'skip_boosterback':False,
          'skip_entryburn':False,
          'skip_aerodynamic_guidance':False,
          'max_tilt':20,
          'throttle_limit':[0.1, 1],
          'target_roll':0,
          'gfold_start_velocity':140,
          'land_confirm':True,
          'landing_gear':True,
          'final_altitude':0}

lock = Lock()

B1 = KAL('B1', Targets_JNSQ.landing_zone_1, params, lock)
B2 = KAL('B2', Targets_JNSQ.landing_zone_2, params, lock)
to_recover = [B1, B2]

pool = [Thread(target=rocket.land) for rocket in to_recover]
for thread in pool:
    thread.start()
for thread in pool:
    thread.join()