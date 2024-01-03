from GFOLD_SOLVER.AutoIterator import GFOLD

gravity=9.80665
dry_mass=(22.2)*1e3
fuel_mass=(13.4)*1e3
max_thrust=845000*3
min_throttle=0.4
max_throttle=1.
max_structural_Gs=9
specific_impulse=282
max_velocity=900
glide_slope_cone=1
thrust_pointing_constraint=120
planetary_angular_velocity=(2.53*1e-5, 0, 6.62*1e-5)
initial_position=(2400, 20, 50)
initial_velocity=(-80,  3,   1)
target_velocity = (0,0,0)
target_position = (0,0,0)

G = GFOLD(gravity=9.80665,
          dry_mass=(22.2)*1e3,
          fuel_mass=(13.4)*1e3,
          max_thrust=845000*3,
          min_throttle=0.4,
          max_throttle=1.,
          max_structural_Gs=9,
          specific_impulse=282,
          max_velocity=900,
          glide_slope_cone=1,
          thrust_pointing_constraint=120,
          planetary_angular_velocity=(2.53*1e-5, 0, 6.62*1e-5),
          initial_position=(2400, 20, 50),
          initial_velocity=(-80,  3,   1),
          target_position=(0,0,0),
          target_velocity=(0,0,0))


print(G.find_best_result())