# GFOLD_parms_static_gen

import numpy as np
from .GFOLD_Main_Solver import GFOLD

def e(i): # create a specific basis vector
    if i==0:
        return [1,0,0]
    if i==1:
        return [0,1,0]
    if i==2:
        return [0,0,1]

def S_mat(_w_): # _w_ to distinguish from our global namespace's w!
    return np.matrix([[0,-_w_[2],+_w_[1]],
                     [_w_[2],0, -_w_[0]],
                     [-_w_[1],_w_[0],0]])

def generate_solution(estimated_landing_time:float,
                      gravity:float,
                      dry_mass:float,
                      fuel_mass:float,
                      max_thrust:float,
                      min_throttle:float,
                      max_throttle:float,
                      max_structural_Gs:float,
                      specific_impulse:float,
                      max_velocity:float,
                      glide_slope_cone:float,
                      thrust_pointing_constraint:float,
                      planetary_angular_velocity:tuple,
                      initial_position:tuple,
                      initial_velocity:tuple,
                      target_position:tuple,
                      target_velocity:tuple,
                      prog_flag='p4',
                      solver='ECOS',
                      N_tf=160,
                      plot=False):
    #kwargs = locals()
    #for k,v in kwargs.items():
    #    print(f"{k.replace('_' ,' ')}: {v}")
    
    s = [ # scalars
        #'N'     : 100,                                                 # Deprecated, replaced by N_tf, static
        ['tf'    , estimated_landing_time],
        ['g0'    , 9.80665],                                            # standard gravity [m/s**2]
        ['m_dry' , dry_mass],                                           # dry mass kg
        ['m_fuel', fuel_mass],                                          # fuel in tons
        ['T_max' , max_thrust],                                         # thrust max
        ['tmin' ,  min_throttle],                                       # throttle ability
        ['tmax' ,  max_throttle],
        ['G_max' , max_structural_Gs],                                  # maximum allowable structural Gs
        ['Isp'   , specific_impulse],                                   # fuel efficiency (specific impulse)
        ['V_max' , max_velocity] ,                                      # velocity max
        ['y_gs'  , np.radians(glide_slope_cone)],                       # glide slope cone, must be 0 < Degrees < 90
        ['p_cs'  , np.cos(np.radians(thrust_pointing_constraint))],     # thrust pointing constraint
    ]
    v = [ # vectors

        ['g' , np.array([-gravity,0,0])],                               # gravity
        ['w' , np.array(planetary_angular_velocity)] ,                  # planetary angular velocity
        ['nh', np.array([1,0,0])   ],                                   # thrust vector reference direction

        ['r0' , np.array(initial_position) ],                           # initial position
        ['v0' , np.array(initial_velocity) ],                           # initial velocity
        #['v0' , np.array([0,  0,   0]) ],                              # initial velocity
        #['r0' , np.array([2400, 0, 0]) ],                              # initial position
        #['v0' , np.array([-40,  0,   0]) ],                            # initial velocity

        ['rf3', np.array(target_position)   ]    ,                              # final position target for p4
        ['rf' , np.array(target_position)   ]    ,                              # final position target
        ['vf' , np.array(target_velocity)   ]                                   # final velocity target
    ]

    sk = [k[0] for k in s]
    sv = [n[1] for n in s]
    # derived values:
    s += [
            ['alpha' , 1/(sv[sk.index('Isp')]*sv[sk.index('g0')])    ],                 # fuel consumption parameter
            ['m_wet' , (sv[sk.index('m_dry')]+sv[sk.index('m_fuel')])],                 # wet mass kg
            ['r1'    , sv[sk.index('tmin')]*sv[sk.index('T_max')] ],                    # lower thrust bound
            ['r2'    , sv[sk.index('tmax')]*sv[sk.index('T_max')] ],                    # upper thrust bound
            #['z0' , np.log(sv[sk.index('m_dry')]+sv[sk.index('m_fuel')])]              # initial log(mass) constraint
            ['z0' , np.log(sv[sk.index('m_dry')]+sv[sk.index('m_fuel')])]               # initial log(mass) constraint
    ]
    v += [
            ['c' , np.divide(e(0),np.tan(sv[sk.index('y_gs')]))],
    ]
    S_,Sk,n=[],{},0
    for loople in (s): # 'loople' = a list who wants to be a tuple, but wants assignment too :)
        key = loople[0]
        value=loople[1]
        Sk[key] = n
        S_.append( value)
        n+=1
    S=np.matrix(S_)

    V_,Vk,n=[],{},0
    for loople in (v):
        key = loople[0]
        value=loople[1]
        Vk[key] = n
        V_.append( value)
        n+=1

    V = np.matrix(V_).transpose() # form into shape (width,height) not (height,width)

    #print('MAKE S HAVE SHAPE',S.shape)
    #print('MAKE V HAVE SHAPE',V.shape)

    return GFOLD(S,V,Sk,Vk,S_,prog_flag=prog_flag,solver=solver,N_tf=N_tf,plot=plot)

'''
if __name__ == '__main__':
    result = generate_solution(estimated_landing_time=20,
                           gravity=9.80665,
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
                           plot=True)
'''