import numpy as np
from cvxpy import Variable, Parameter, Maximize, Minimize, Problem
from cvxpy import ECOS, SCS
from cvxpy import norm

from .GFOLD_utils import e
from .EvilPlotting import plot_run3D

import warnings
warnings.filterwarnings('ignore')

class GFOLD:
    def __init__(self, bundled_data:dict):
        self.gravity =                    bundled_data['gravity']
        self.dry_mass =                   bundled_data['dry_mass']
        self.fuel_mass =                  bundled_data['fuel_mass']
        self.max_thrust =                 bundled_data['max_thrust']
        self.min_throttle =               bundled_data['min_throttle']
        self.max_throttle =               bundled_data['max_throttle']
        self.max_structural_Gs =          bundled_data['max_structural_Gs']
        self.specific_impulse =           bundled_data['specific_impulse']
        self.max_velocity =               bundled_data['max_velocity']
        self.glide_slope_cone =           bundled_data['glide_slope_cone']
        self.thrust_pointing_constraint = bundled_data['thrust_pointing_constraint']
        self.planetary_angular_velocity = bundled_data['planetary_angular_velocity']
        self.initial_position =           bundled_data['initial_position']
        self.initial_velocity =           bundled_data['initial_velocity']
        self.target_position =            bundled_data['target_position']
        self.target_velocity =            bundled_data['target_velocity']
        self.prog_flag =                  bundled_data['prog_flag']
        self.solver =                     bundled_data['solver']
        self.N_tf =                       bundled_data['N_tf']
        self.plot =                       bundled_data['plot']
        self.min_tf =                     bundled_data['min_tf']
        self.max_tf =                     bundled_data['max_tf']

        self.S =                          None
        self.V =                          None
        self.Sk =                         None
        self.Vk =                         None

        self.solution =                   None

        if self.solver == 'ECOS':
            self.solver = ECOS
        elif self.solver == 'SCS':
            self.solver = SCS

    def generate_params(self, tf):
        s = [ # scalars
            #'N'     : 100,                                                   # Deprecated, replaced by N_tf, static
            ['tf'    , tf],
            ['g0'    , 9.80665],                                              # standard gravity [m/s**2]
            ['m_dry' , self.dry_mass],                                        # dry mass kg
            ['m_fuel', self.fuel_mass],                                       # fuel in tons
            ['T_max' , self.max_thrust],                                      # thrust max
            ['tmin' ,  self.min_throttle],                                    # throttle ability
            ['tmax' ,  self.max_throttle],                                    # throttle max
            ['G_max' , self.max_structural_Gs],                               # maximum allowable structural Gs
            ['Isp'   , self.specific_impulse],                                # fuel efficiency (specific impulse)
            ['V_max' , self.max_velocity] ,                                   # velocity max
            ['y_gs'  , np.radians(self.glide_slope_cone)],                    # glide slope cone, must be 0 < Degrees < 90
            ['p_cs'  , np.cos(np.radians(self.thrust_pointing_constraint))],  # thrust pointing constraint
        ]
        v = [ # vectors

            ['g' , np.array([-self.gravity,0,0])],                            # gravity
            ['w' , np.array(self.planetary_angular_velocity)] ,               # planetary angular velocity
            ['nh', np.array([1,0,0])],                                        # thrust vector reference direction

            ['r0' , np.array(self.initial_position)],                         # initial position
            ['v0' , np.array(self.initial_velocity)],                         # initial velocity
            #['v0' , np.array([0,  0,   0]) ],                                # initial velocity
            #['r0' , np.array([2400, 0, 0]) ],                                # initial position
            #['v0' , np.array([-40,  0,   0]) ],                              # initial velocity

            ['rf3', np.array(self.target_position)]    ,                      # final position target for p4
            ['rf' , np.array(self.target_position)]    ,                      # final position target
            ['vf' , np.array(self.target_velocity)]                           # final velocity target
        ]

        sk = [k[0] for k in s]
        sv = [n[1] for n in s]
        # derived values:
        s += [
                ['alpha' , 1/(sv[sk.index('Isp')]*sv[sk.index('g0')])    ],    # fuel consumption parameter
                ['m_wet' , (sv[sk.index('m_dry')]+sv[sk.index('m_fuel')])],    # wet mass kg
                ['r1'    , sv[sk.index('tmin')]*sv[sk.index('T_max')] ],       # lower thrust bound
                ['r2'    , sv[sk.index('tmax')]*sv[sk.index('T_max')] ],       # upper thrust bound
                #['z0' , np.log(sv[sk.index('m_dry')]+sv[sk.index('m_fuel')])] # initial log(mass) constraint
                ['z0' , np.log(sv[sk.index('m_dry')]+sv[sk.index('m_fuel')])]  # initial log(mass) constraint
        ]
        v += [
                ['c' , np.divide(e(0),np.tan(sv[sk.index('y_gs')]))],
        ]
        S,Sk,n=[],{},0
        for loople in (s): # 'loople' = a list who wants to be a tuple, but wants assignment too :)
            key = loople[0]
            value=loople[1]
            Sk[key] = n
            S.append( value)
            n+=1
        self.Sk = Sk
        S=np.matrix(S)
        self.S = S

        V,Vk,n=[],{},0
        for loople in (v):
            key = loople[0]
            value=loople[1]
            Vk[key] = n
            V.append( value)
            n+=1
        self.Vk = Vk
        V = np.matrix(V).transpose() # form into shape (width,height) not (height,width)
        self.V = V

        #print('MAKE S HAVE SHAPE',S.shape)
        #print('MAKE V HAVE SHAPE',V.shape)
        self.S, self.V = S, V
        self.Sk, self.Vk = Sk, Vk

        return None

    def solve(self, N_tf, iterative):

        V=self.V # for cvxpy testing
        S=self.S # for cvxpy testing
        sk, vk = self.Sk, self.Vk

        dt=Parameter((1,1),name='dt') # determines tf implicitly dt = tf/N,
                                    # tf = dt*N(const)

        dt.value = np.array([float(S[0,sk['tf']])/(N_tf)]).reshape((1,1))

        # Precalculate Z limits, then pass in as a PARAMETER

        z0=           Parameter((1,N_tf),name='z0')
        z1=           Parameter((1,N_tf),name='z1')
        mu_1=         Parameter((1,N_tf),name='mu_1')
        mu_2=         Parameter((1,N_tf),name='mu_2')
        z0_term=      Parameter((1,N_tf),name='z0_term')
        z1_term=      Parameter((1,N_tf),name='z1_term')

        z0_term_, z1_term_ = np.zeros((1,N_tf)),np.zeros((1,N_tf))
        z0_, z1_           = np.zeros((1,N_tf)),np.zeros((1,N_tf))
        mu_1_, mu_2_       = np.zeros((1,N_tf)),np.zeros((1,N_tf))

        for n in range(0,N_tf-1):
            z0_term_[0,n] = S[0,sk['m_wet']] - S[0,sk['alpha']] * S[0,sk['r2']] * (n) * dt.value  # see ref [2], eq 34,35,36
            z1_term_[0,n] = S[0,sk['m_wet']] - S[0,sk['alpha']] * S[0,sk['r1']] * (n) * dt.value
            z0_[0,n] = np.log( z0_term_[0,n] )
            z1_[0,n] = np.log( z1_term_[0,n] )
            mu_1_[0,n] = S[0,sk['r1']]/(z1_term_[0,n])
            mu_2_[0,n] = S[0,sk['r2']]/(z0_term_[0,n])

        z0_term.value = z0_term_
        z1_term.value = z1_term_
        z0.value = z0_
        z1.value = z1_
        mu_1.value = mu_1_
        mu_2.value = mu_2_

        # new variables here for brevity in the dynamics equations
        c=vk['c']
        g=vk['g']
        rf=vk['rf']
        alpha=sk['alpha']
        #print(c,g,rf,alpha)

        if self.prog_flag=='p3':
            program = 3
        elif self.prog_flag=='p4':
            program = 4

        x = Variable((6,N_tf),name='x') # state vector (3position,3velocity)
        u = Variable((3,N_tf),name='u') # u = Tc/mass because Tc[:,n]/m[n] is not allowed by DCP
        z = Variable((1,N_tf),name='z')  # z = ln(mass)
        s = Variable((1,N_tf),name='s') # thrust slack parameter

        con = []

        con += [x[0:3,0].reshape((3,1))  ==   V[:,vk['r0']].reshape((3,1))]
        con += [x[3:6,0].reshape((3,1))  ==   V[:,vk['v0']].reshape((3,1))]
        con += [x[3:6,N_tf-1].reshape((3,1))==V[:,vk['vf']].reshape((3,1))] # don't forget to slow down, buddy!

        con += [s[0,N_tf-1] == 0] # thrust at the end must be zero
        con += [u[:,0] == s[0,0]*np.array([1,0,0]) ] # thrust direction starts straight
        con += [u[:,N_tf-1] == s[0,N_tf-1]*np.array([1,0,0]) ] # and ends straight
        con += [z[0,0] == S[0,sk['z0']]] # convexified (7)

        if program==3:
            con += [x[0,N_tf-1] == 0] # end altitude

        elif program==4:
            # force landing point equal to found program 3 point
            con += [x[0:3,N_tf-1].reshape((3,1)) == V[:,vk['rf3']].reshape((3,1))]

        for n in range(0,N_tf-1): # any t in [0,tf] maps to any n in [0,N-1]

            # Leapfrog Integration Method
            #    accurate +/- sqrt( (dt*df/dr)**2 + 1)
            #    https://goo.gl/jssWkB
            #    https://en.wikipedia.org/wiki/Leapfrog_integration

            con += [x[3:6,n+1].reshape((3,1)) == x[3:6,n].reshape((3,1)) + (dt*0.5)*((u[:,n].reshape((3,1))+V[:,g].reshape((3,1))) + (u[:,n+1].reshape((3,1))+V[:,g].reshape((3,1))))]
            con += [x[0:3,n+1].reshape((3,1)) == x[0:3,n].reshape((3,1)) + (dt*0.5)*(x[3:6,n+1].reshape((3,1))+x[3:6,n].reshape((3,1)))]
            con += [x[0,n+1] <= x[0,n]] # we are doing a descent, not an ascent
            #con += [x[3,n+1] >= x[3,n]] # for energy optimal concern!(tested, but save only 0.3% of ΔV)

            con += [ norm((x[0:3,n].reshape((3,1))-V[:,rf].reshape((3,1)))[0:2] ) - V[0,c]*(x[0,n]-V[0,rf])  <= 0 ] # glideslope constraint
            con += [ norm(x[3:6,n].reshape((3,1))) <= S[0,sk['V_max']] ] # velocity

            con += [z[0,n+1] == z[0,n] - (S[0,alpha]*dt*0.5)*(s[0,n] + s[0,n+1])] # mass decreases
            con += [norm(u[:,n]) <= s[0,n]] # limit thrust magnitude & also therefore, mass

            # Thrust pointing constraint
            con += [  u[0,n] >= S[0,sk['p_cs']]*s[0,n]  ]
            if n > 0:
                # lower thrust bound
                #con += [s[0,n] >= mu_1[0,n] * (1 - (z[:,n] - z0[0,n]) + (1/2)*square(z[:,n] - z0[0,n]))]
                con += [s[0,n] <= mu_2[0,n] * (1 - (z[:,n] - z0[0,n]))] # upper thrust bound
                con += [z[0,n] >= z0[0,n]] # Ensures physical bounds on z are never violated
                con += [z[0,n] <= z1[0,n]]

        con += [x[0,0:N_tf-1] >= 0] # no, this is not the Boring Company!

        if self.prog_flag == 'p3':
            objective=Minimize(norm(x[0:3,N_tf-1].reshape((3,1))-V[:,rf]))
            problem=Problem(objective,con)
            obj_opt=problem.solve(solver=self.solver,verbose=False if iterative else True)
        elif self.prog_flag == 'p4':
            objective=Maximize(z[0,N_tf-1])
            problem=Problem(objective,con)
            obj_opt=problem.solve(solver=self.solver,verbose=False if iterative else True)
        
        x=x.value
        u=u.value
        s=s.value
        z=z.value
        tf=(N_tf*norm(dt.value)).value
        m=list(map(np.exp,z[0].tolist()))

        self.solution = {'x':x, 'u':u, 'm':m, 's':s, 'z':z}

        return None

    def find_optimal_solution(self):
        time = []
        cost = []
        iter_count = 0
        print('ITERATING:')
        for tf in range(self.min_tf, self.max_tf+1):
            self.generate_params(tf)
            progress = int(iter_count/(self.max_tf-self.min_tf)*100)

            try:
                self.solve(20,iterative=True)
                if self.solution['z'][-1,-1] is not None:
                    time.append(tf)
                    cost.append(np.log(self.dry_mass+self.fuel_mass)-self.solution['z'][-1,-1])
                print(f"    TIME:{tf} | COST:{np.log(self.dry_mass+self.fuel_mass)-self.solution['z'][-1,-1]} ({progress}%)")
            except Exception as ex:
                print(f"    TIME:{tf} | COST:inf (SOLVER FAILED:{ex}) ({progress}%)")
            finally:
                iter_count += 1
            
        if len(cost)==0 or len(time)==0:
            print('NO SOLUTION')
            quit()
        
        #a second check
        available_tf = time
        cost.clear()
        #print(f"AVAILABLE:{available_tf}")
        print('CHECKING:')
        for tf in available_tf:
            try:
                self.generate_params(tf)
                self.solve(self.N_tf, iterative=True)
                print(f"    TIME:{tf} | COST:{np.log(self.dry_mass+self.fuel_mass)-self.solution['z'][-1,-1]}")
                cost.append(np.log(self.dry_mass+self.fuel_mass)-self.solution['z'][-1,-1])
                break
            except Exception as ex:
                print(f"    TIME:{tf} | FALSE SOLUTION (SOLVER FAILED:{ex})")
        
        if len(cost)==0:
            print('NO SOLUTION')
            quit()
        print(f"OPTIMAL TIME:{tf}")
        if self.plot:plot_run3D(tf,self.solution['x'],self.solution['u'],self.solution['m'],self.solution['s'],self.solution['z'],self.S,self.Sk)

        return None