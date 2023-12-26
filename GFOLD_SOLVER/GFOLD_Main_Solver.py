import numpy as np
from .EvilPlotting import *

'''

    This code can do both static runs (tests) AND initialize code gen.
    If doing code generation, you must still compile the generated code.

 PROBLEM 1: Minimum Landing Error (tf roughly solved)
 MINIMIZE : norm of landing error vector
 SUBJ TO  :
            0) initial conditions satisfied (position, velocity)
            1) final conditions satisfied (altitude, velocity)
            2) dynamics always satisfied
            3) x stays in cone at all times
            4) relaxed convexified mass and thrust constraints
            5) thrust pointing constraint
            6) sub-surface flight constraint

 PROBLEM 2: Minimum Fuel Use
 MAXIMIZE : landing mass, opt variables are dynamical and
 SUBJ TO  :
            0) same constraints as p1, plus:
            1) landing point must be equal or better than that found by p1
'''

VERSION = 1.1

test = 1  # are we doing a static run or a generation run?

from cvxpy import *
from cvxpygen.cpg import generate_code
import warnings
warnings.filterwarnings("ignore")


def GFOLD(_s_, _v_, Sk, Vk, S_, 
          prog_flag:str='p4', solver:int=0, plot:bool=False): # PRIMARY GFOLD SOLVER

    if solver == 0:
        solver = ECOS
    else:
        solver = SCS

    N_tf=250  # MUST BE FIXED FOR CODE GEN TO WORK

    sk,vk=Sk,Vk
    if not test:
        dt=     Parameter((1,1),name='dt') # determines tf implicitly dt = tf/N, tf = dt*N(const)
        S=      Parameter((1,17),name='S') # contains all parms_static scalar variables
        V=      Parameter((3,9),name='V') # contains all parms_static vect variables
        z0=     Parameter((1,N_tf),name='z0')
        z1=     Parameter((1,N_tf),name='z1')
        mu_1=   Parameter((1,N_tf),name='mu_1')
        mu_2=   Parameter((1,N_tf),name='mu_2')
        z0_term=Parameter((1,N_tf),name='z0_term')
        z1_term=Parameter((1,N_tf),name='z1_term')

    else:
        V=_v_ # for cvxpy testing
        S=_s_ # for cvxpy testing

        dt=Parameter((1,1),name='dt') # determines tf implicitly dt = tf/N,
                                    # tf = dt*N(const)

        dt.value = np.array([float(S[0,sk['tf']])/(N_tf)]).reshape((1,1))

        # Precalculate Z limits, then pass in as a PARAMETER

        z0=           Parameter((1,N_tf),name='z0')
        z1=           Parameter((1,N_tf),name='z1')
        mu_1=         Parameter((1,N_tf),name='mu_1')
        mu_2=         Parameter((1,N_tf),name='mu_2')
        z0_term=Parameter((1,N_tf),name='z0_term')
        z1_term=Parameter((1,N_tf),name='z1_term')

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

    if prog_flag=='p3':
        program = 3
    elif prog_flag=='p4':
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

    def run_p3(solver):
        print('-----------------------------')
        if test:

            objective=Minimize(norm(x[0:3,N_tf-1].reshape((3,1))-V[:,rf]))
            problem=Problem(objective,con)
            obj_opt=problem.solve(solver=solver,verbose=True)

        else:

            objective=Minimize(norm(x[0:3,N_tf-1]-V[:,rf]))
            problem=Problem(objective,con)
            obj_opt=generate_code(problem,'GFOLD_'+prog_flag+'_')

        print('-----------------------------')

        return obj_opt


    def run_p4(solver):
        print('-----------------------------')
        if test:
            
            #expression = 0
            #for i in range(N_tf):
            #    expression += norm(x[4:6,i])*(i**1.1/N_tf) # - rf[0:3,0]
            #expression += -z[0,N_tf-1]
            #objective=Minimize(expression)
            objective=Maximize(z[0,N_tf-1])
            problem=Problem(objective,con)
            obj_opt=problem.solve(solver=solver,verbose=True)

        else:

            objective=Maximize(z[0,N_tf-1])
            problem=Problem(objective,con)
            #obj_opt=problem.codegen('GFOLD_'+prog_flag)
            obj_opt = generate_code(problem,'GFOLD_'+prog_flag)
            
        print('-----------------------------')
        
        return obj_opt
    
    def run(flag):
        if flag == 'p3':
            opt = run_p3(solver=solver)
            return opt
        elif flag == 'p4':
            opt = run_p4(solver=solver)
            return opt

    
    opt = run(prog_flag)
    x=x.value
    u=u.value
    s=s.value
    z=z.value
    tf=(N_tf/norm(dt.value)).value#/
    try:
        m=list(map(np.exp,z[0].tolist()))
        if plot:plot_run3D(tf,x,u,m,s,z,S_,sk)
    except Exception as e:
        print(f'ERROR: {e}')
        print('THE PROBLEM IS INFEASIBLE')

    return {'x':x, 'u':u, 'tf':tf, 'opt':opt}
