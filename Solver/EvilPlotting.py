import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def plot_run3D(tf, x, u, m, s, z, S,Sk):

    print('tf',tf)
    t = np.linspace(0,tf,num=len(m))

    r = np.array(x[0:3,:])
    v = np.array(x[3:6,:])
    z = np.array(z)
    s = np.array(s)
    u = np.array(u)

    print('t',t.shape)
    print('r',r.shape)
    print('v',v.shape)
    print('u',u.shape)
    print('m',len(m))
    print('s',s.shape)
    print('z',z.shape)

    if t.shape==() or r.shape==() or v.shape==() or u.shape==():
        print('data actually empty')
        return

    Th= [np.linalg.norm(u[:,i])*m[i] for i in range(len(v.T))]
    vnorm = [np.linalg.norm(vel) for vel in v.T]

    #u_dirs_1 = [90 - np.degrees(np.atan2(u[0,n], u[1,n])) for n in range(N)]
    #u_dirs_2 = [90 - np.degrees(np.atan2(u[0,n], u[2,n])) for n in range(N)]

    traj = plt.figure()
    ax = traj.add_axes(Axes3D(traj))
    ax.set_aspect('equal')

    r_= np.linspace(0, max(max(r[1,:]),max(r[2,:])), 7)
    a_= np.linspace(0, 2*np.pi, 20)
    R, P = np.meshgrid(r_, a_)
    X, Y, Z = R*np.cos(P), R*np.sin(P), R*(np.tan(S[0,Sk['y_gs']]))
    #X,Y,Z=R*np.cos(P), R*np.sin(P),((R**2 - 1)**2)

    #ax.plot(x(t),y(t),z(t),label='Flight Path')
    ax.plot(r[1,:],r[2,:],r[0,:],label='Flight Path')
    ax.plot_surface(X, Y, Z, cmap=plt.cm.YlGnBu_r,alpha=0.6)

    # Tweak the limits and add latex math labels.

    ax.set_xlabel(r'$x{1}$')
    ax.set_ylabel(r'$x{2}$')
    ax.set_zlabel(r'$x{0}$')

    ax.legend()

    f = plt.figure()
    ax = f.add_subplot(511)

    plt.plot(t,vnorm)
    y=str(S[0,Sk['V_max']])
    x=np.array(range(0,int(max(t))))
    plt.plot(x,eval('0*x+'+y))
    plt.title('Velocity Magnitude (m/s)')

    plt.subplot(5,1,2)
    plt.plot(t,r[0,:])
    plt.title('Altitude (m)')

    plt.subplot(5,1,3)
    plt.plot(t,m)
    plt.title('Mass (kg)')

    plt.subplot(5,1,4)
    plt.plot(t,Th)
    y=str(S[0,Sk['T_max']])
    x=np.array(range(0,int(max(t))))
    plt.plot(x,eval('0*x+'+y))
    plt.title('Thrust (N)')

    z0_term = (S[0,Sk['m_wet']] - S[0,Sk['alpha']] * S[0,Sk['r2']])  # see ref [2], eq 34,35,36
    z1_term = (S[0,Sk['m_wet']] - S[0,Sk['alpha']] * S[0,Sk['r1']])
    lim=[]
    lim2=[]
    n=0
    z=z.flatten()
    for t_ in t:
        if t_ > 0:
            try:
                v = S[0,Sk['r1']]/(z0_term*t_) * (1 - (z[n] - np.log(z0_term*t_)))
            except ZeroDivisionError:
                v = 0
            lim.append( v )
            try:
                v = S[0,Sk['r1']]/(z1_term*t_) *(1 - (z[n] - np.log(z0_term*t_)) + (1/2)*(z[n] - np.log(z0_term*t_))**2 )
            except ZeroDivisionError:
                v = 0
            lim2.append( v )
        else:
            lim.append(0)
            lim2.append(0)
        n+=1
    lim = np.array(lim).flatten()
    plt.subplot(5,1,5)
    plt.plot(t,lim)
    plt.plot(t,lim2)
    s = s.flatten()
    if s.shape == (1,65):
        s.reshape((65,))
        print('reshape',s)
    #print('s',s)
    plt.plot(t,s)
    plt.title('Sigma Slack')

    plt.tight_layout()
    plt.subplots_adjust(hspace=0)
    plt.show()
