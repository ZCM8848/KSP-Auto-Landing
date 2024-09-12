import numpy as np
import time
import os

class GFOLD:
    #def __init__(self, v_data=None):
    #    print(os.getcwd())
    #    if v_data != None:
    #        self.set_params()
    
    #def set_params(self, v_data):
    #    self.Isp_inv = 1 / v_data['specific_impulse']
    #    self.alpha = 1 / 9.80665 / v_data['specific_impulse']
    #    self.G_max = v_data['max_structural_Gs']
    #    self.V_max = v_data['max_velocity']
    #    self.y_gs_cot = 1 / np.tan(v_data['glide_slope_cone'])
    #    self.p_cs_cos = np.cos(v_data['thrust_pointing_constraint'])
    #    self.m_wet = v_data['mass']
    #    self.m_wet_log = np.log(v_data['mass'])
    #    self.r1 = v_data['max_thrust'] * v_data['min_throttle']
    #    self.r2 = v_data['max_thrust'] * v_data['max_throttle']
    #    self.tf_ = v_data['tf']
    #    self.straight_fac = v_data['straight']
    #    
    #    self.x0 = v_data['x0']
    #    self.g = v_data['gravity']

    #    self.plot = v_data['plot']
    #    self.N = 250
    #    self.v_data = v_data
    def __init__(self):
        self.Isp_inv = 1 / np.load('.\\Solver\\inputs\\specific_impulse.npy')
        self.alpha = 1 / 9.80665 / np.load('.\\Solver\\inputs\\specific_impulse.npy')
        self.G_max = np.load('.\\Solver\\inputs\\max_structural_Gs.npy')
        self.V_max = np.load('.\\Solver\\inputs\\max_velocity.npy')
        self.y_gs_cot = 1 / np.tan(np.load('.\\Solver\\inputs\\glide_slope_cone.npy'))
        self.p_cs_cos = np.cos(np.load('.\\Solver\\inputs\\thrust_pointing_constraint.npy'))
        self.m_wet = np.load('.\\Solver\\inputs\\mass.npy')
        self.m_wet_log = np.log(np.load('.\\Solver\\inputs\\mass.npy'))
        self.r1 = np.load('.\\Solver\\inputs\\max_thrust.npy') * np.load('.\\Solver\\inputs\\min_throttle.npy')
        self.r2 = np.load('.\\Solver\\inputs\\max_thrust.npy') * np.load('.\\Solver\\inputs\\max_throttle.npy')
        self.tf_ = np.load('.\\Solver\\inputs\\tf.npy')
        self.straight_fac = np.load('.\\Solver\\inputs\\straight.npy')
        
        self.x0 = np.load('.\\Solver\\inputs\\x0.npy')
        self.g = np.load('.\\Solver\\inputs\\gravity.npy')

        self.N = 250
    def pack_data(self, N):
        dt = self.tf_ / N
        alpha_dt = self.alpha * dt
        t = np.linspace(0, (N-1) * dt, N)
        z0_term = self.m_wet - self.alpha * self.r2 * t
        z0_term_inv = (1 / z0_term).reshape(1, N)
        z0_term_log = np.log(z0_term).reshape(1, N)
        x0 = self.x0.reshape(6,1)
        g = self.g.reshape(3,1)
        sparse_params = np.array((alpha_dt, self.G_max, self.V_max, self.y_gs_cot, self.p_cs_cos, self.m_wet_log, self.r1, self.r2, self.tf_, self.straight_fac))
        sparse_params=sparse_params.reshape(len(sparse_params),1)
        return (x0,z0_term_inv,z0_term_log,g,sparse_params)
    
    def run_p3(self):
        import gfold_solver_p3 as solver3
        (x0,z0_term_inv,z0_term_log,g,sparse_params) = self.pack_data(self.N)
        res = solver3.cg_solve(x0 = x0, g_vec = g, z0_term_log = z0_term_log, z0_term_inv = z0_term_inv, 
            sparse_params=sparse_params)
        
        if res[1]['status'] == 'optimal':
            tf_m = self.tf_
            x = res[0]['var_x']
            for i in range(x.shape[1]):
                if (np.linalg.norm(x[0:3,i]) + np.linalg.norm(x[3:6,i])) < 0.1:
                    tf_m = i / x.shape[1] * self.tf_
                    break
            return tf_m
        else:
            #print(res)
            return self.tf_ #None
        
    def run_p4(self):
        m = 0
        import gfold_solver_p4 as solver4
        (x0,z0_term_inv,z0_term_log,g,sparse_params) = self.pack_data(self.N)
        res = solver4.cg_solve(x0 = x0, g_vec = g, z0_term_log = z0_term_log, z0_term_inv = z0_term_inv, 
            sparse_params=sparse_params)
        
        if res[1]['status'] == 'optimal':
            m = np.exp(res[0]['var_z'])
            return (self.tf_,res[0]['var_x'],res[0]['var_u'],m,res[0]['var_s'],res[0]['var_z'])
        else:
            #print(res)
            # = np.exp(res[0]['var_z'])
            return (self.tf_,res[0]['var_x'],res[0]['var_u'],m,res[0]['var_s'],res[0]['var_z'])#None

    def solve(self):
        start = time.time()
        
        tf_m = self.run_p3()
        if tf_m == None:
            print('p3 failed')
            return None
        self.tf_ = tf_m + 0.1 * self.straight_fac
        print('OPTIMAL TIME: ' + str(tf_m))
        res = self.run_p4()
        if res == None:
            print('p4 failed')
            return None
        print("SOLVED IN %f SECONDS" % (time.time() - start))
        (tf,x,u,m,s,z) = res
        np.save('.\\Solver\\\\results\\x.npy', x)
        np.save('.\\Solver\\results\\u.npy', u)
        return res
        
    
'''
#data should looks like this:
test_vessel = {
    'specific_impulse' : 203.94,
    'max_structural_Gs' : 3,
    'max_velocity' : 90,
    'glide_slope_cone' : np.radians(30),
    'thrust_pointing_constraint' : np.radians(45),
    'mass' : (2)*1e3 + (0.3)*1e3,
    'max_thrust' : 24000,
    'min_throttle' : 0.2, 'max_throttle' : 0.8,
    #'x0' : np.array([2400, 450, -330,  -10,  -40,   10]),
    'x0' : np.array([1400, 450, -330,  -20,  40,   40]),
    'gravity' : np.array([-3.71,0,0]),
    'tf' : 80,
    'straight' : 1,
    'plot' : True
}
'''
#from EvilPlotting import *
(tf,x,u,m,s,z) = GFOLD().solve()
#plot_run3D(tf,x,u,m,s,z, test_vessel)