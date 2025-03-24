# Software License Agreement (BSD License)
# 
# Copyright (c) 2024, LAAS-CNRS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Simon WASIELA 

import sys
import os

# Get the current directory of this script file (script.py), which is /CAMP/src/robots/scripts
current_dir = os.path.dirname(__file__)
parent_dir1 = os.path.abspath(os.path.join(current_dir, '..'))
parent_dir2 = os.path.abspath(os.path.join(current_dir, '../../../../devel/lib/python3/dist-packages'))
# adding Scripts to the system path
sys.path.insert(0, parent_dir1)
sys.path.insert(0, parent_dir2)

from lib.sym_gen import JitParam, ODEproblem, t
from lib.base_model import Model, Mode
from math import comb
from math import *
import symengine as se
import numpy as np
import nlopt
import time
from sympy import symbols, Matrix
from sympy.physics.mechanics import *
from numpy.linalg import *

from scipy.spatial.transform import Rotation as R

from services_msgs.srv import SensitivitySrv, SensitivitySrvResponse
from services_msgs.srv import LocaloptSrv, LocaloptSrvResponse
from services_msgs.msg import DesiredState

N_states = 3           # q = [x y theta]
N_inputs = 2           # u = [wr wl]
N_outputs = 2          # y = [x y]
N_ctrl_states = 3      # xi = [xi_v xi_x xi_y]
N_par = 2              # p = [r b]

def pnorm(vec, p):
    pnorm = 0
    for elem in vec:
        pnorm += pow(elem,p)
    return pow(pnorm,(1/p))

class Unicycle(Model):

    def generate(self, N_lc, verbose=False, mode: Mode=Mode.NOGRAD, token="", overwrite=False):
        if not isinstance(mode, Mode):
            raise ValueError(f"mode must either be a member of the Mode enum")

        module_name = "".join([f"jitced_unicycle_kinematic_DFL_n{self.N_ctrl_points}_{mode.value}", f"_{token}" if token else "", ".so"])

        helpers = dict()
        system = dict()
        subs = dict()
        self._problem = pb = ODEproblem()
        
        ti = self._problem.new_parameter("ti", 1, real=True)
        tf = self._problem.new_parameter("tf", 1, real=True)

        # parameters (q, u, p, xi)
        q = self._problem.add_states("q", N_states)
        u = self._problem.new_sym_matrix("u", N_inputs)
        p = self._problem.new_parameter("p", N_par)
        xi = self._problem.add_states("xi", N_ctrl_states)

        # system definition f() = dq/dt
        S = se.DenseMatrix([
            [p[0], p[0]],
            [p[0]/p[1], -p[0]/p[1]]
        ])/2

        f = se.DenseMatrix([
            [se.cos(q[2]), 0],
            [se.sin(q[2]), 0],
            [0, 1]
        ]) * S * u

        # 2D Controller : dxi = g(xi, q, a, t, p_c) ; u = h(xi, q, a, t, p_c)
        # introducing helpers
        pos = self._problem.new_parameter("pos", 2, 1) #position of the waypoint (X, Y)
        vel = self._problem.new_parameter("vel", 2, 1)
        acc = self._problem.new_parameter("acc", 2, 1)
        rd = self._problem.new_sym_matrix("rd", 2)
        vd = self._problem.new_sym_matrix("vd", 2)
        ad = self._problem.new_sym_matrix("ad", 2)
        helpers.update({
            rd[0]: pos[0,0],
            rd[1]: pos[1,0],
            vd[0]: vel[0,0],
            vd[1]: vel[1,0],
            ad[0]: acc[0,0],
            ad[1]: acc[1,0],
        })

        p_c = self._problem.new_parameter("p_c", N_par)
        k_c = self._problem.new_parameter("k_c", 3, real=True)
        r = se.DenseMatrix(q[0:2])
        
        Sc = S.subs(p, p_c)
        r_xi_dot = se.DenseMatrix([se.cos(q[2])*xi[0], se.sin(q[2])*xi[0]])
        eta = k_c[2]*(vd - r_xi_dot) + k_c[0]*(rd - r) + k_c[1]*xi[1:3, :]

        A = se.DenseMatrix([
            [se.cos(q[2]), -xi[0]*se.sin(q[2])],
            [se.sin(q[2]),  xi[0]*se.cos(q[2])]
        ])
        g = se.DenseMatrix([
            se.DenseMatrix([[1, 0]]) * A.inv() * eta,
            rd - r
        ])

        h = Sc.inv()*se.DenseMatrix([
            xi[0],
            se.DenseMatrix([[0, 1]]) * A.inv() * eta
        ])

        h.simplify()
        
        helpers[u] = h
        u_int = self._problem.add_states("u_int", N_inputs)
            
        f.simplify()
        g.simplify()
        
        system.update({
            q: f,
            xi: g,
            u_int: u,
        })

        if mode != Mode.SIMU:

            PI = self._problem.add_states("PI", N_states, N_par)
            PI_xi = self._problem.add_states("PI_xi", N_ctrl_states, N_par)
            TH_val = self._problem.add_states("TH_int",  N_inputs, N_par) # integral of Th

            # helpers function (derivatives, TH, u evaluations)
            TH = self._problem.new_sym_matrix("TH", (N_inputs, N_par))

            df_dq = self._problem.new_sym_matrix("df_dq", (N_states, N_states))
            df_du = self._problem.new_sym_matrix("df_du", (N_states, N_inputs))
            df_dp = self._problem.new_sym_matrix("df_dp", (N_states, N_par))
            dg_dq = self._problem.new_sym_matrix("dg_dq", (N_ctrl_states, N_states))
            dg_dxi = self._problem.new_sym_matrix("dg_dxi", (N_ctrl_states, N_ctrl_states))
            dh_dq = self._problem.new_sym_matrix("dh_dq", N_inputs, N_states)
            dh_dxi = self._problem.new_sym_matrix("dh_dxi", N_inputs, N_ctrl_states)

            helpers.update({
                df_dq : f.jacobian(q),
                df_du : f.jacobian(u),
                df_dp : f.jacobian(p),
                dg_dq : g.jacobian(q),
                dg_dxi: g.jacobian(xi),
                dh_dq : h.jacobian(q),
                dh_dxi: h.jacobian(xi),
                TH    : dh_dq*PI + dh_dxi*PI_xi,
            })

            system.update({
                PI: df_dq*PI + df_du*TH + df_dp,
                PI_xi: dg_dq*PI + dg_dxi*PI_xi,
                TH_val: TH,
            })
        if mode == mode.GRAD:
            raise NotImplementedError("There is no plan to implement the gradient version for now")

        self._problem.register_helpers(helpers)
        self._problem.register_system(system)

        ODE = self._problem.init_ODE(verbose=verbose, module_location=module_name, overwrite=overwrite)

        # nominal parameters
        r_M = 0.1
        b_M = 0.4
        ODE["p"] = [r_M, b_M]
        ODE["p_c"] = [r_M, b_M]
        ODE["k_c"] = [4.0, 0.8, 4.0]
        
        ODE.param_alias.update({
            "r_M": ("p", 0),
            "b_M": ("p", 1),
            "r_M_c": ("p_c", 0),
            "b_M_c": ("p_c", 1),
        })

        def set_check_time_params(ODE, time_vector, **_):
            ODE["ti"] = ODE.t
            ODE["tf"] = time_vector[-1]

        ODE.register_pre_integration_callback(set_check_time_params)
        self.ODE = ODE
        return ODE

    def set_initial_state(self, req):
        ODE = self.ODE
        
        x0 = req.init_robot_state.x
        y0 = req.init_robot_state.y

        vx0 = req.init_robot_state.vx
        vy0 = req.init_robot_state.vy
        
        init_quat = [req.init_robot_state.qx, req.init_robot_state.qy, req.init_robot_state.qz, req.init_robot_state.qw]
        r = R.from_quat(init_quat)     
         
        # Convert to Euler angles
        euler_angles = r.as_euler('xyz', degrees=False)

        q_0 = [x0, y0, euler_angles[2]]
        xi_0 = req.initial_xi
        ODE.set_default_initial_state({
            ODE.states["xi"]: xi_0,
            ODE.states["q"]: q_0,
            ODE.states["PI"]: req.initial_PI,
            ODE.states["PI_xi"]: req.initial_PI_xi,
        })
    
    def compute_sensitivity(self, req):

        import time
        start_time = time.time()

        #####################################################################################
        # START INITIALISATION
        #####################################################################################
        
        # Initialize the delta p ranges for the tubes computation
        sensitivity_parameters_dict = { "r_M": 0.1,
                                        "b_M": 0.4 }

        deviation = 0.03  # 0.1 = 10% deviation on parameters with non-zero nominal value
        par_range = []
        for key in sensitivity_parameters_dict:
            val = sensitivity_parameters_dict[key]
            par_range.append((val*deviation)**2)
        W_range = np.diag(par_range)  # this matrix is used for sensitivity norm computation, see, 'PRG_sens_norm.pdf'

        # Initialize robot state
        self.set_initial_state(req)
            
        # Set the robot uncertain parameters
        self.ODE["r_M"] = req.model_params[0]
        self.ODE["b_M"] = req.model_params[1]
        
        # Initialize the time
        self.ODE["ti"] = req.t_init
        self.ODE["tf"] = req.t_final
        
        # Initialize gains
        self.ODE["k_c"] = req.gains[0].values

        self.ODE.apply_parameters() 

        #####################################################################################
        # START INTEGRATION
        #####################################################################################
        des_traj = [] # contains all the waypoints
        for des_state in req.desired_states:
            st = []
            st.append(des_state.x)
            st.append(des_state.y)
            st.append(des_state.vx)
            st.append(des_state.vy)
            st.append(des_state.ax)
            st.append(des_state.ay)
            des_traj.append(st)
        
        start_int = time.time()
        time_vector = self.integrate_along_trajectory(des_traj, req.time_vec, req.dt, req.integrator)
        # print("Sensitivity time: ", time.time() - start_int)   

        #####################################################################################
        # START TUBES COMPUTATIONS
        #####################################################################################
        PImatrices = self.ODE.last_result[:,self.ODE.states_indices["PI"]]
        PI_xi_matrices = self.ODE.last_result[:,self.ODE.states_indices["PI_xi"]]
        THmatrices = self.ODE.last_result[:,self.ODE.states_indices["TH_int"]]
        q = self.ODE.last_result[:,self.ODE.states_indices["q"]]
        xi = self.ODE.last_result[:,self.ODE.states_indices["xi"]]
        u_int = self.ODE.last_result[:, self.ODE.states_indices["u_int"]]  # last 2 states are the integral of u
        u_vec = np.diff(u_int, axis=0).T / np.diff(time_vector, axis=0).T
        
        PI_reshape = np.reshape(PImatrices, (-1, N_states, N_par))  # get integral of PI
        TH_reshape = np.reshape(THmatrices, (-1, N_inputs, N_par))  # get integral of TH
        N_time, N_dim, _ = np.shape(TH_reshape)  # get sizes of interest from integral of Theta (input sensitivity)
        
        response = SensitivitySrvResponse()

        # Compute the tube along the state space
        for matrix in PI_reshape:
            radii, lambda_max = self.compute_tubes(matrix, W_range)
            response.ellipsoid_alongX.append(radii[0])
            response.ellipsoid_alongY.append(radii[1])
            response.radii_lambda.append(lambda_max)
        
        # Compute the tube along the input space
        for i in range(N_time - 1):
            TH = np.subtract(TH_reshape[i + 1, :, :], TH_reshape[i, :, :]) / (time_vector[i + 1] - time_vector[i])  # compute Theta for one step
            radii, _ = self.compute_tubes(TH, W_range)
            response.ellipsoid_u1.append(radii[0])
            response.ellipsoid_u2.append(radii[1])
            
        # Pad the missing state due to the integral with the first value
        response.ellipsoid_u1.insert(0, 0.0)
        response.ellipsoid_u2.insert(0, 0.0)
        
        response.executionTime = time.time() - start_int
        # print("All tubes computation: ", response.executionTime)
   
        # The PI matrix at the final state
        response.final_PI = PImatrices[-1]
        response.final_PI_xi = PI_xi_matrices[-1]
        
        response.final_xi = xi[-1]
        
        for state in q:
            response.trajX.append(state[0])
            response.trajY.append(state[1])
            
            # Create a Rotation object from Euler angles
            rot = R.from_euler('xyz', [0.0, 0.0, state[2]], degrees=False)
            # Convert to quaternion
            quaternion = rot.as_quat()
            response.trajQx.append(quaternion[0])
            response.trajQy.append(quaternion[1])
            response.trajQz.append(quaternion[2])
            response.trajQw.append(quaternion[3]) 
            
        response.trajVX.append(req.init_robot_state.vx)
        response.trajVY.append(req.init_robot_state.vy)
        for st in req.desired_states:
            response.trajVX.append(st.vx)
            response.trajVY.append(st.vy)
        
        # Pad the missing state due to the integral with the first value
        u_vec_padd0 = np.insert(u_vec[0], 0, 0.0)
        u_vec_padd1 = np.insert(u_vec[1], 0, 0.0)
        
        if(len(u_vec_padd0)>2):
            u_vec_padd0[1] = u_vec_padd0[2] / 2
            u_vec_padd1[1] = u_vec_padd1[2] / 2
        
        response.u1 = u_vec_padd0
        response.u2 = u_vec_padd1
        
        return response
    
    def compute_tubes(self, matrix, W_range):
        N_dim, N_params = np.shape(matrix)  # get sizes of interest from integral of Theta (input sensitivity)
        ei = np.eye(N_dim)  # base vectors in data space

        mat = matrix @ W_range @ matrix.T  # kernel matrix of the input sensitivity
    
        geometric_lambda_max = 0
        ri = []
 
        for k in range(N_dim):  # each state component
            ri.append(np.sqrt(ei[:, k].T @ mat @ ei[:, k])) 
        
        geometric_lambda_max = pnorm(ri[0:2],8) 
        
        return (ri, geometric_lambda_max)

    def integrate_along_trajectory(self, des_traj, time_vec, dt, integrator):
        if self.ODE is None:
            raise RuntimeError("model not generated")

        self.ODE.set_initial_value()
        time_vector_to_return = np.array([self.ODE["ti"]])
        
        previousTime = self.ODE["ti"]
        actualTime = dt
        
        prev_vel_x = 0
        prev_vel_y = 0
        
        for state in des_traj:
            time_vector = np.linspace(previousTime, actualTime, 2)
            time_vector_to_return = np.append(time_vector_to_return,np.array([actualTime]))
            
            q_current = self.ODE.last_result[:,self.ODE.states_indices["q"]]

            pos = [[state[0]],
                    [state[1]]]
            
            vel = [[state[2]],
                    [state[3]]]
            
            acc = [[state[4]],
                    [state[5]]]
            
            self.ODE["pos"] = pos
            self.ODE["vel"] = vel
            self.ODE["acc"] = acc
            self.ODE.apply_parameters()
            previousTime = actualTime
            actualTime += dt
            
            # RK4 integration
            if integrator == "RK4":
                states = np.zeros((len(time_vector), self.ODE.n))
                states[0, :] = self.ODE.last_result[-1]
                self.ODE.time_points = np.concatenate((self.ODE.time_points, time_vector[1:]))
                for i, t in enumerate(time_vector[1:], start=1):
                    h = time_vector[1]-time_vector[0]
                    k1 = self.ODE.f(t,states[0, :])
                    k2 = self.ODE.f(t+(h/2), states[0, :] + (h/2)*k1)  
                    k3 = self.ODE.f(t+(h/2), states[0, :] + (h/2)*k2)   
                    k4 = self.ODE.f(t+h, states[0, :] + h*k3) 
                    states[i, :] = states[0, :] + (h/6)*(k1 + 2*k2 + 2*k3 + k4)
                self.ODE.last_result = np.concatenate((self.ODE.last_result, states[1:, :]), axis=0)

            # RK2 integration
            if integrator == "RK2":
                states = np.zeros((len(time_vector), self.ODE.n))
                states[0, :] = self.ODE.last_result[-1]
                self.ODE.time_points = np.concatenate((self.ODE.time_points, time_vector[1:]))
                for i, t in enumerate(time_vector[1:], start=1):
                    h = time_vector[1]-time_vector[0]
                    states[i, :] = states[0, :] + h*self.ODE.f(t+h/2, states[0, :] + (h/2)*self.ODE.f(t,states[0, :]))
                self.ODE.last_result = np.concatenate((self.ODE.last_result, states[1:, :]), axis=0)
            
            # Euler integration
            if integrator == "Euler":
                states = np.zeros((len(time_vector), self.ODE.n))
                states[0, :] = self.ODE.last_result[-1]
                self.ODE.time_points = np.concatenate((self.ODE.time_points, time_vector[1:]))
                for i, t in enumerate(time_vector[1:], start=1):
                    states[i, :] = states[0, :] + (time_vector[1]-time_vector[0])*self.ODE.f(t,states[0, :])
                self.ODE.last_result = np.concatenate((self.ODE.last_result, states[1:, :]), axis=0)
            
        return time_vector_to_return
    
    def integrate_along_flatten_trajectory(self, flatten_des_traj, dt):
        pass

    def optimize_trajectory(self, req):
        pass
    
    def optimize_mpc(self, req):
        pass
    
def test_integration_time(model):
    # Test integration time  
    ##################################################################################################### 
    
    # Initialize the delta p ranges for the tubes computation
    sensitivity_parameters_dict = { "r_M": 0.1,
                                    "b_M": 0.4 }

    deviation = 0.03  # 0.1 = 10% deviation on parameters with non-zero nominal value
    par_range = []
    for key in sensitivity_parameters_dict:
        val = sensitivity_parameters_dict[key]
        par_range.append((val*deviation)**2)
    W_range = np.diag(par_range)  # this matrix is used for sensitivity norm computation, see, 'PRG_sens_norm.pdf'
        
    total_time = 0.0
    # Compute for 100 trajectories
    nb_traj = 100
    for i in range(0,nb_traj):
        # Initialize robot state
        ODE = model.ODE
        x0 = 0.0
        y0 = 0.0

        vx0 = 0.0
        vy0 = 0.0
        
        init_quat = [0.0, 0.0, 0.0, 1.0]
        r = R.from_quat(init_quat)

        # Convert to Euler angles
        euler_angles = r.as_euler('xyz', degrees=False)

        q_0 = [x0, y0, euler_angles[2]]
        xi_0 = [0.01, 0.0, 0.0]
        ODE.set_default_initial_state({
            ODE.states["xi"]: xi_0,
            ODE.states["q"]: q_0,
        })
        
        # Initialize the time
        model.ODE["ti"] = 0.0
        model.ODE["tf"] = nb_traj*0.05
        
        model.ODE.apply_parameters() 
    
        # Create a 100 states trajectory
        des_traj = []
        nb_states = 100
        for j in range(0,nb_states):
            st = []
            st.append(j*0.01)
            st.append(j*0.01)
            st.append(0.01)
            st.append(0.01)
            st.append(0.0)
            st.append(0.0)
            des_traj.append(st)
            
        start_int = time.time()
        time_vector = model.integrate_along_trajectory(des_traj, 0.05)
        print("Sensitivity time: ", time.time() - start_int)   
        
        #####################################################################################
        # START TUBES COMPUTATIONS
        #####################################################################################
        PImatrices = model.ODE.last_result[:,model.ODE.states_indices["PI"]]
        THmatrices = model.ODE.last_result[:,model.ODE.states_indices["TH_int"]]
        q = model.ODE.last_result[:,model.ODE.states_indices["q"]]
        u_int = model.ODE.last_result[:, model.ODE.states_indices["u_int"]]  # last 2 states are the integral of u
        u_vec = np.diff(u_int, axis=0).T / np.diff(time_vector, axis=0).T
        
        PI_reshape = np.reshape(PImatrices, (-1, N_states, N_par))  # get integral of TH
        TH_reshape = np.reshape(THmatrices, (-1, N_inputs, N_par))  # get integral of TH
        N_time, N_dim, _ = np.shape(TH_reshape)  # get sizes of interest from integral of Theta (input sensitivity)

        # Compute the tube along the state space
        for matrix in PI_reshape:
            radii, lambda_max = model.compute_tubes(matrix, W_range)
        
        # Compute the tube along the input space
        for i in range(N_time - 1):
            TH = np.subtract(TH_reshape[i + 1, :, :], TH_reshape[i, :, :]) / (time_vector[i + 1] - time_vector[i])  # compute Theta for one step
            radii, _ = model.compute_tubes(TH, W_range)
        
        print("All tubes computation: ", time.time() - start_int)
        
        total_time += time.time() - start_int  
    
    print("Mean computing time: ", total_time/100)
    ##################################################################################################### 


if __name__ == '__main__':
    model = Unicycle(10)
    model.generate(3, verbose=False, overwrite=True)
    
    # Test integration time
    test_integration_time(model)
