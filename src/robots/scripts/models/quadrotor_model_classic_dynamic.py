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
from sympy import symbols, Matrix
from sympy.physics.mechanics import *
from numpy.linalg import *

from services_msgs.srv import DynamicSrv, DynamicSrvResponse

N_states = 13          # q = [x y z vx vy vz Q(4*1) Omega(3*1)]
N_inputs = 4           # u = [f(4-1)]
N_outputs = 4          # y = [x y z yaw]
N_ctrl_states = 3      # xi = [e_xi(3)]
N_par = 3              # p = [kf, ktau, m]
N_par_aux = 5          # p_aux = [l, g, Jx, Jy, Jz]

def hat_map(vec):
    x, y, z = vec
    # Construct the skew-symmetric matrix
    return se.DenseMatrix([
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]
    ])

def quat_norm2(q):
    return (q.T*q)[0]

def quat_to_mat(q):
    a, b, c, d = q[:]
    return se.DenseMatrix([
        [a**2+b**2-c**2-d**2, 2*b*c - 2*a*d, 2*a*c + 2*b*d],
        [2*a*d + 2*b*c, a**2-b**2+c**2-d**2, 2*c*d - 2*a*b],
        [2*b*d - 2*a*c, 2*a*b + 2*c*d, a**2-b**2-c**2+d**2],
    ]) / quat_norm2(q)

class Jetson(Model):

    def generate(self, N_lc, verbose=False, mode: Mode=Mode.NOGRAD, token="", overwrite=False) -> (JitParam):
        if not isinstance(mode, Mode):
            raise ValueError(f"mode must either be a member of the Mode enum")

        module_name = "".join([f"jitced_jetson_classic_dynamic_n{self.N_ctrl_points}_{mode.value}", f"_{token}" if token else "", ".so"])
        module_name_simu = "".join([f"jitced_jetson_classic_dynamic_n{self.N_ctrl_points}_{Mode.SIMU.value}", f"_{token}" if token else "", ".so"])

        N_traj_param = N_outputs * self.N_ctrl_points
        N_controllable = N_traj_param - 2 * N_outputs * N_lc
        self.control_mask = np.concatenate([np.zeros(N_lc), np.ones(self.N_ctrl_points - 2 * N_lc), np.zeros(N_lc)] * N_outputs).astype(bool)

        helpers = dict()
        system = dict()
        subs = dict()
        self._problem = pb = ODEproblem()

        # State variables
        # ---------------
        x, y, z, vx, vy, vz = dynamicsymbols('x y z vx vy vz')
        qw, qx, qy, qz, wx, wy, wz = dynamicsymbols('qw qx qy qz wx wy wz')
        q = pb.add_states("q", N_states)
        subs["q"] = [x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz], q

        # Parameters, forces and torques
        # ------------------------------
        m, Jx, Jy, Jz, l, kf, ktau, g = symbols('m Jx Jy Jz l kf ktau g')
        F = [symbols('F' + str(i)) for i in range(N_inputs)]
        T = [symbols('T' + str(i)) for i in range(N_inputs)]

        p = pb.new_parameter("p", N_par)
        subs["p"] = [kf, ktau, m], p
        p_c = pb.new_parameter("p_c", N_par)
        subs["p_c"] = [kf, ktau, m], p_c
        p_aux = pb.new_parameter("p_aux", N_par_aux)
        subs["p_aux"] = [l, g, Jx, Jy, Jz], p_aux

        # Inputs
        # ------
        u = pb.new_sym_matrix("u", N_inputs)  # u1 = w1**2

        # Allocation matrix
        # -----------------
        S = kf * se.DenseMatrix([
            [     1,      1,      1,         1],
            [     0,      l,      0,        -l],
            [    -l,     0,       l,        0],
            [  ktau,  -ktau,   ktau,     -ktau]])

        S_c = S.subs(*subs["p_aux"]).subs(*subs["p_c"])

        # aliases
        x = q[:3, :]
        v = q[3:6, :]
        Q = q[6:10, :]
        Omega = q[10:, :]
        l, g, Jx, Jy, Jz = p_aux
        m = p[2]

        # Trajectory
        # -----------------
        N_ctrl_points = self.N_ctrl_points
        # Controller : u = h(q, a, t, p_c)
        pos = pb.new_parameter("pos", 4, 1) #position of the waypoint (X, Y, Z, Yaw)
        vel = pb.new_parameter("vel", 4, 1)
        acc = pb.new_parameter("acc", 3, 1)
        ti = pb.new_parameter("ti", 1)
        tf = pb.new_parameter("tf", 1)

        # introducing helpers
        xd = pb.new_sym_matrix("xd", 3)
        vd = pb.new_sym_matrix("vd", 3)
        ad = pb.new_sym_matrix("ad", 3)
        yawd = pb.new_sym_matrix("yawd", 1)
        wyawd = pb.new_sym_matrix("wyawd", 1)
        helpers.update({
            xd[0]: pos[0,0],
            xd[1]: pos[1,0],
            xd[2]: pos[2,0],
            vd[0]: vel[0,0],
            vd[1]: vel[1,0],
            vd[2]: vel[2,0],
            ad[0]: acc[0,0],
            ad[1]: acc[1,0],
            ad[2]: acc[2,0],
            yawd: pos[3,0],
            wyawd: vel[3,0],
        })

        kx = pb.new_parameter("kx", 3, real=True)
        ki = pb.new_parameter("ki", 3, real=True)
        kv = pb.new_parameter("kv", 3, real=True)
        kR = pb.new_parameter("kR", 3, real=True)
        kOmega = pb.new_parameter("kOmega", 3, real=True)
        kx = se.diag(*kx)
        ki = se.diag(*ki)
        kv = se.diag(*kv)
        kR = se.diag(*kR)
        kOmega = se.diag(*kOmega)
        
        # Lee Controller
        # --------------

        ex = x - xd
        ev = v - vd
        
        norm = lambda mat: se.sqrt(np.sum(np.power(mat, 2), axis=None))
        e3 = se.DenseMatrix([0, 0, 1])

        b3d_eval = -kx * ex - kv * ev + m * g * e3 + m * ad
        b3d_eval = b3d_eval / norm(b3d_eval)
        b3d = pb.new_sym_matrix("b3d", 3)
        xc = se.DenseMatrix(3, 1, [se.cos(yawd), se.sin(yawd), 0.0])
        b2d = hat_map(b3d) * xc / norm(hat_map(b3d) * xc)
        b1d = hat_map(b2d) * b3d
        Rd = se.DenseMatrix([b1d.transpose(), b2d.transpose(), b3d.transpose()]).transpose()
        R = quat_to_mat(Q)
        eR = pb.new_sym_matrix("eR", 3)
        eR_eval = se.DenseMatrix([0, 0, 0])
        E = 0.5 * (Rd.transpose() * R - R.transpose() * Rd)
        eR_eval[0] = (E[2, 1] - E[1, 2])/2.
        eR_eval[1] = (E[0, 2] - E[2, 0])/2.
        eR_eval[2] = (E[1, 0] - E[0, 1])/2.
        eOmega = Omega # Omega_d = 0
        helpers[eR] = eR_eval
        helpers[b3d] = b3d_eval

        f_tmp = (-kx * ex - kv * ev + m * g * e3 + m * ad).transpose() * (R * e3)
        M_tmp = -kR * eR - kOmega * eOmega 
        
        U1, U2, U3, U4 = f_tmp, M_tmp[0], M_tmp[1], M_tmp[2],
        helpers[u] = h = S_c.inv() * se.DenseMatrix([U1, U2, U3, U4])

        g_xi = ex 

        # Dynamics of the system
        # ----------------------
        
        tmp = S*u  # using allocation matrix to get forces and torques
        U, Troll, Tpitch, Tyaw = tmp[0], tmp[1], tmp[2], tmp[3]

        f = se.DenseMatrix([
            [vx],
            [vy],
            [vz],
            [-0.5 * (wx * qx + wy * qy + qz * wz)],
            [ 0.5 * (wx * qw - wy * qz + qy * wz)],
            [ 0.5 * (wx * qz + wy * qw - qx * wz)],
            [-0.5 * (wx * qy - wy * qx - qw * wz)], 
            [(2 * U * ( qw * qy + qx * qz)) / m],
            [(2 * U * (-qw * qx + qy * qz)) / m],
            [(U * (qw ** 2 - qx ** 2 - qy ** 2 + qz ** 2)) / m - g], 
            [((Jy - Jz) / Jx) * wy * wz + Troll  / Jx],
            [((Jz - Jx) / Jy) * wx * wz + Tpitch / Jy],
            [((Jx - Jy) / Jz) * wx * wy + Tyaw   / Jz]])

        f = se.DenseMatrix(
            [f[:3, :], f[3 + 4:3 + 4 + 3, :], f[3:3 + 4, :], f[3 + 4 + 3:, :]])  # changing order to x, v, q, Omega
        f = f.subs(*subs["q"])
        f = f.subs(*subs["p_aux"]).subs(*subs["p"])
        u_int = pb.add_states("u_int", N_inputs)
        
        system.update({
            q: f,
            xi: g_xi,
            u_int: u,
        })

        self._problem.register_helpers(helpers)
        self._problem.register_system(system)


        ODE = self._problem.init_ODE(verbose=verbose, module_location=module_name, overwrite=overwrite)

        def _load(name, path):
            from jitcxde_common.modules import find_and_load_module
            self.jitced = find_and_load_module(name, path)

        def set_check_time_params(ODE, time_vector, **_):
            ODE["ti"] = ODE.t
            ODE["tf"] = time_vector[-1]

        ODE.register_pre_integration_callback(set_check_time_params)
        
        # default parameters

        ODE["p_c"] = np.array([5.9e-04, 1e-5/5.9e-04, 1.113])  # [kf, ktau, m]
        ODE["p"] = np.array([5.9e-04, 1e-5/5.9e-04, 1.113])  # [kf, ktau, m]
        ODE.param_alias.update({
            "kf": ("p", 0),
            "ktau": ("p", 1),
            "m": ("p", 2),
            "kf_c": ("p_c", 0),
            "ktau_c": ("p_c", 1),
            "m_c": ("p_c", 2), 
        })
        ODE["p_aux"] = np.array([0.23, 9.81, 0.015, 0.015, 0.007])  # [l, g, Jx, Jy, Jz]
        ODE.param_alias.update({
            "l": ("p_aux", 0),
            "g": ("p_aux", 1),
            "Jx": ("p_aux", 2),
            "Jy": ("p_aux", 3),
            "Jz": ("p_aux", 4),
        })

        # controller gains

        ODE["kx"] = np.array([20, 20, 25])
        ODE["kv"] = np.array([9, 9, 12])
        ODE["ki"] = np.array([0.08, 0.08, 0.08]) 
        ODE["kR"] = np.array([4.6, 4.6, 0.8])
        ODE["kOmega"] = np.array([0.5, 0.5, 0.08])

        ODE["ti"] = 0
        ODE["tf"] = 5
        self.ODE = ODE
        return ODE

    def set_default_state(self, init_waypoint, req ):
        ODE = self.ODE
        p_0 = np.zeros(3)
        v_0 = np.zeros(3)
        q_0 = np.zeros(4)
        w_0 = np.zeros(3)
        
        p_0[0] = req.init_robot_state.x
        p_0[1] = req.init_robot_state.y
        p_0[2] = req.init_robot_state.z
        
        v_0[0] = req.init_robot_state.vx
        v_0[1] = req.init_robot_state.vy
        v_0[2] = req.init_robot_state.vz
        
        q_0[0] = req.init_robot_state.qw
        q_0[1] = req.init_robot_state.qx
        q_0[2] = req.init_robot_state.qy
        q_0[3] = req.init_robot_state.qz
        
        w_0[0] = req.init_robot_state.wx
        w_0[1] = req.init_robot_state.wy
        w_0[2] = req.init_robot_state.wz

        ODE.set_default_initial_state({
            ODE.states["q"]: np.concatenate((p_0, v_0, q_0 , w_0)),
        })

    def compute_dynamic(self, req):

        import time
        start_time = time.time()

        #####################################################################################
        # START INITIALISATION
        #####################################################################################
        # Set the robot uncertain parameters
        self.ODE["m"] = req.model_params[0]
        self.ODE["kf"] = req.model_params[1]
        self.ODE["ktau"] = req.model_params[2]
        
        # Initialize the time
        self.ODE["ti"] = req.t_init
        self.ODE["tf"] = req.t_final
        
        # Initialize gains
        self.ODE["kx"] = req.gains[0].values
        self.ODE["kv"] = req.gains[1].values
        self.ODE["ki"] = req.gains[2].values
        self.ODE["kR"] = req.gains[3].values
        self.ODE["kOmega"] = req.gains[4].values

        self.ODE.apply_parameters() 

        #####################################################################################
        # START INTEGRATION
        #####################################################################################
        des_traj = [] # contains all the waypoints
        for des_state in req.desired_states:
            st = []
            st.append(des_state.x)
            st.append(des_state.y)
            st.append(des_state.z)
            st.append(des_state.yaw)
            st.append(des_state.vx)
            st.append(des_state.vy)
            st.append(des_state.vz)
            st.append(des_state.wyaw)
            st.append(des_state.ax)
            st.append(des_state.ay)
            st.append(des_state.az)
            des_traj.append(st)
        
        start_int = time.time()
        time_vector = self.integrate_along_trajectory(des_traj, req.time_vec, req.dt, req.integrator)
        # print("Sensitivity time: ", time.time() - start_int)             
        
        #####################################################################################
        # START TUBES COMPUTATIONS
        #####################################################################################
        q = self.ODE.last_result[:,self.ODE.states_indices["q"]]
        u_int = self.ODE.last_result[:, self.ODE.states_indices["u_int"]]  # last 2 states are the integral of u
        u_vec = np.diff(u_int, axis=0).T / np.diff(time_vector, axis=0).T
        
        response = SensitivitySrvResponse()
        
        response.executionTime = time.time() - start_int
        # print("All tubes computation: ", response.executionTime)
            
        for state in q:
            response.trajX.append(state[0])
            response.trajY.append(state[1])
            response.trajZ.append(state[2])
            response.trajVX.append(state[3])
            response.trajVY.append(state[4])
            response.trajVZ.append(state[5])
            response.trajQw.append(state[6])
            response.trajQx.append(state[7])
            response.trajQy.append(state[8])
            response.trajQz.append(state[9])
            response.trajWx.append(state[10])
            response.trajWy.append(state[11])
            response.trajWz.append(state[12])
        
        response.u1 = abs(u_vec[0])
        response.u2 = abs(u_vec[1])
        response.u3 = abs(u_vec[2])
        response.u4 = abs(u_vec[3])

        return response

    def integrate_along_trajectory(self, des_traj, time_vec, dt, integrator):
        if self.ODE is None:
            raise RuntimeError("model not generated")

        self.ODE.set_initial_value()
        time_vector_to_return = np.array([self.ODE["ti"]])

        previousTime = time_vec[0]
        for i in range(len(des_traj)):
            time_vector = np.linspace(previousTime, time_vec[i+1], 2)
            time_vector_to_return = np.append(time_vector_to_return,np.array([time_vec[i+1]]))

            pos = [[state[0]],
                    [state[1]],
                    [state[2]],
                    [state[3]]]
            
            vel = [[state[4]],
                    [state[5]],
                    [state[6]],
                    [state[7]]]
            
            acc = [[state[8]],
                    [state[9]],
                    [state[10]]]
            
            self.ODE["pos"] = pos
            self.ODE["vel"] = vel
            self.ODE["acc"] = acc
            self.ODE.apply_parameters()
            previousTime = time_vec[i+1]

            # RK4 integration
            if integrator == "RK4":
                # Old dopri5 integration
                # self.ODE.integrate_on_time_vector(time_vector)
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
    
    def integrate_along_flaten_trajectory(self, flatten_des_traj, dt):
        pass

    def optimize_trajectory(self, req):
        pass
    
    def optimize_mpc(self, req):
        pass

if __name__ == '__main__':
    model = Jetson(6)
    model.generate(3, verbose=True, overwrite=True)
