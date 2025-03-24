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
import sympy as sy
import numpy as np
import random
from sympy import symbols, Matrix, DiracDelta, diff, simplify
from sympy.physics.mechanics import *
from numpy.linalg import *

from services_msgs.srv import DynamicSrv, DynamicSrvResponse
from services_msgs.msg import *

N_states = 13          # q = [x y z vx vy vz Q(4*1) Omega(3*1)]
N_inputs = 4           # u = [f(4-1)]
N_par = 6            # p = [m, gx, gy, Jx, Jy, Jz]
N_par_aux = 5          # p_aux = [l, g, kf, ktau, gz]
    
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

def custom_simplify(expr):
    # expr = sy.expand(expr)
    expr = sy.factor(expr)
    expr = sy.simplify(expr)
    return expr

class Jetson(Model):

    def generate(self, N_lc, verbose=False, mode: Mode=Mode.NOGRAD, token="", overwrite=False) -> (JitParam):
        if not isinstance(mode, Mode):
            raise ValueError(f"mode must either be a member of the Mode enum")

        module_name = "".join([f"jitced_jetson_CoM_dynamic_n{self.N_ctrl_points}_{mode.value}", f"_{token}" if token else "", ".so"])
        module_name_simu = "".join([f"jitced_jetson_CoM_dynamic_n{self.N_ctrl_points}_{Mode.SIMU.value}", f"_{token}" if token else "", ".so"])

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
        
        # Inputs
        # ------
        u = pb.new_sym_matrix("u", N_inputs)  # u1 = w1**2

        # Parameters, forces and torques
        # ------------------------------
        m, Jx, Jy, Jz, gx, gy, gz, l, kf, ktau, g = symbols('m Jx Jy Jz gx gy gz l kf ktau g')
        F = [symbols('F' + str(i)) for i in range(N_inputs)]
        T = [symbols('T' + str(i)) for i in range(N_inputs)]

        p = pb.new_parameter("p", N_par)
        subs["p"] = [m, gx, gy, Jx, Jy, Jz], p
        p_c = pb.new_parameter("p_c", N_par)
        subs["p_c"] = [m, gx, gy, Jx, Jy, Jz], p_c
        p_aux = pb.new_parameter("p_aux", N_par_aux)
        subs["p_aux"] = [l, g, kf, ktau, gz], p_aux

        # aliases
        # ------------------------------
        x = q[:3, :]
        v = q[3:6, :]
        Q = q[6:10, :]
        Omega = q[10:, :]
        l, g, kf, ktau, gz = p_aux
        
        # Sytem
        m, gx, gy, Jx, Jy, Jz = p
        #Controller
        m_c, gx_c, gy_c, Jx_c, Jy_c, Jz_c = p_c

        # Allocation matrix for the controller (uses p_c instead of p):
        # -----------------
        S_c = kf * se.DenseMatrix([
            [       1,         1,        1,           1],
            [   -gy_c,  l - gy_c,    -gy_c, -(l + gy_c)],
            [gx_c - l,      gx_c, l + gx_c,        gx_c],
            [  ktau,   -ktau,   ktau,     -ktau]])

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
        kv = pb.new_parameter("kv", 3, real=True)
        kR = pb.new_parameter("kR", 3, real=True)
        kOmega = pb.new_parameter("kOmega", 3, real=True)
        kx = se.diag(*kx)
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
        
        f_tmp.simplify()
        M_tmp.simplify()
        
        U1, U2, U3, U4 = f_tmp, M_tmp[0], M_tmp[1], M_tmp[2],
        helpers[u] = h = S_c.inv() * se.DenseMatrix([U1, U2, U3, U4])
        
        # Dynamics of the system with a shift in CoM
        # ----------------------
        
        # Orientation and inertia:
        Q_mat = se.DenseMatrix([qw, qx, qy, qz])  # quaternion orienting the body frame
        bRw = quat_to_mat(Q_mat)  # rotation matrix of quaternion from Fw to Fb
        wRb = bRw.transpose()  # from Fb to Fw
        w = se.DenseMatrix([wx, wy, wz])  # angular velocity
        wskew = hat_map(w)  # cross product operator of w
        J = se.DenseMatrix([[Jx, 0, 0], [0, Jy, 0], [0, 0, Jz]])  # inertia matrix at Gb

        # Allocation matrix at Ob:
        S = kf * se.DenseMatrix([
            [   1,     1,    1,     1],
            [   0,     l,    0,    -l],
            [  -l,     0,    l,     0],
            [ktau, -ktau, ktau, -ktau]])

        # Mixing inputs with allocation matrix:
        FM = S * u
        FM.simplify()
        F_thrust, M_xb, M_yb, M_zb = FM[0], FM[1], FM[2], FM[3]

        # Place points in body frame (position from Ob):
        CoMoff = se.DenseMatrix([gx, gy, gz])  # CoM offset from geometric center Ob
        CoMoffskew = hat_map(CoMoff)  # cross product operator of CoMoff

        # List all forces (expressed in Fw):
        Weight = se.DenseMatrix([0, 0, - m * g])  # weight applied at Gb
        Thrust = wRb.transpose() * se.DenseMatrix([0, 0, F_thrust])  # total thrust applied at Ob
        FictF = m * wRb.transpose() * wskew * wskew * CoMoff  # fictitious forces
        F_tot = Thrust + Weight + FictF  # sum used for screw theory
        F_tot.simplify()

        # List all moments (at Ob, expressed in Fb):
        PropM = se.DenseMatrix([M_xb, M_yb, M_zb])  # total torque from the propellers at Ob
        WeightM = bRw * CoMoffskew * Weight  # moment from weight at Ob
        FictM = bRw * wskew * J * w  # fictitious moments at Ob
        M_tot = PropM + WeightM + FictM  # in the body frame Fb
        M_tot.simplify()
        
        # Vectors and matrices for dynamics computation:
        FM_tot = se.DenseMatrix([F_tot, M_tot])  # total forces and moments at Ob (in Fw, Fb respectively)

        SI = se.DenseMatrix([
            [m, 0, 0, 0, m * gz, -m * gy],
            [0, m, 0, -m * gz, 0, m * gx],
            [0, 0, m, m * gy, -m * gx, 0],
            [0, -m * gz, m * gy, Jx, 0, 0],
            [m * gz, 0, -m * gx, 0, Jy, 0],
            [-m * gy, m * gx, 0, 0, 0, Jz]])  # spatial inertia matrix
        SIinv = SI.inv()
        
        # # Loop through each element in the matrix for simplification
        # # Takes a really long time
        # for i in range(SIinv.rows):
        #     for j in range(SIinv.cols):
        #         # Count operations before simplification (SymEngine)
        #         print(f"Ops before: {se.count_ops(SIinv[i, j])}")
        #         # Convert to SymPy
        #         sympy_expr = sy.sympify(str(SIinv[i, j]))
        #         # Simplify
        #         sympy_expr_simp = custom_simplify(sympy_expr)
        #         # Convert back to SymEngine
        #         simplified_expr_symengine = se.sympify(sympy_expr_simp)
        #         # Update the matrix element
        #         SIinv[i, j] = simplified_expr_symengine
        #         # Count operations after simplification (SymEngine)
        #         print(f"Ops after: {se.count_ops(SIinv[i, j])}")
              
        ddrdw = SIinv * FM_tot
        
        # # Loop through each element in ddrdw to simplify
        # # Takes a really long time
        # for i in range(ddrdw.rows):
        #     # Count operations before simplification (SymEngine)
        #     print(f"Ops before ddrdw: {se.count_ops(ddrdw[i])}")
        #     # Convert the SymEngine expression to SymPy
        #     sympy_expr = sy.sympify(str(ddrdw[i]))
        #     # Simplify
        #     sympy_expr_simp = custom_simplify(sympy_expr)
        #     # Convert back to SymEngine
        #     simplified_expr_symengine = se.sympify(factored_expr_simpl)
        #     # Update the vector element with the simplified expression
        #     ddrdw[i] = simplified_expr_symengine
        #     # Count operations after simplification (SymEngine)
        #     print(f"Ops after ddrdw: {se.count_ops(ddrdw[i])}")
        
        f = se.DenseMatrix([
            [vx],
            [vy],
            [vz],
            [ddrdw[0]],
            [ddrdw[1]],
            [ddrdw[2]],
            [-0.5 * (wx * qx + wy * qy + qz * wz)],
            [0.5 * (wx * qw - wy * qz + qy * wz)],
            [0.5 * (wx * qz + wy * qw - qx * wz)],
            [-0.5 * (wx * qy - wy * qx - qw * wz)],
            [ddrdw[3]],
            [ddrdw[4]],
            [ddrdw[5]]])

        f = f.subs(*subs["q"])
        f = f.subs(*subs["p_aux"]).subs(*subs["p"])
        u_int = pb.add_states("u_int", N_inputs)
        
        system.update({
            q: f,
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

        ODE["p_c"] = np.array([1.113, 0.0, 0.0, 0.015, 0.015, 0.007])  # [m, gx, gy, Jx, Jy, Jz]
        ODE["p"] = np.array([1.113, 0.0, 0.0, 0.015, 0.015, 0.007])  # [m, gx, gy, Jx, Jy, Jz]
        ODE.param_alias.update({
            "m": ("p", 0),
            "gx": ("p", 1),
            "gy": ("p", 2),
            "Jx": ("p", 3),
            "Jy": ("p", 4),
            "Jz": ("p", 5),
            "m_c": ("p_c", 0),
            "gx_c": ("p_c", 1),
            "gy_c": ("p_c", 2),
            "Jx_c": ("p_c", 3),
            "Jy_c": ("p_c", 4),
            "Jz_c": ("p_c", 5),
        })
        ODE["p_aux"] = np.array([0.23, 9.81, 5.9e-04, 1e-5/5.9e-04, 0.0])  # [l, g, kf, ktau, gz]
        ODE.param_alias.update({
            "l": ("p_aux", 0),
            "g": ("p_aux", 1),
            "kf": ("p_aux", 2),
            "ktau": ("p_aux", 3),
            "gz": ("p_aux", 4),
        })

        # controller gains

        ODE["kx"] = np.array([20, 20, 25])
        ODE["kv"] = np.array([9, 9, 12])
        ODE["kR"] = np.array([4.6, 4.6, 0.8])
        ODE["kOmega"] = np.array([0.5, 0.5, 0.08])

        ODE["ti"] = 0
        ODE["tf"] = 5
        self.ODE = ODE
        return ODE
    
    def set_initial_state(self, req):
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
        # Initialize robot state
        self.set_initial_state(req)
            
        # Set the robot uncertain parameters
        self.ODE["m"] = req.model_params[0]
        self.ODE["gy"] = req.model_params[1]
        self.ODE["gx"] = req.model_params[2]
        self.ODE["Jx"] = req.model_params[3]
        self.ODE["Jy"] = req.model_params[4]
        self.ODE["Jz"] = req.model_params[5]
        
        # Initialize the time
        self.ODE["ti"] = req.t_init
        self.ODE["tf"] = req.t_final
        
        # Initialize gains
        self.ODE["kx"] = req.gains[0].values
        self.ODE["kv"] = req.gains[1].values
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
        # print("Dynamic time: ", time.time() - start_int)             
        
        #####################################################################################
        # RECOVER INTEGRATION RESULTS
        #####################################################################################
        q = self.ODE.last_result[:,self.ODE.states_indices["q"]]
        u_int = self.ODE.last_result[:, self.ODE.states_indices["u_int"]]  # last 2 states are the integral of u
        u_vec = np.diff(u_int, axis=0).T / np.diff(time_vector, axis=0).T
        
        response = DynamicSrvResponse()
        
        response.executionTime = time.time() - start_int
            
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

            pos = [[des_traj[i][0]],
                    [des_traj[i][1]],
                    [des_traj[i][2]],
                    [des_traj[i][3]]]
            
            vel = [[des_traj[i][4]],
                    [des_traj[i][5]],
                    [des_traj[i][6]],
                    [des_traj[i][7]]]
            
            acc = [[des_traj[i][8]],
                    [des_traj[i][9]],
                    [des_traj[i][10]]]
            
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
    
    def integrate_along_flatten_trajectory(self, flatten_des_traj, dt):
        pass

    def optimize_trajectory(self, req):
        pass
    
    def optimize_mpc(self, req):
        pass

if __name__ == '__main__':
    model = Jetson(6)
    model.generate(3, verbose=True, overwrite=True)
