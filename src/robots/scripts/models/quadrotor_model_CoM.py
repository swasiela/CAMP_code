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
import pybullet as p
import random
import nlopt
import copy
import rospy
import time
from sympy import symbols, Matrix, DiracDelta, diff, simplify
from sympy.physics.mechanics import *
from numpy.linalg import *

from services_msgs.srv import SensitivitySrv, SensitivitySrvResponse
from services_msgs.srv import LocaloptSrv, LocaloptSrvResponse
from services_msgs.msg import DesiredState

N_states = 13          # q = [x y z vx vy vz Q(4*1) Omega(3*1)]
N_inputs = 4           # u = [f(4-1)]
N_par = 6            # p = [m, gx, gy, Jx, Jy, Jz]
N_par_aux = 5          # p_aux = [l, g, kf, ktau, gz]

MIN_THRUST = 0.0
MAX_THRUST = 15.0
MIN_M = -10.0
MAX_M = -MIN_M

def pnorm(vec, p):
        pnorm = 0
        for elem in vec:
            pnorm += pow(elem,p)
        return pow(pnorm,(1/p))
    
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

        module_name = "".join([f"jitced_jetson_CoM_n{self.N_ctrl_points}_{mode.value}", f"_{token}" if token else "", ".so"])
        module_name_simu = "".join([f"jitced_jetson_CoM_n{self.N_ctrl_points}_{Mode.SIMU.value}", f"_{token}" if token else "", ".so"])

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
        # Saturate actuators, this saturation is check in robust collision checking, no need to saturate it here
        # F_thrust = min(max(F_thrust, MIN_THRUST), MAX_THRUST)
        # M_xb = min(max(M_xb, MIN_M), MAX_M)
        # M_yb = min(max(M_yb, MIN_M), MAX_M)
        # M_zb = min(max(M_zb, MIN_M), MAX_M)

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

        if mode != Mode.SIMU:

            PI = self._problem.add_states("PI", N_states, N_par)
            TH_val = self._problem.add_states("TH_int",  N_inputs, N_par)

            # helpers function (derivatives, TH, u evaluations)
            TH = self._problem.new_sym_matrix("TH", (N_inputs, N_par))

            df_dq = self._problem.new_sym_matrix("df_dq", (N_states, N_states))
            df_du = self._problem.new_sym_matrix("df_du", (N_states, N_inputs))
            df_dp = self._problem.new_sym_matrix("df_dp", (N_states, N_par))
            dh_dq = self._problem.new_sym_matrix("dh_dq", N_inputs, N_states)
            
            helpers.update({
                df_dq : f.jacobian(q),
                df_du : f.jacobian(u),
                df_dp : f.jacobian(p),
                dh_dq : h.jacobian(q) + h.jacobian(eR)*(eR_eval.jacobian(q) + eR_eval.jacobian(b3d)*b3d_eval.jacobian(q)),
                TH    : dh_dq*PI,
            })
            
            system.update({
                PI: df_dq*PI + df_du*TH + df_dp,
                TH_val: TH,
            })
        if mode == mode.GRAD:
            raise NotImplementedError("There is no plan to implement the gradient version for now")

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
            ODE.states["PI"]: req.initial_PI,
        })
    
    def compute_sensitivity(self, req):

        start_time = time.time()

        #####################################################################################
        # START INITIALISATION
        #####################################################################################
        
        # Initialize the delta p ranges for the tubes computation
        sensitivity_parameters_dict = { "m_c": 1.113,
                                        "gx_c": 0.0,
                                        "gy_c": 0.0,
                                        "Jx_c": 0.015,
                                        "Jy_c": 0.015,
                                        "Jz_c": 0.007}
        dev_m = 0.07  # 0.1 = 10% deviation on parameters with non-zero nominal value  0.09
        dev_J = 0.1  # 0.1 = 10% deviation on parameters with non-zero nominal value
        offset = 0.03  # 0.05 = 5 cm for parameters with nominal value at 0 (namely gx and gy and gz) 0.03
        par_range = []
        for key in sensitivity_parameters_dict:
            val = sensitivity_parameters_dict[key]
            if val == 0:  # use offset for this one
                par_range.append(offset**2)
            else:
                if key == "Jx_c" or key == "Jy_c" or key == "Jz_c": 
                    par_range.append((val*dev_J)**2)
                elif key == "m_c":
                    par_range.append((val*dev_m)**2)
                   
        W_range = np.diag(par_range)  # this matrix is used for sensitivity norm computation, see, 'PRG_sens_norm.pdf'
        
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
        # print("Sensitivity time: ", time.time() - start_int)             
        
        #####################################################################################
        # START TUBES COMPUTATIONS
        #####################################################################################
        PImatrices = self.ODE.last_result[:,self.ODE.states_indices["PI"]]
        THmatrices = self.ODE.last_result[:,self.ODE.states_indices["TH_int"]]
        q = self.ODE.last_result[:,self.ODE.states_indices["q"]]
        u_int = self.ODE.last_result[:, self.ODE.states_indices["u_int"]]  # last 2 states are the integral of u
        u_vec = np.diff(u_int, axis=0).T / np.diff(time_vector, axis=0).T
        
        PI_reshape = np.reshape(PImatrices, (-1, N_states, N_par))  # get integral of TH
        TH_reshape = np.reshape(THmatrices, (-1, N_inputs, N_par))  # get integral of TH
        N_time, N_dim, _ = np.shape(TH_reshape)  # get sizes of interest from integral of Theta (input sensitivity)
        
        response = SensitivitySrvResponse()

        # Compute the tube along the state space
        for matrix in PI_reshape:
            radii, lambda_max = self.compute_tubes(matrix, W_range)
            response.ellipsoid_alongX.append(radii[0])
            response.ellipsoid_alongY.append(radii[1])
            response.ellipsoid_alongZ.append(radii[2])
            response.radii_lambda.append(lambda_max)
        
        # Compute the tube along the input space
        for i in range(N_time - 1):
            TH = np.subtract(TH_reshape[i + 1, :, :], TH_reshape[i, :, :]) / (time_vector[i + 1] - time_vector[i])  # compute Theta for one step
            radii, _ = self.compute_tubes(TH, W_range)
            response.ellipsoid_u1.append(radii[0])
            response.ellipsoid_u2.append(radii[1])
            response.ellipsoid_u3.append(radii[2])
            response.ellipsoid_u4.append(radii[3])
        # Pad the missing state due to the integral with the first value
        response.ellipsoid_u1.insert(0,response.ellipsoid_u1[0])
        response.ellipsoid_u2.insert(0,response.ellipsoid_u2[0])
        response.ellipsoid_u3.insert(0,response.ellipsoid_u3[0])
        response.ellipsoid_u4.insert(0,response.ellipsoid_u4[0])
        
        response.executionTime = time.time() - start_int
        # print("All tubes computation: ", response.executionTime)
   
        # The PI matrix at the final state
        response.final_PI = PImatrices[-1]
            
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
        
        # Pad the missing state due to the integral with the first value
        u_vec_padd0 = np.insert(u_vec[0], 0, u_vec[0][0])
        u_vec_padd1 = np.insert(u_vec[1], 0, u_vec[1][0])
        u_vec_padd2 = np.insert(u_vec[2], 0, u_vec[2][0])
        u_vec_padd3 = np.insert(u_vec[3], 0, u_vec[3][0])
        
        response.u1 = abs(u_vec_padd0)
        response.u2 = abs(u_vec_padd1)
        response.u3 = abs(u_vec_padd2)
        response.u4 = abs(u_vec_padd3)

        return response
    
    def compute_tubes(self, matrix, W_range):
        N_dim, N_params = np.shape(matrix)  # get sizes of interest from integral of Theta (input sensitivity)
        ei = np.eye(N_dim)  # base vectors in data space

        mat = matrix @ W_range @ matrix.T  # kernel matrix of the input sensitivity
    
        geometric_lambda_max = 0
        
        # Calculate the worst-case deviations
        ri = [np.sqrt(ei[:, k].T @ mat @ ei[:, k]) for k in range(N_dim)]
        
        geometric_lambda_max = pnorm(np.concatenate((ri[0:3],ri[6:10]), axis=None),8) 
        
        return (ri, geometric_lambda_max)
    
    def ellipsoid_volume_and_orientation(self, matrix, W_range):
        """
        Compute the volume of the ellipsoid in the 3D projection and its orientation with respect to the canonical basis.

        Parameters:
        - matrix (ndarray): Transformation matrix for the ellipsoid (n x n).
        - W_range (ndarray): Diagonal matrix of scaling factors (n x n).

        Returns:
        - semi_axes_lengths (ndarray): Lengths of the semi-axes in the projected 3D space.
        - orientation_angles (ndarray): Orientation angles between each ellipsoid axis and each canonical basis axis.
        """
        # Compute the full kernel matrix
        K = matrix @ W_range @ matrix.T
        
        # Project to the first 3 dimensions
        K_3d = K[:3, :3]  # Taking the top-left 3x3 submatrix

        # Check for positive definiteness
        eigenvalues, _ = eigh(K_3d)
        
        print("Eigenvalues of K_3d:", eigenvalues)
        
        if np.any(eigenvalues <= 0):
            print("Kernel matrix is not positive definite. Regularization is applied.")
            
            # Regularization: Add a small value to the diagonal
            epsilon = 1e-5  # Regularization strength
            K_3d += np.eye(K_3d.shape[0]) * epsilon

        # Attempt to compute the inverse of the kernel matrix
        try:
            K_inv_3d = np.linalg.inv(K_3d)
        except np.linalg.LinAlgError:
            print("Kernel matrix is singular, using pseudo-inverse instead.")
            K_inv_3d = pinv(K_3d)

        # Perform eigenvalue decomposition on K_inv_3d
        eigenvalues_inv, eigenvectors = eigh(K_inv_3d)

        # Calculate the semi-axis lengths from the eigenvalues
        semi_axes_lengths = 1 / np.sqrt(eigenvalues_inv)

        # Calculate orientation angles between each eigenvector and the canonical axes
        orientation_angles = np.arccos(np.clip(eigenvectors, -1.0, 1.0))  # in radians

        return semi_axes_lengths, orientation_angles
    
    def computeEuclideanDist(self, des_traj):
        dist = 0
        for i in range(0,len(des_traj)-1):
            dist += sqrt((des_traj[i][0] - des_traj[i+1][0])**2 + (des_traj[i][1] - des_traj[i+1][1])**2 + (des_traj[i][2] - des_traj[i+1][2])**2)
        return dist

    def getSurfacePoint(self, angles, center, rad):
        # Ellipsoid parametric representation
        # x=xc​+rx ​sin(ϕ)cos(θ)
        # y=yc+ry sin⁡(ϕ)sin⁡(θ)
        # z=zc+rz cos⁡(ϕ)
        # θ : longitude
        # ϕ : latitude
        
        pt = []
        if len(rad) == 2: # 2D
            pt.append(center[0] + rad[0]*sin(angles[1])*cos(angles[0])); # x
            pt.append(center[1] + rad[1]*sin(angles[1])*sin(angles[0])); # y
        else: # 3D
            pt.append(center[0] + rad[0]*sin(angles[1])*cos(angles[0])); # x
            pt.append(center[1] + rad[1]*sin(angles[1])*sin(angles[0])); # y
            pt.append(center[2] + rad[2]*cos(angles[1])); # z
        return pt

    def approximateEllipsoid(self, state, tube):
        angles = [] # Latitude and longitude angle to project on the ellipsoid 

        # We first check for all the maximum values
        if len(tube) == 2: # In 2D 
            angles = [(0,pi/2),(pi/2,pi/2),(pi,pi/2),(-pi/2,pi/2)]
        if len(tube) == 3:  # In 3D 
            angles = [(0,pi/2),(pi/2,pi/2),(pi,pi/2),(-pi/2,pi/2),(0, 0),(0, pi)]
        for i in range(0, len(angles)):

            state_ellipsoid = self.getSurfacePoint(angles[i], [state[0], state[1], state[2]], tube)

            # The orientation is the same as the nominal state
            p.resetBasePositionAndOrientation(self.robot_id,[state_ellipsoid[0], state_ellipsoid[1], state_ellipsoid[2]], [state[4], state[5], state[6], state[3]])
            p.performCollisionDetection()

            cc_blt_ellipsoid = p.getContactPoints(self.robot_id, self.env_id)
            if cc_blt_ellipsoid:
                if cc_blt_ellipsoid[0][8] < 0: #PyBullet use a margin of 0.04m for collisions
                    return False

        # Then we fill some gaps on the hull of the ellipsoid to refine the approximation (here we consider each center of the eight of the ellipsoid)
        angles = []
        if len(tube) == 2: # In 2D 
            angles = [(pi/4,pi/2),(3*pi/4,pi/2),(5*pi/4,pi/2),(7*pi/4,pi/2)]
        if len(tube) == 3:  # In 3D 
            angles = [(pi/4,pi/4),(3*pi/4,pi/4),(5*pi/4,pi/4),(7*pi/4,pi/4),
                        (pi/4,3*pi/4),(3*pi/4,3*pi/4),(5*pi/4,3*pi/4),(7*pi/4,3*pi/4)]
        for i in range(0, len(angles)):
            
            state_ellipsoid = self.getSurfacePoint(angles[i], [state[0], state[1], state[2]], tube)

            # The orientation is the same as the nominal state
            p.resetBasePositionAndOrientation(self.robot_id,[state_ellipsoid[0], state_ellipsoid[1], state_ellipsoid[2]], [state[4], state[5], state[6], state[3]])
            p.performCollisionDetection()

            cc_blt_ellipsoid = p.getContactPoints(self.robot_id, self.env_id)
            if cc_blt_ellipsoid:
                if cc_blt_ellipsoid[0][8] < 0: #PyBullet use a margin of 0.04m for collisions
                    return False
                
        # No collisions are found with the tube
        return True  
    
    def isStateValid(self, state, tube):
        # Send a request to check collision with the nominal robot state and the envronment
        p.resetBasePositionAndOrientation(self.robot_id, [state[0], state[1], state[2]], [state[4], state[5], state[6], state[3]])
        p.performCollisionDetection()
        
        cc_blt =p.getContactPoints(self.robot_id, self.env_id)

        # If the nominal state is in collision we don't need to check the tube
        if cc_blt:
            dist = p.getClosestPoints(self.robot_id, self.env_id, distance=0.1) #PyBullet use a margin of 0.04m for collisions
            if dist[0][8] < 0:
                return False
        else:
            return self.approximateEllipsoid(state, tube)
                              
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
        if self.ODE is None:
            raise RuntimeError("model not generated")

        self.ODE.set_initial_value()
        time_vector_to_return = np.array([self.ODE["ti"]])
        
        # State counter
        st_counter = 0

        previousTime = self.ODE["ti"]
        actualTime = dt

        while st_counter*11 < len(flatten_des_traj):
            time_vector = np.linspace(previousTime, actualTime, 2)
            time_vector_to_return = np.append(time_vector_to_return,np.array([actualTime]))

            pos = [[flatten_des_traj[st_counter*11]],
                    [flatten_des_traj[st_counter*11 + 1]],
                    [flatten_des_traj[st_counter*11 + 2]],
                    [flatten_des_traj[st_counter*11 + 3]]]
            
            vel = [[flatten_des_traj[st_counter*11 + 4]],
                    [flatten_des_traj[st_counter*11 + 5]],
                    [flatten_des_traj[st_counter*11 + 6]],
                    [flatten_des_traj[st_counter*11 + 7]]]
            
            acc = [[flatten_des_traj[st_counter*11 + 8]],
                    [flatten_des_traj[st_counter*11 + 9]],
                    [flatten_des_traj[st_counter*11 + 10]]]
            
            self.ODE["pos"] = pos
            self.ODE["vel"] = vel
            self.ODE["acc"] = acc
            self.ODE.apply_parameters()
            previousTime = actualTime
            actualTime += dt
            st_counter += 1

            # Uncomment to perform dopri5 integration
            # self.ODE.integrate_on_time_vector(time_vector)

            # Euler integration
            states = np.zeros((len(time_vector), self.ODE.n))
            states[0, :] = self.ODE.last_result[-1]
            self.ODE.time_points = np.concatenate((self.ODE.time_points, time_vector[1:]))
            for i, t in enumerate(time_vector[1:], start=1):
                states[i, :] = states[0, :] + (time_vector[1]-time_vector[0])*self.ODE.f(t,states[0, :])
            self.ODE.last_result = np.concatenate((self.ODE.last_result, states[1:, :]), axis=0)
            
        return time_vector_to_return
    
    def optimize_trajectory(self, req):
        
        class CustomStopper:
            def __init__(self, tolerance=1e-2, window_size=5, max_iter = 50, max_time = 2.0, criterion = "Iter"):
                self.criterion = criterion # Stopping criterion type
                self.tolerance = tolerance  # Percentage tolerance for convergence (e.g., 1% -> 0.01)
                self.window_size = window_size  # The number of iterations to track
                self.costs_window = []  # List to store recent cost values
                self.nb_iter = 0 # Number of iteration
                self.max_iter = max_iter # Maximum number of iteration
                self.max_time = max_time # Maximum allowed planning time
                self.start_time = time.time() # Starting time

            def check_convergence(self):
                if len(self.costs_window) < self.window_size:
                    return False  # Not enough data to check for convergence yet
                
                # Compute the maximum percentage variation between consecutive costs
                max_percentage_variation = 0.0
                for i in range(1, len(self.costs_window)):
                    current_cost = self.costs_window[i]
                    previous_cost = self.costs_window[i - 1]
                    
                    if abs(previous_cost) > 1e-10:  # Avoid division by zero
                        percentage_variation = (abs(current_cost - previous_cost) / abs(previous_cost)) * 100.0
                        max_percentage_variation = max(max_percentage_variation, percentage_variation)

                # If the maximum percentage variation is less than the tolerance, consider it converged
                return max_percentage_variation < self.tolerance
            
            def stopping_condition(self):
                if self.criterion == "Iter":
                    return self.nb_iter > self.max_iter
                elif self.criterion == "Convergence":
                    return self.check_convergence()
                elif self.criterion == "Time":
                    current_time = time.time()  # Get the current time
                    elapsed_time = current_time - self.start_time  # Calculate elapsed time
                    return elapsed_time > self.max_time
                else:
                    logging.error("Stopping criterion not implemented for post processing!")
                    return False

            def update(self, cost_value):
                """Update the costs window with the latest cost value."""
                if len(self.costs_window) >= self.window_size:
                    self.costs_window.pop(0)  # Remove the oldest cost if the window is full
                self.costs_window.append(cost_value)
                self.nb_iter += 1 # Update number of iteration

        # Objective function
        def objective_cost_fun(traj, grad=None):
            def cost_func(traj, wpt_index, lambda_, cost_type):
                if cost_type == "Length":
                    # Length-based cost
                    return len(traj)
                
                elif cost_type == "Sensi":
                    return sum(lambda_)
                
                elif cost_type == "Accuracy":
                    # Uncertainty-based cost
                    w1 = 0.5
                    w2 = 0.5
                    
                    # The vector to store all the pnorm at the desired states in the trajectory
                    L = [lambda_[i] for i in wpt_index]
                    
                    # Mean calculation (linked to the eigenvalues of the projector)
                    mean = np.mean(L)
                    
                    # Variance calculation
                    var = np.var(L)
                    
                    # Combine the mean and variance with weights
                    return w1 * mean + w2 * var
                
                else:
                    print("Error: Cost function not implemented for post-processing! Cost is set to infinity.")
                    return float('inf')
                
            des_traj = []
            # Initial state already used so we skip it
            for st in traj:
                des_traj.append(st)
            # Last state
            des_traj.append(copy.copy(self.target_x))
            des_traj.append(copy.copy(self.target_y))
            des_traj.append(copy.copy(self.target_z))
            des_traj.append(copy.copy(self.target_yaw))
            des_traj.append(copy.copy(self.target_vx))
            des_traj.append(copy.copy(self.target_vy))
            des_traj.append(copy.copy(self.target_vz))
            des_traj.append(copy.copy(self.target_wyaw))
            des_traj.append(copy.copy(self.target_ax))
            des_traj.append(copy.copy(self.target_ay))
            des_traj.append(copy.copy(self.target_az))
                
            self.time_vector = self.integrate_along_flatten_trajectory(des_traj, req.dt)
            #______________________________________________________________________________
            
            #############################################################################################
            # HARD CONSTRAINTS ON COLLISIONS
            #############################################################################################
            # Get sensitivity results
            PImatrices = self.ODE.last_result[:,self.ODE.states_indices["PI"]]
            THmatrices = self.ODE.last_result[:,self.ODE.states_indices["TH_int"]]
            q = self.ODE.last_result[:,self.ODE.states_indices["q"]]
            u_int = self.ODE.last_result[:, self.ODE.states_indices["u_int"]]  # last 2 states are the integral of u
            u_vec = np.diff(u_int, axis=0).T / np.diff(self.time_vector, axis=0).T
            
            PI_reshape = np.reshape(PImatrices, (-1, N_states, N_par))  # get integral of TH
            TH_reshape = np.reshape(THmatrices, (-1, N_inputs, N_par))  # get integral of TH
            N_time, N_dim, _ = np.shape(TH_reshape)  # get sizes of interest from integral of Theta (input sensitivity)

            # First check saturation
            for i in range(N_time - 1):
                TH = np.subtract(TH_reshape[i + 1, :, :], TH_reshape[i, :, :]) / (self.time_vector[i + 1] - self.time_vector[i])  # compute Theta for one step
                radii, _ = self.compute_tubes(TH, W_range)
                ellipsoid_u1 = radii[0]
                ellipsoid_u2 = radii[1]
                ellipsoid_u3 = radii[2]
                ellipsoid_u4 = radii[3]
                if(u_vec[0][i] + ellipsoid_u1 > 25000):
                    print("Optimization nb ", self.count_opti)
                    self.count_opti += 1
                    return inf
                if(u_vec[1][i] + ellipsoid_u2 > 25000):
                    print("Optimization nb ", self.count_opti)
                    self.count_opti += 1
                    return inf
                if(u_vec[2][i] + ellipsoid_u3 > 25000):
                    print("Optimization nb ", self.count_opti)
                    self.count_opti += 1
                    return inf
                if(u_vec[3][i] + ellipsoid_u4 > 25000):
                    print("Optimization nb ", self.count_opti)
                    self.count_opti += 1
                    return inf
            
            # Then check robust collision
            cpt_state = 0
            lambda_vec = []
            for matrix in PI_reshape:
                radii, lambda_max = self.compute_tubes(matrix, W_range)
                lambda_vec.append(lambda_max)
                
                current_st = []
                
                current_st.append(q[cpt_state][0])
                current_st.append(q[cpt_state][1])
                current_st.append(q[cpt_state][2])
                current_st.append(q[cpt_state][6])
                current_st.append(q[cpt_state][7])
                current_st.append(q[cpt_state][8])
                current_st.append(q[cpt_state][9])
                
                if not self.isStateValid(current_st, radii[0:3]):
                    print("Optimization nb ", self.count_opti)
                    self.count_opti += 1
                    return inf
                
                cpt_state += 1
            #############################################################################################
            #############################################################################################
            
            cost = cost_func(des_traj, [-1], lambda_vec, req.cost_id)
            
            # Display
            print("Optimization nb ", self.count_opti)
            
            if(self.best_cost > cost):
                self.best_cost = cost
                # Update the custom stopper with the new cost value and iteration value
                stopper.update(cost)
                executionTime = time.time() - self.start_time 
                # Display
                print("New best cost: ", cost)
                with open(self.cost_file_path, 'a') as file:
                    file.write("Cost\n")
                    file.write(str(self.best_cost))
                    file.write("\n")
                    file.write("Time\n")
                    file.write(str(executionTime))
                    file.write("\n")
                
            self.count_opti += 1
            
            # Check if the custom stopping criterion is met
            if stopper.stopping_condition():
                print("Custom stopping criterion met. Forcing optimization to stop.")
                self.last_known_opt = np.copy(traj)  # Keep a copy of the current parameter values
                opt.force_stop()  # Trigger the force stop
                    
            return float(cost)
        
        self.start_time = time.time()
        
        # Init Pybullet
        if p.isConnected():
            p.disconnect()
        self.p_ = p.connect(p.DIRECT) 
        # Load the robot URDF
        # Get the current directory of this script file
        current_dir = os.path.dirname(__file__)
        camp_dir = os.path.abspath(os.path.join(current_dir, '..', '..', '..', '..'))
        
        relative_cost_file = os.path.join('src', 'results', 'cost_nlopt.txt')
        self.cost_file_path = os.path.join(camp_dir, relative_cost_file)
        
        relative_urdf_robot = os.path.join('src', 'robots', 'urdf', 'quadrotor.urdf')
        robot_urdf_path = os.path.join(camp_dir, relative_urdf_robot)
        self.robot_id = p.loadURDF(robot_urdf_path, [0,0,0], [0,0,0,1]) 

        # Load the OBJ environment
        relative_urdf_scene = os.path.join('src', 'robots', 'urdf', 'scene_quad.urdf')
        robot_urdf_env_path = os.path.join(camp_dir, relative_urdf_scene)
        self.env_id = p.loadURDF(robot_urdf_env_path, [0, 0, 0])
        
        # Set the init state
        p.resetBasePositionAndOrientation(self.robot_id,[req.init_robot_state.x, req.init_robot_state.y, req.init_robot_state.z], [req.init_robot_state.qx, req.init_robot_state.qy, req.init_robot_state.qz, req.init_robot_state.qw])
        
        #####################################################################################
        # START INITIALISATION
        #####################################################################################
        
        # Initialize the delta p ranges for the tubes computation
        sensitivity_parameters_dict = { "m_c": 1.113,
                                        "gx_c": 0.0,
                                        "gy_c": 0.0,
                                        "Jx_c": 0.015,
                                        "Jy_c": 0.015,
                                        "Jz_c": 0.007}
        dev_m = 0.07  # 0.1 = 10% deviation on parameters with non-zero nominal value  0.09
        dev_J = 0.1  # 0.1 = 10% deviation on parameters with non-zero nominal value
        offset = 0.03  # 0.05 = 5 cm for parameters with nominal value at 0 (namely gx and gy and gz) 0.03
        par_range = []
        for key in sensitivity_parameters_dict:
            val = sensitivity_parameters_dict[key]
            if val == 0:  # use offset for this one
                par_range.append(offset**2)
            else:
                if key == "Jx_c" or key == "Jy_c" or key == "Jz_c": 
                    par_range.append((val*dev_J)**2)
                elif key == "m_c":
                    par_range.append((val*dev_m)**2)
                   
        W_range = np.diag(par_range)  # this matrix is used for sensitivity norm computation, see, 'PRG_sens_norm.pdf'
        
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
        
        self.count_opti = 1

        #####################################################################################
        # INITIALIZE TRAJECTORY
        #####################################################################################
        # The trajectory must be flatten for the optimization
        # We optimized only the states between the first and the last one that we want to keep fixed
        flatten_des_traj = [] # contains all the waypoints
        for des_state in req.desired_states[1:-1]:
            flatten_des_traj.append(des_state.x)
            flatten_des_traj.append(des_state.y)
            flatten_des_traj.append(des_state.z)
            flatten_des_traj.append(des_state.yaw)
            flatten_des_traj.append(des_state.vx)
            flatten_des_traj.append(des_state.vy)
            flatten_des_traj.append(des_state.vz)
            flatten_des_traj.append(des_state.wyaw)
            flatten_des_traj.append(des_state.ax)
            flatten_des_traj.append(des_state.ay)
            flatten_des_traj.append(des_state.az)

        if req.opti_params != "Traj":
            print("Optimizing something else than the trajectory waypints not implmented yet !")
            return False
        
        # Setup the inital state (i.e. the first one)
        self.init_x = req.desired_states[0].x
        self.init_y = req.desired_states[0].y
        self.init_z = req.desired_states[0].z
        self.init_yaw = req.desired_states[0].yaw
        self.init_vx = req.desired_states[0].vx
        self.init_vy = req.desired_states[0].vy
        self.init_vz = req.desired_states[0].vz
        self.init_wyaw = req.desired_states[0].wyaw
        self.init_ax = req.desired_states[0].ax
        self.init_ay = req.desired_states[0].ay
        self.init_az = req.desired_states[0].az
        
        # Setup the target state (i.e. the final one)
        self.target_x = req.desired_states[-1].x
        self.target_y = req.desired_states[-1].y
        self.target_z = req.desired_states[-1].z
        self.target_yaw = req.desired_states[-1].yaw
        self.target_vx = req.desired_states[-1].vx
        self.target_vy = req.desired_states[-1].vy
        self.target_vz = req.desired_states[-1].vz
        self.target_wyaw = req.desired_states[-1].wyaw
        self.target_ax = req.desired_states[-1].ax
        self.target_ay = req.desired_states[-1].ay
        self.target_az = req.desired_states[-1].az
        
        # Create the optimization problem
        opt = nlopt.opt(nlopt.LN_COBYLA, len(flatten_des_traj))
        
        # Custom stopper instance
        stopper = CustomStopper(tolerance=req.tolerance, window_size=req.window_size, max_iter=req.nb_iter, max_time=req.max_time, criterion=req.stopping_condition) 
        opt.set_ftol_rel(1e-8)
        
        opt.set_min_objective(objective_cost_fun)
        
        # Get initial cost
        self.best_cost = inf
        init_cost = objective_cost_fun(flatten_des_traj, None)   
        
        with open(self.cost_file_path, 'w') as file:
            file.write("Cost\n")
            file.write(str(init_cost))
            file.write("\n")
            file.write("Time\n")
            file.write(str(0.0))
            file.write("\n")
        
        # Start optimization with an initial guess
        self.last_known_opt = None  # Initialize the variable to store optimization result
        try:
            state_opt = opt.optimize(flatten_des_traj)
        except nlopt.ForcedStop:
            state_opt = self.last_known_opt  # Get the last known solution
        
        response = LocaloptSrvResponse()
        
        # State counter
        st_counter = 0
        
        # Add the initial state
        init_des_st = DesiredState()
        init_des_st.x = copy.copy(self.init_x)
        init_des_st.y = copy.copy(self.init_y)
        init_des_st.z = copy.copy(self.init_z)
        init_des_st.yaw = copy.copy(self.init_yaw)
        init_des_st.vx = copy.copy(self.init_vx)
        init_des_st.vy = copy.copy(self.init_vy)
        init_des_st.vz = copy.copy(self.init_vz)
        init_des_st.wyaw = copy.copy(self.init_wyaw)
        init_des_st.ax = copy.copy(self.init_ax)
        init_des_st.ay = copy.copy(self.init_ay)
        init_des_st.az = copy.copy(self.init_az)
        response.desired_states_opt.append(init_des_st)
        
        while st_counter*11 < len(state_opt):
            des_state = DesiredState()
            des_state.x = copy.copy(state_opt[st_counter*11])
            des_state.y = copy.copy(state_opt[st_counter*11 + 1])
            des_state.z = copy.copy(state_opt[st_counter*11 + 2])
            des_state.yaw = copy.copy(state_opt[st_counter*11 + 3])
            des_state.vx = copy.copy(state_opt[st_counter*11 + 4])
            des_state.vy = copy.copy(state_opt[st_counter*11 + 5])
            des_state.vz = copy.copy(state_opt[st_counter*11 + 6])
            des_state.wyaw = copy.copy(state_opt[st_counter*11 + 7])
            des_state.ax = copy.copy(state_opt[st_counter*11 + 8])
            des_state.ay = copy.copy(state_opt[st_counter*11 + 9])
            des_state.az = copy.copy(state_opt[st_counter*11 + 10])
            
            response.desired_states_opt.append(des_state)
            st_counter += 1
        
        # Add the final state
        final_des_st = DesiredState()
        final_des_st.x = copy.copy(self.target_x)
        final_des_st.y = copy.copy(self.target_y)
        final_des_st.z = copy.copy(self.target_z)
        final_des_st.yaw = copy.copy(self.target_yaw)
        final_des_st.vx = copy.copy(self.target_vx)
        final_des_st.vy = copy.copy(self.target_vy)
        final_des_st.vz = copy.copy(self.target_vz)
        final_des_st.wyaw = copy.copy(self.target_wyaw)
        final_des_st.ax = copy.copy(self.target_ax)
        final_des_st.ay = copy.copy(self.target_ay)
        final_des_st.az = copy.copy(self.target_az)
        response.desired_states_opt.append(final_des_st)
        
        PImatrices = self.ODE.last_result[:,self.ODE.states_indices["PI"]]
        THmatrices = self.ODE.last_result[:,self.ODE.states_indices["TH_int"]]
        q = self.ODE.last_result[:,self.ODE.states_indices["q"]]
        
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
        
        # The PI matrix at the final state
        response.final_PI = PImatrices[-1]
        
        # Cost
        response.cost = opt.last_optimum_value()
        
        print("Init cost: ", init_cost)
        print("Final cost: ", response.cost)
            
        return response
     
    def optimize_mpc(self, req):
        pass
   
def test_integration_time(model, integrator):
    # Test integration time  
    ##################################################################################################### 
    
    # Initialize the delta p ranges for the tubes computation
    sensitivity_parameters_dict = { "m_c": 1.113,
                                    "gx_c": 0.0,
                                    "gy_c": 0.0,
                                    "Jx_c": 0.015,
                                    "Jy_c": 0.015,
                                    "Jz_c": 0.007}
    dev_m = 0.07  # 0.1 = 10% deviation on parameters with non-zero nominal value  0.09
    dev_J = 0.1  # 0.1 = 10% deviation on parameters with non-zero nominal value
    offset = 0.03  # 0.05 = 5 cm for parameters with nominal value at 0 (namely gx and gy and gz) 0.03
    par_range = []
    for key in sensitivity_parameters_dict:
        val = sensitivity_parameters_dict[key]
        if val == 0:  # use offset for this one
            par_range.append(offset**2)
        else:
            if key == "Jx_c" or key == "Jy_c" or key == "Jz_c": 
                par_range.append((val*dev_J)**2)
            elif key == "m_c":
                par_range.append((val*dev_m)**2)
                
    W_range = np.diag(par_range)  # this matrix is used for sensitivity norm computation, see, 'PRG_sens_norm.pdf'
        
    total_time = 0.0
    # Compute for 100 trajectories
    nb_traj = 100
    for i in range(0,nb_traj):
        # Initialize robot state
        ODE = model.ODE
        p_0 = np.zeros(3)
        v_0 = np.zeros(3)
        q_0 = np.zeros(4)
        w_0 = np.zeros(3)

        ODE.set_default_initial_state({
            ODE.states["q"]: np.concatenate((p_0, v_0, q_0 , w_0))
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
            st.append(j*0.01)
            st.append(0.0)
            st.append(0.01)
            st.append(0.01)
            st.append(0.01)
            st.append(0.0)
            st.append(0.0)
            st.append(0.0)
            st.append(0.0)
            des_traj.append(st)
            
        start_int = time.time()
        
        # Create the time vector
        dt = 0.05
        time_vector_int = np.arange(0, (len(des_traj)+1) * dt, dt)
    
        time_vector = model.integrate_along_trajectory(des_traj, time_vector_int, dt, integrator)
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
    model = Jetson(6)
    model.generate(3, verbose=True, overwrite=False)
    integrator = "RK2" # Euler RK2 RK4
    
    # Test integration time
    test_integration_time(model, integrator)