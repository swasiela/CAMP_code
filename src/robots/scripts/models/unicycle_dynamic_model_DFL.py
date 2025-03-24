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
from sympy import *
from sympy import symbols, Matrix
from sympy.physics.mechanics import *
from numpy.linalg import *

from scipy.spatial.transform import Rotation as R

from services_msgs.srv import SensitivitySrv, SensitivitySrvResponse
from services_msgs.srv import LocaloptSrv, LocaloptSrvResponse
from services_msgs.srv import KdtpSrv, KdtpSrvRequest, KdtpSrvResponse
from services_msgs.msg import DesiredState, KdtpState

from time import time

N_states = 5           # q = [x y v theta w]
N_inputs = 2           # u = [ul ur]
N_outputs = 2          # y = [x y]
N_ctrl_states = 3      # xi = [xi_v xi_x xi_y]
N_par = 3              # p = [r b m]
N_par_aux = 2          # p_aux = [a I]

def pnorm(vec, p):
    pnorm = 0
    for elem in vec:
        pnorm += pow(elem,p)
    return pow(pnorm,(1/p))

class Unicycle(Model):
    
    def equations(debug=False):
        print("Making symbolic state evolution computations ...")

        fx = Function('x')(t)
        fy = Function('y')(t)
        fv = Function('v')(t)
        ftheta = Function('theta')(t)
        fw = Function('w')(t)

        r = Matrix([[fx], [fy]])
        p = Matrix([[r_], [b], [a], [m], [I]])
        u = Matrix([[ul], [ur]])

        S = Matrix([[r_/2, r_/2], [r_/(2*b), -r_/(2*b)]])

        # Compute function f and jacobians
        f1 = Matrix([[fv*cos(ftheta) - a*fw*sin(ftheta)], [fv*sin(ftheta) + a*fw*cos(ftheta)], [-a*fw*fw], [fw], [0]])

        f2 = Matrix([[0, 0], [0, 0], [1/m, 0], [0, 0], [0, 1/I]])*S  # Get rid of S if inputs as accelerations in robot frame
        f = factor(f1 + f2*u)


        # Computing the inverse kinematics of the model (namely the A matrix and b vector)
        # Time derivatives
        drdt = diff(r, t)
        drdt = drdt.subs([(diff(fx,t), f[0,0]), (diff(fy,t), f[1,0]), (diff(fv,t), f[2,0]), (diff(ftheta,t), f[3,0]), (diff(fw,t), f[4,0])])

        ddrdt = diff(drdt, t)
        ddrdt = ddrdt.subs([(diff(fx,t), f[0,0]), (diff(fy,t), f[1,0]), (diff(fv,t), f[2,0]), (diff(ftheta,t), f[3,0]), (diff(fw,t), f[4,0])])

        # Remove time dependence
        x, y, v, theta, w = symbols('x y v theta w')
        q = Matrix([[x], [y], [v], [theta], [w]])
        r = Matrix([[x], [y]])
        f = f.subs([(fx, x), (fy, y), (fv, v), (ftheta, theta), (fw, w)])
        drdt = drdt.subs([(fx, x), (fy, y), (fv, v), (ftheta, theta), (fw, w)])
        ddrdt = ddrdt.subs([(fx, x), (fy, y), (fv, v), (ftheta, theta), (fw, w)])

        b = ddrdt.subs([(ul, 0), (ur,0)])

        A = Matrix(BlockMatrix([simplify(ddrdt - ddrdt.subs(ul, 0)).subs(ul, 1),
                                simplify(ddrdt - ddrdt.subs(ur, 0)).subs(ur, 1)]))

        assert simplify(A*u + b - ddrdt) == Matrix([[0], [0]])  # Make sure de decomposition is right	

        fq = f.jacobian(q)
        fp = f.jacobian(p)
        fu = f.jacobian(u)

        # Define controler variables
        xiv, xix, xiy = symbols('xiv xix xiy')
        ddrdx, ddrdy, drdx, drdy, rdx, rdy = symbols('ddrdx ddrdy drdx drdy rdx rdy')
        kv, kp, ki = symbols('kv kp ki')
        k = Matrix([[kv], [kp], [ki]])

        ddrd = Matrix([[ddrdx], [ddrdy]])
        drd = Matrix([[drdx], [drdy]])
        rd = Matrix([[rdx], [rdy]])

        xi = Matrix([[xix], [xiy]])
        xixy = Matrix([[xix], [xiy]])

        eta = ddrd + kv*(drd - drdt) + kp*(rd - r) + ki*xixy
        A_inv = A.inv()

        temp1 = simplify(A_inv*(eta - b))

        # Computing g, h, and their jacobians
        g = (rd - r)
        gq = g.jacobian(q)
        gxi = g.jacobian(xi)

        h = temp1
        hq = h.jacobian(q)
        hxi = h.jacobian(xi)

        # Lambdifying necessary functions
        f_np = lambdify((q,p,u), f, "numpy")
        fq_np = lambdify((q,p,u), fq, "numpy")
        fp_np = lambdify((q,p,u), fp, "numpy")
        fu_np = lambdify((q,p,u), fu, "numpy")

        g_np =lambdify((q, u, xi, ddrd, drd, rd, k), g, "numpy")
        gq_np =lambdify((q, u, xi, ddrd, drd, rd, k), gq, "numpy")
        gxi_np =lambdify((q, u, xi, ddrd, drd, rd, k), gxi, "numpy")

        h_np =lambdify((q, p, u, xi, ddrd, drd, rd, k), h, "numpy")
        hq_np =lambdify((q, p, u, xi, ddrd, drd, rd, k), hq, "numpy")
        hxi_np =lambdify((q, p, u, xi, ddrd, drd, rd, k), hxi, "numpy")

        # # Adapting formalism (i.e. flattening arrays)
        def f(q,p,u):
            return f_np(q.flatten(), p.flatten(), u.flatten())

        def fq(q,p,u):
            return fq_np(q.flatten(), p.flatten(), u.flatten())

        def fp(q,p,u):
            return fp_np(q.flatten(), p.flatten(), u.flatten())

        def fu(q,p,u):
            return fu_np(q.flatten(), p.flatten(), u.flatten())

        def g(q, u, xi, ddrd, drd, rd, k):
            return g_np(q.flatten(), u.flatten(), xi.flatten(), ddrd.flatten(), drd.flatten(),rd.flatten(), k.flatten())

        def gq(q, u, xi, ddrd, drd, rd, k):
            return gq_np(q.flatten(), u.flatten(), xi.flatten(), ddrd.flatten(), drd.flatten(),rd.flatten(), k.flatten())

        def gxi(q, u, xi, ddrd, drd, rd, k):
            return gxi_np(q.flatten(), u.flatten(), xi.flatten(), ddrd.flatten(), drd.flatten(),rd.flatten(), k.flatten())

        def h(q, p, u, xi, ddrd, drd, rd, k):
            # cmd = 5 # for input saturation
            u = h_np(q.flatten(), p.flatten(), u.flatten(), xi.flatten(), ddrd.flatten(), drd.flatten(),rd.flatten(), k.flatten())
            return u

        def hq(q, p, u, xi, ddrd, drd, rd, k):
            return hq_np(q.flatten(), p.flatten(), u.flatten(), xi.flatten(), ddrd.flatten(), drd.flatten(),rd.flatten(), k.flatten())

        def hxi(q, p, u, xi, ddrd, drd, rd, k):
            return hxi_np(q.flatten(), p.flatten(), u.flatten(), xi.flatten(), ddrd.flatten(), drd.flatten(),rd.flatten(), k.flatten())

        return f, fq, fp, fu, g, gq, gxi, h, hq, hxi

    def generate(self, N_lc, verbose=False, mode: Mode=Mode.NOGRAD, token="", overwrite=False):
        if not isinstance(mode, Mode):
            raise ValueError(f"mode must either be a member of the Mode enum")

        module_name = "".join([f"jitced_unicycle_dynamic_DFL_n{self.N_ctrl_points}_{mode.value}", f"_{token}" if token else "", ".so"])

        helpers = dict()
        system = dict()
        subs = dict()
        self._problem = pb = ODEproblem()
        
        ti = pb.new_parameter("ti", 1, real=True)
        tf = pb.new_parameter("tf", 1, real=True)
        
        # parameters (q, u, p, xi, kc)
        q = pb.add_states("q", N_states)
        u = pb.new_sym_matrix("u", N_inputs)
        p = pb.new_parameter("p", N_par)
        xi = pb.add_states("xi", N_ctrl_states)
        k_c = pb.new_parameter("k_c", 3, real=True)
        
        # Symbols
        x, y, v, theta, w, ul, ur, r_, b, a, m, I = dynamicsymbols('x y v theta w ul ur r_ b a m I')
        t = dynamicsymbols('t')
        
        subs["q"] = [x, y, v, theta, w], q
        subs["u"] = [ul, ur], u
        p = pb.new_parameter("p", N_par)
        subs["p"] = [r_, b, m], p
        p_c = pb.new_parameter("p_c", N_par)
        subs["p_c"] = [r_, b, m], p_c
        p_aux = pb.new_parameter("p_aux", N_par_aux)
        subs["p_aux"] = [a, I], p_aux
        
        self.u_min = -5
        self.u_max = 5
        
        ####################################################################################################################
        # EQUATIONS
        ####################################################################################################################

        # --------
        # Dynamic
        # --------

        fx = Function('x')(t)
        fy = Function('y')(t)
        fv = Function('v')(t)
        ftheta = Function('theta')(t)
        fw = Function('w')(t)

        r = Matrix([[fx], [fy]])
        p = Matrix([[r_], [b], [a], [m], [I]])
        u = Matrix([[ul], [ur]])

        S = Matrix([[r_/2, r_/2], [r_/(2*b), -r_/(2*b)]])

        # Compute function f and jacobians
        f1 = Matrix([[fv*cos(ftheta) - a*fw*sin(ftheta)], [fv*sin(ftheta) + a*fw*cos(ftheta)], [-a*fw*fw], [fw], [0]])

        f2 = Matrix([[0, 0], [0, 0], [1/m, 0], [0, 0], [0, 1/I]])*S  # Get rid of S if inputs as accelerations in robot frame
        f = factor(f1 + f2*u)

        # Computing the inverse kinematics of the model (namely the A matrix and b vector)
        # Time derivatives
        drdt = diff(r, t)
        drdt = drdt.subs([(diff(fx,t), f[0,0]), (diff(fy,t), f[1,0]), (diff(fv,t), f[2,0]), (diff(ftheta,t), f[3,0]), (diff(fw,t), f[4,0])])

        ddrdt = diff(drdt, t)
        ddrdt = ddrdt.subs([(diff(fx,t), f[0,0]), (diff(fy,t), f[1,0]), (diff(fv,t), f[2,0]), (diff(ftheta,t), f[3,0]), (diff(fw,t), f[4,0])])

        # Remove time dependence
        r = Matrix([[x], [y]])
        f = f.subs([(fx, x), (fy, y), (fv, v), (ftheta, theta), (fw, w)])
        drdt = drdt.subs([(fx, x), (fy, y), (fv, v), (ftheta, theta), (fw, w)])
        ddrdt = ddrdt.subs([(fx, x), (fy, y), (fv, v), (ftheta, theta), (fw, w)])

        b = ddrdt.subs([(ul, 0), (ur,0)])

        A = Matrix(BlockMatrix([simplify(ddrdt - ddrdt.subs(ul, 0)).subs(ul, 1),
                                simplify(ddrdt - ddrdt.subs(ur, 0)).subs(ur, 1)]))

        assert simplify(A*u + b - ddrdt) == Matrix([[0], [0]])  # Make sure de decomposition is right

        fq = f.jacobian(q)
        fp = f.jacobian(p)
        fu = f.jacobian(u)

        # Define controler variables
        # introducing helpers
        pos = pb.new_parameter("pos", 2, 1) #position of the waypoint (X, Y)
        vel = pb.new_parameter("vel", 2, 1)
        acc = pb.new_parameter("acc", 2, 1)
        rd = pb.new_sym_matrix("rd", 2)
        drd = pb.new_sym_matrix("drd", 2)
        ddrd = pb.new_sym_matrix("ddrd", 2)
        helpers.update({
            rd[0]: pos[0,0],
            rd[1]: pos[1,0],
            drd[0]: vel[0,0],
            drd[1]: vel[1,0],
            ddrd[0]: acc[0,0],
            ddrd[1]: acc[1,0],
        })
        
        xiv, xix, xiy = symbols('xiv xix xiy')
        kv, kp, ki = symbols('kv kp ki')
        
        subs["k_c"] = [kv, kp, ki], k_c
        subs["xi"] = [xiv, xix, xiy], xi

        ddrd = Matrix([[ddrd[0]], [ddrd[1]]])
        drd = Matrix([[drd[0]], [drd[1]]])
        rd = Matrix([[rd[0]], [rd[1]]])
        xixy = Matrix([[xix], [xiy]])

        eta = ddrd + kv*(drd - drdt) + kp*(rd - r) + ki*xixy
        A_inv = A.inv()

        # Computing g, h
        g = rd - r
        h = simplify(A_inv*(eta - b))
        
        # Transform to symengine
        f = se.sympify(f)
        h = se.sympify(h)
        g = se.sympify(g)

        # Compute jacobians
        
        gq = g.jacobian(q)
        gxi = g.jacobian(xi)

        hq = h.jacobian(q)
        hxi = h.jacobian(xi) 

        # Helpers and system update
        helpers[u] = h
        u_int = pb.add_states("u_int", N_inputs)
            
        system.update({
            q: f,
            xi: g,
            u_int: u,
        })

        if mode != Mode.SIMU:

            PI = pb.add_states("PI", N_states, N_par)
            PI_xi = pb.add_states("PI_xi", N_ctrl_states, N_par)
            TH_val = pb.add_states("TH_int",  N_inputs, N_par) # integral of Th

            # helpers function (derivatives, TH, u evaluations)
            TH = pb.new_sym_matrix("TH", (N_inputs, N_par))

            df_dq = pb.new_sym_matrix("df_dq", (N_states, N_states))
            df_du = pb.new_sym_matrix("df_du", (N_states, N_inputs))
            df_dp = pb.new_sym_matrix("df_dp", (N_states, N_par))
            dg_dq = pb.new_sym_matrix("dg_dq", (N_ctrl_states, N_states))
            dg_dxi = pb.new_sym_matrix("dg_dxi", (N_ctrl_states, N_ctrl_states))
            dh_dq = pb.new_sym_matrix("dh_dq", N_inputs, N_states)
            dh_dxi = pb.new_sym_matrix("dh_dxi", N_inputs, N_ctrl_states)

            helpers.update({
                df_dq : fq,
                df_du : fu,
                df_dp : fp,
                dg_dq : gq,
                dg_dxi: gxi,
                dh_dq : hq,
                dh_dxi: hxi,
                TH    : dh_dq*PI + dh_dxi*PI_xi,
            })

            system.update({
                PI: df_dq*PI + df_du*TH + df_dp,
                PI_xi: dg_dq*PI + dg_dxi*PI_xi,
                TH_val: TH,
            })
        if mode == mode.GRAD:
            raise NotImplementedError("There is no plan to implement the gradient version for now")

        pb.register_helpers(helpers)
        pb.register_system(system)

        ODE = pb.init_ODE(verbose=verbose, module_location=module_name, overwrite=overwrite)

        # nominal parameters
        r_nom = 0.1
        b_nom = 0.15
        a_nom = 0.1
        m_nom = 0.5
        I_nom = 0.3
        
        ODE["p"] = [r_nom, b_nom, m_nom]
        ODE["p_c"] = [r_nom, b_nom, m_nom]
        ODE["p_aux"] = [a_nom, I_nom]
        ODE["k_c"] = [5., 19.,  4.]
        
        ODE.param_alias.update({
            "r": ("p", 0),
            "b": ("p", 1),
            "m": ("p", 2),
            "r_c": ("p_c", 0),
            "b_c": ("p_c", 1),
            "m_c": ("p_c", 2),
            "a": ("p_aux", 0),
            "I": ("p_aux", 1),
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
        # The order 'zyx' means we want yaw (z), pitch (y), and roll (x)
        euler_angles = r.as_euler('zyx', degrees=False)

        q_0 = [x0, y0, sqrt(vx0**2+vy0**2), euler_angles[0], 0.0]
        xi_0 = [0.1, 0.0, 0.0]
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

        deviation = 0.05  # 0.1 = 10% deviation on parameters with non-zero nominal value
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
        time_vector = self.integrate_along_trajectory(des_traj, req.time_vec, req.dt)
        # print("Sensitivity time: ", time.time() - start_int)   

        #####################################################################################
        # START TUBES COMPUTATIONS
        #####################################################################################
        PImatrices = self.ODE.last_result[:,self.ODE.states_indices["PI"]]
        PI_xi_matrices = self.ODE.last_result[:,self.ODE.states_indices["PI_xi"]]
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
            response.radii_lambda.append(lambda_max)
        
        # Compute the tube along the input space
        for i in range(N_time - 1):
            TH = np.subtract(TH_reshape[i + 1, :, :], TH_reshape[i, :, :]) / (time_vector[i + 1] - time_vector[i])  # compute Theta for one step
            radii, _ = self.compute_tubes(TH, W_range)
            response.ellipsoid_u1.append(radii[0])
            response.ellipsoid_u2.append(radii[1])
        
        response.executionTime = time.time() - start_int
        # print("All tubes computation: ", response.executionTime)
   
        # The PI matrix at the final state
        response.final_PI = PImatrices[-1]
        response.final_PI_xi = PI_xi_matrices[-1]
        
        for state in q:
            response.trajX.append(state[0])
            response.trajY.append(state[1])
            response.trajVX.append(0.0)
            response.trajVY.append(0.0)
            
            # Create a Rotation object from Euler angles
            r = R.from_euler('zyx', [state[2], 0.0, 0.0])
            # Convert to quaternion
            quaternion = r.as_quat()
            response.trajQx.append(quaternion[0])
            response.trajQy.append(quaternion[1])
            response.trajQz.append(quaternion[2])
            response.trajQw.append(quaternion[3]) # Theta angle !! Not a quaternion
        
        response.u1 = u_vec[0]
        response.u2 = u_vec[1]
        
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

    def integrate_along_trajectory(self, des_traj, time_vec, dt):
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
            
            vel = [[(state[0] - q_current[-1][0]) / dt],
                    [(state[1] - q_current[-1][1]) / dt]]
            
            acc = [[(vel[0][0] - prev_vel_x) / dt],
                    [(vel[1][0] - prev_vel_y) / dt]]
            
            prev_vel_x = vel[0][0]
            prev_vel_y = vel[1][0]
            
            # vel = [[state[2]],
            #         [state[3]]]
            
            # acc = [[state[4]],
            #         [state[5]]]
            
            self.ODE["pos"] = pos
            self.ODE["vel"] = vel
            self.ODE["acc"] = acc
            self.ODE.apply_parameters()
            previousTime = actualTime
            actualTime += dt

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
    
    def integrate_along_flatten_trajectory(self, flatten_des_traj, dt):
        pass

    def optimize_trajectory(self, req):
        response = LocaloptSrvResponse()
        return response
    
    def optimize_mpc(self, req):
        response = LocaloptSrvResponse()
        return response
    
def _test():
    ODE.set_initial_value({
        ODE.states["q"]: q_0
    })
    ODE.apply_parameters()
    return [ODE.integrate(T) for T in np.linspace(0.0001, 5, 100)]


if __name__ == '__main__':
    model = Unicycle(10)
    ODE = model.generate(3, verbose=False, overwrite=True)
