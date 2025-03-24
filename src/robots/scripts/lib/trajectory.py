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

import math
import time

import symengine as se

from lib.base_model import Model
from lib.sym_gen import JitParam

import collections
from typing import Union, Sequence
import random
from functools import lru_cache
from math import atan2, comb
import numpy as np
import scipy.linalg as spl
from jitcode import UnsuccessfulIntegration
from numpy.linalg import pinv, norm
from scipy.linalg import block_diag
import nlopt

import matplotlib.pyplot as plt


def seed(a=None):
    random.seed(a)


@lru_cache(maxsize=None)
def discrete_set(N_coeff, N=500, ti=0, tf=1):
    assert ti < tf
    t = np.linspace(ti, tf, N)
    out = np.zeros((N_coeff, N))
    for i in range(N_coeff):
        out[i, :] = comb(N_coeff-1, i) * (t-ti)**i * (tf-t)**(N_coeff-1-i) / (tf-ti)**(N_coeff-1)
    return out


def discrete_traj(a, N_coeff, N=100, ti=0, tf=1):
    assert ti < tf
    if isinstance(a, np.ndarray):
        numel = a.size
    else:
        numel = len(a)
    return np.reshape(a, (numel//N_coeff, N_coeff)) @ discrete_set(N_coeff, N=N, ti=ti, tf=tf)


def bspline_deriv(ctrl_points, t, ti, tf, deriv, degree, N_dim):
    assert tf >= t >= ti
    assert tf > ti
    dt = tf - ti
    a = ctrl_points.reshape((N_dim, -1))
    point = np.zeros(N_dim)
    for k in range(deriv+1):
        for j in range(deriv - k , degree - k + 1):
            point[:] += math.factorial(degree) * comb(deriv, k) * (-1)**k * (t-ti)**(j-deriv+k) * (tf-t)**(degree-j-k) / \
                        (dt**degree * math.factorial(j-deriv+k) * math.factorial(degree-j-k)) * a[:, j]
    return point



def _spline_deriv_start(degree, N_deriv_max, dt):
    mat = np.zeros((N_deriv_max+1, N_deriv_max+1), dtype=np.object_)
    for N_deriv in range(N_deriv_max+1):
        for k in range(N_deriv+1):
            mat[N_deriv, N_deriv-k] = dt**(-N_deriv) * np.prod(np.arange(degree-N_deriv, degree)+1) * comb(N_deriv, k) * (-1)**k
    return mat


def _spline_deriv_end(degree, N_deriv_max, dt):
    mat = np.zeros((N_deriv_max+1, N_deriv_max+1), dtype=np.object_)
    for N_deriv in range(N_deriv_max+1):
        for k in range(N_deriv+1):
            mat[N_deriv, N_deriv_max-k] = dt**(-N_deriv) * np.prod(np.arange(degree-N_deriv, degree)+1) * comb(N_deriv, k) * (-1)**k
    return mat


@lru_cache()
def get_constraint_matrix_function(degree, N_deriv_max, N_dim):
    dt = se.Symbol("dt", real=True)
    beginning = _spline_deriv_start(degree, N_deriv_max, dt)
    end = _spline_deriv_end(degree, N_deriv_max, dt)
    M_part = spl.block_diag(beginning, end)
    M = spl.block_diag(*[M_part] * N_dim)
    M_func = se.lambdify(dt, M)
    return lambda dt: np.array(M_func(dt))


class PiecewiseSplineTrajectory:

    def __init__(self, time_vec, waypoints: Union[list, np.ndarray]):
        self._waypoints = np.array(waypoints)
        self._waypoints_t = np.array(time_vec)
        assert np.all(time_vec[1:] > time_vec[:-1])  # check if sorted
        self._N_waypoints = len(time_vec)
        tmp, self._N_jc, self._N_dim = self._waypoints.shape
        assert self._N_waypoints == tmp
        self._waypoints_mask = np.full(self.waypoints.shape, False, dtype=np.bool_)
        self._waypoints_t_mask = np.full(self._waypoints_t.shape, False)

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            raise TypeError(f"Expected CompositeSplineTrajectory not {type(other)}")
        return (
            np.all(self._waypoints == other._waypoints)
            and np.all(self._waypoints_t == other._waypoints_t)
            and np.all(self._waypoints_mask == other._waypoints_mask)
            and self._N_waypoints == other._N_waypoints
            and np.all(self._waypoints_t_mask == other._waypoints_t_mask)
        )

    def __deepcopy__(self, memodict={}):
        new_obj = PiecewiseSplineTrajectory(self._waypoints_t.copy(), self._waypoints.copy())
        new_obj._waypoints_mask[:] = self._waypoints_mask[:]
        new_obj._waypoints_t_mask[:] = self._waypoints_t_mask[:]
        return new_obj

    def deepcopy(self):
        return self.__deepcopy__()

    @staticmethod
    def gen_random_traj(init_waypoint: np.ndarray, N_waypoints: int, t: Union[float, Sequence[float]],
                        model: Model, lower_bounds=None, upper_bounds=None, nlc=None):
        N_jc, N_dim = init_waypoint.shape
        if lower_bounds is None:
            lower_bounds = np.full(N_jc*N_dim, -1e3)

        if upper_bounds is None:
            upper_bounds = np.full(N_jc*N_dim, 1e3)

        if isinstance(t, float) and t > 0:
            ti, tf = 0.0, t
        elif isinstance(t, collections.Sequence) and len(t) == 2:
            ti, tf = map(float, t)
        else:
            raise ValueError("incorrect value for t, should be positive final value or the interval [t_init, t_final]")

        # final waypoint
        while True:
            f_waypoint = np.reshape(
                np.random.uniform(lower_bounds, upper_bounds),
                init_waypoint.shape
            )
            r = norm(f_waypoint[0,:])
            if 0.1 <= r:
                break

        target_point = f_waypoint
        time_points = np.linspace(ti, tf, N_waypoints)
        traj_obj = PiecewiseSplineTrajectory(np.array([ti, tf]), np.array([init_waypoint, f_waypoint]))
        for tp in time_points[1:-1]:
            traj_obj.interpolate_waypoint_at(tp)
        traj_obj.optimize(model, nlc, target_point, lower_bounds, upper_bounds)
        return traj_obj, target_point

    def optimize(self, model, nlc, target_point, use_target=False, lower_bounds=None, upper_bounds=None, optim_time=10.0):
        model.set_default_state(self._waypoints[0])
        model.ODE.set_initial_value()
        # debugging
        # fig = plt.figure(900)
        # ax = fig.add_subplot(1, 1, 1, projection="3d")
        # traj_obj.plot(plot_control_points=True, axis=ax)
        if nlc is None:
            return self, target_point
        N_eval = [0]

        def objective(x, grad):
            N_eval[0] += 1
            self.update_from_flat_params(x)
            model.ODE.set_initial_value(time=self.waypoints_t[0])
            ti, tf = self.waypoints_t[0], self.waypoints_t[-1]
            for ctrl_points, ti_part, tf_part in self.traj_iter():
                time_vector = np.linspace(ti_part, tf_part, max(2, round(200 * (tf_part - ti_part) / (tf - ti))))
                model.ODE["a"] = ctrl_points
                model.ODE.integrate_on_time_vector(time_vector)
            states = model.ODE.last_result
            return _pre_opti_cost_from_states(model, states, model.ODE.time_points)

        def soft_bounds(x, time_vector, states, lb, ub):
            pos = states[:, model.output_indices()]
            lower = np.max(lb - pos, axis=0)
            upper = np.min(pos - ub, axis=0)
            return np.concatenate([lower, upper])

        def nlc_wrap(result, x, grad):
            states = model.ODE.last_result
            time_vector = model.ODE.time_points
            cond1 = nlc(grad, x, time_vector, states)
            result[:len(cond1)] = cond1
            result[len(cond1):] = soft_bounds(x, time_vector, states, lb2, ub2)
            # print(result)
            return result[:]

        def eq_constraints(result, x, grad):
            states = model.ODE.last_result
            times_vec = model.ODE.time_points
            result[:] = model.eq_constraints(grad, states, times_vec, target_point)
            return result

        # OPTIMIZE

        tic = time.time()
        x_ini = self.get_flat_free_params()
        opt = nlopt.opt(nlopt.LN_COBYLA, len(x_ini))  # choosing optimizer for pre-conditionning

        # input saturation and minimum curvature radius contraints, first pre-conditionning of the trajectory,
        # to make the trajectory dynamically feasible by the system
        opt.add_inequality_mconstraint(nlc_wrap, np.concatenate([model.nonlcon_tol(), 1e-6*np.ones(2*len(model.output_indices()))]))

        if model.eq_constraints_tol(target_point):
            self._waypoints_mask[-1, :, :] = True
            if use_target == True:  # for second pre-opt, with imperfect controller, target reach constraint
                opt.add_equality_mconstraint(eq_constraints, model.eq_constraints_tol(target_point))

        if lower_bounds is None:
            lower_bounds = np.full(np.size(x_ini), -np.inf)
        if upper_bounds is None:
            upper_bounds = np.full(np.size(x_ini), np.inf)

        ub2 = upper_bounds[0, :len(model.output_indices())]
        lb2 = lower_bounds[0, :len(model.output_indices())]
        lb, ub = self.generate_bounds()
        if lower_bounds.shape == self.waypoints[0].shape:
            for i in range(lb.shape[0]):
                lb[i, 1:] = np.maximum(lower_bounds.flatten(), lb[i, 1:])
        elif lower_bounds.size == lb.size:
            lb = np.maximum(lower_bounds.flatten(), lb.flatten())
        if upper_bounds.shape == self.waypoints[0].shape:
            for i in range(ub.shape[0]):
                ub[i, 1:] = np.minimum(upper_bounds.flatten(), ub[i, 1:])
        elif upper_bounds.size == ub.size:
            ub = np.minimum(upper_bounds.flatten(), ub.flatten())
        opt.set_lower_bounds(lb.flatten())
        opt.set_upper_bounds(ub.flatten())
        x_ini = np.minimum(ub.flatten(), np.maximum(lb.flatten(), x_ini))

        opt.set_min_objective(objective)
        opt.set_maxtime(optim_time)
        init_cost = objective(x_ini, None)
        x_final = opt.optimize(x_ini)
        self.update_from_flat_params(x_final)
        print(
            f"Obj improvement: {init_cost:.2e} --> {opt.last_optimum_value():.2e} "
            f"in {time.time() - tic} with exit code {opt.last_optimize_result()} in {opt.get_numevals()} {N_eval} evals"
        )

    @property
    def waypoints(self):
        return self._waypoints

    @property
    def waypoints_t(self):
        return self._waypoints_t

    @property
    def waypoints_mask(self):
        return self._waypoints_mask

    @property
    def N_waypoints(self):
        return self._N_waypoints

    @property
    def N_jc(self):
        return self._N_jc

    @property
    def N_dim(self):
        return self._N_dim

    @property
    def N_deriv(self):
        return self._N_jc -1

    @property
    def degree(self):
        return 2*self._N_jc - 1

    def free_waypoint(self, index):
        if index == 0:
            raise RuntimeError("cannot free 1st waypoint")
        self._waypoints_mask[index] = True

    def lock_waypoint(self, index):
        self._waypoints_mask[index] = False

    def generate_bounds(self):  # to generate the bounds for the optimization
        relative_margin = 0.01
        l_stack = np.full((self.N_waypoints, self.N_jc*self.N_dim + 1), -np.inf)
        u_stack = np.full((self.N_waypoints, self.N_jc*self.N_dim + 1), np.inf)
        # preventing points in the past
        ti = self._waypoints_t[0]
        tf = self.waypoints_t[-1]
        dt = (tf - ti) * relative_margin
        l_stack[1:, 0] = ti + dt
        u_stack[1:, 0] = tf
        indices = np.flatnonzero(np.diff(self._waypoints_t_mask))
        for start, end in zip(indices[::2], indices[1::2] + 1):
            ti = self._waypoints_t[start]
            tf = self.waypoints_t[end]
            dt = (tf - ti)*relative_margin
            l_stack[start+1:end, 0] = ti + dt
            u_stack[start+1:end, 0] = tf - dt
        ctrl_mask = np.reshape(self._waypoints_mask, newshape=(self.N_waypoints, self.N_jc*self.N_dim))
        waypoints_tmp = np.reshape(self.waypoints, newshape=(self.N_waypoints, self.N_jc*self.N_dim))
        l_stack[:, 1:][ctrl_mask == False] = u_stack[:, 1:][ctrl_mask == False] = waypoints_tmp[ctrl_mask == False]
        l_stack[:, 0][self._waypoints_t_mask == False] = u_stack[:, 0][self._waypoints_t_mask == False] = self.waypoints_t[self._waypoints_t_mask == False]
        return l_stack, u_stack

    def nlc(self, flat_params, grad):
        stack = np.reshape(flat_params, (self.N_waypoints, -1))
        if grad is not None and grad.size > 0:
            raise NotImplementedError("")
        dt = stack[:, 0]
        indices = np.flatnonzero(np.diff(self._waypoints_t_mask))
        condition = np.zeros(len(indices)//2)
        if self._waypoints_t_mask[indices[0]]:
            raise RuntimeError()
        i = 0
        for start, end in zip(indices[::2], indices[1::2] + 1):
            ti = self._waypoints_t[start]
            tf = self.waypoints_t[end]
            plage_dt = np.sum(dt[start+1:end])
            condition[i] = plage_dt - (tf-ti)
            i += 1
        return condition

    def get_flat_free_params(self):
        stack = np.zeros((self.N_waypoints, self.N_jc*self.N_dim + 1))
        stack[:, 1:] = np.reshape(self._waypoints, (self.N_waypoints, -1))
        stack[:, 0] = self._waypoints_t
        return stack.flatten()

    def update_from_flat_params(self, params):
        stack = np.reshape(params, (self.N_waypoints, -1))
        waypoints = stack[:, 1:]
        waypoints_t = stack[:, 0]
        indices = np.flatnonzero(np.diff(self._waypoints_t_mask))
        order = np.arange(self.N_waypoints, dtype=int)
        corrected_times = [np.zeros(0)]
        for start, end in zip(indices[::2], indices[1::2]+1):
            ti = self._waypoints_t[start]
            tf = self._waypoints_t[end]
            dt = (tf - ti)*1e-2
            ids = start + 1 + np.argsort(waypoints_t[start+1:end])
            order[start+1:end] = ids
            t_vec = waypoints_t[ids]
            t_vec[1:] = np.maximum(
                t_vec[:-1]+dt,
                t_vec[1:]
            )
            if t_vec[0] < ti + dt:
                t_vec += (ti + dt) - t_vec[0]
            if t_vec[-1] > tf - dt:
                t_vec = (t_vec - t_vec[0])*(tf - dt - t_vec[0])/(t_vec[-1] - t_vec[0]) + t_vec[0]
            corrected_times.append(t_vec)
        if len(indices) % 2 == 1:
            start = indices[-1]
            ti = self._waypoints_t[start]
            tf = self._waypoints_t[-1]
            dt = (tf - ti) * 1e-3
            ids = start + 1 + np.argsort(waypoints_t[start+1:])
            order[start+1:] = ids
            t_vec = waypoints_t[ids]
            t_vec[1:] = np.maximum(
                t_vec[:-1]+dt,
                t_vec[1:]
            )
            if t_vec[0] < ti + dt:
                t_vec += (ti + dt) - t_vec[0]
            corrected_times.append(t_vec)
        self._waypoints[self._waypoints_mask] = waypoints[order].reshape((-1, self._N_jc, self._N_dim))[self._waypoints_mask]
        self._waypoints_t[self._waypoints_t_mask] = np.concatenate(corrected_times)

    def part_traj(self, i):
        if not (0<=i<=self._N_waypoints-1):
            raise RuntimeError(
                f"i is not a valid index, must be between 0 and N_waypoints-1 ({i} not in [0; {self._N_waypoints-1}])"
            )
        start_conditions = self._waypoints[i]
        end_conditions = self._waypoints[i+1]
        dt = self._waypoints_t[i+1] - self._waypoints_t[i]
        M = get_constraint_matrix_function(self.degree, self.N_deriv, self.N_dim)
        b = []
        for dim in range(self.N_dim):
            b.append(start_conditions[:, dim])
            b.append(end_conditions[:, dim])
        b = np.hstack(b)
        #debug (also you can display dt, which sometimes goes to 0..)
        #print(M(dt))
        #print(b)
        return spl.solve(M(dt), b), self._waypoints_t[i], self._waypoints_t[i+1]

    def traj_iter(self):
        for i in range(self.N_waypoints-1):
            yield self.part_traj(i)

    def __iter__(self):
        for i in range(self.N_waypoints):
            yield self.waypoints_t[i], self.waypoints[i]

    def insert_waypoint(self, t, waypoint: np.ndarray, free_mask=True, t_free=True):
        if isinstance(free_mask, bool):
            free_mask = np.full((self.N_jc, self.N_dim), free_mask)
        if not(isinstance(free_mask, np.ndarray)) or np.shape(free_mask) != (self.N_jc, self.N_dim) or free_mask.dtype != np.bool_:
            raise ValueError("free_mask must either a boolean or a (N_jc, N_dim) numpy array of dtype bool")
        index = np.searchsorted(self._waypoints_t, t)
        if self._waypoints_t[index] == t:
            raise RuntimeError(f"waypoint already defined at t={t}")
        self._waypoints_t = np.insert(self._waypoints_t, index, t)
        self._waypoints = np.insert(self._waypoints, index, waypoint, axis=0)
        self._waypoints_mask = np.insert(self._waypoints_mask, index, free_mask, axis=0)
        self._waypoints_t_mask = np.insert(self._waypoints_t_mask, index, t_free, axis=0)
        self._N_waypoints += 1

    def interpolate_waypoint_at(self, t, free=True):
        if t < self._waypoints_t[0] or t > self._waypoints_t[-1]:
            raise RuntimeError(f"Cannot interpolate waypoint outside of already defined trajectory")
        if t in self._waypoints_t:
            raise RuntimeError(f"waypoint already defined at t={t}")
        index = np.searchsorted(self._waypoints_t, t) - 1
        ctrl_points, ti, tf = self.part_traj(index)
        waypoint = np.zeros((self.N_jc, self.N_dim))
        for deriv in range(self.N_jc):
            waypoint[deriv, :] = bspline_deriv(ctrl_points, t, ti, tf, deriv, self.degree, self.N_dim)
        self.insert_waypoint(t, waypoint, free_mask=free)

    def plot_simu(self, model: Model, plot_control_points=False, plot_expected=False, axis=None, target_point=None, **kwargs):
        if axis is None:
            axis = plt.gca()
        if not ("color" in kwargs):
            kwargs["color"] = axis._get_lines.get_next_color()

        model.integrate_along_trajectory(self, 500)
        if plot_control_points:
            if self.N_dim == 2:
                axis.plot(self.waypoints[:, 0, 0], self.waypoints[:, 0, 1], 'o', color=kwargs["color"])
            else:
                axis.plot(self.waypoints[:, 0, 0], self.waypoints[:, 0, 1], self.waypoints[:, 0, 2],
                          'o', color=kwargs["color"])
        axis.plot(*model.ODE.last_result[:, model.output_indices()].T, **kwargs)
        if target_point is not None:
            axis.plot(*target_point[0, :3], "*", color=kwargs["color"])
        if plot_expected:
            self.plot(plot_control_points=False, axis=axis, color=kwargs["color"], linestyle="dashed")

    def plot(self, plot_control_points=False, axis=None, **kwargs):
        if axis is None:
            axis = plt.gca()
        if not ("color" in kwargs):
            kwargs["color"] = axis._get_lines.get_next_color()
        for a, ti, tf in self.traj_iter():
            a = a.reshape((self.N_dim, -1))
            plot_bspline(a[:min(3, self.N_dim)], self.degree + 1, plot_control_points, axis, **kwargs)
            if self.N_dim == 2:
                axis.plot(self.waypoints[:, 0, 0], self.waypoints[:, 0, 1], 'o', color=kwargs["color"])
            else:
                axis.plot(self.waypoints[:, 0, 0], self.waypoints[:, 0, 1], self.waypoints[:, 0, 2],
                          'o', color=kwargs["color"])


def plot_bspline(a, N_coeff:int, plot_control_points=False, axis=None, **kwargs):
    if axis is None:
        axis = plt.gca()
    if not isinstance(a, np.ndarray):
        a = np.array(a)
    a = np.reshape(a, (a.size//N_coeff, N_coeff))
    if plot_control_points:
        ax, = axis.plot(*a, '+', **kwargs)
        if "color" in kwargs:
            _ = axis.plot(*discrete_traj(a, N_coeff), **kwargs)
        else:
            _ = axis.plot(*discrete_traj(a, N_coeff), color=ax.get_color(), **kwargs)
    else:
        _ = axis.plot(*discrete_traj(a, N_coeff), **kwargs)
    #plt.pause(1e-3)


def _init_traj_gen(N_coeff, T, init_conditions, jitODE):
    p0 = init_conditions[0, :]
    v0 = init_conditions[1, :]
    a0 = init_conditions[2, :]
    af = np.array([0.0, 0.0])
    q_0 = [p0[0], p0[1], atan2(v0[1], v0[0])]
    xi_0 = [v0[0], 0.0, 0.0]
    jitODE.set_integrator("dopri5")
    jitODE.set_default_initial_state({
        jitODE.states["xi"]: xi_0,
        jitODE.states["q"]: q_0
    })
    M = np.block([
        [1, np.zeros(N_coeff - 1)],
        [np.zeros(N_coeff - 1), 1],
        [-N_coeff / T, N_coeff / T, np.zeros(N_coeff - 2)],
        [np.zeros(N_coeff - 2), -N_coeff / T, N_coeff / T],
        [N_coeff * (N_coeff - 1) / T ** 2, -2 * N_coeff * (N_coeff - 1) / T ** 2, N_coeff * (N_coeff - 1) / T ** 2,
         np.zeros(N_coeff - 3)],
        [np.zeros(N_coeff - 3), N_coeff * (N_coeff - 1) / T ** 2, -2 * N_coeff * (N_coeff - 1) / T ** 2,
         N_coeff * (N_coeff - 1) / T ** 2]
    ])
    M = block_diag(M, M)
    M_inv = pinv(M)
    r_inf = 0.3
    r_sup = 1
    return M_inv, a0, af, p0, r_inf, r_sup, v0


def _generate_random_traj(M_inv, N_coeff, N_lc, a0, af, p0, r_inf, r_sup, v0):
    # Final position is in a half disc:
    r1 = random.random() * np.pi  # uniform angle: [0, 90]
    r2 = r_inf + random.random() * (r_sup - r_inf)  # uniform radius [0.3, 1]  m
    thetaf = np.pi * (2 * random.random() - 1)
    pf = r2 * np.array([np.cos(r1), np.sin(r1)])  # final position in the top half ring
    vf = 0.1 * np.array([np.cos(thetaf), np.sin(thetaf)])  # final velocity != 0
    # Computing trajectory limit conditions
    b = [p0[0], pf[0], v0[0], vf[0], a0[0], af[0], p0[1], pf[1], v0[1], vf[1], a0[1], af[1]]
    a = M_inv @ b  # random initial guess
    c = 1
    random_amplitude = 0.1
    w_v = 10
    w_a = 0
    for i in range(N_lc, N_coeff - N_lc):
        w_f = c / (N_coeff - 2 * N_lc + 1)
        w_0 = 1 - w_f
        a[i] = w_0 * (b[0] + w_v * b[2] + w_a * b[4]) + w_f * (b[1] - w_v * b[3] - w_a * b[5])
        a[i + N_coeff] = w_0 * (b[6] + w_v * b[8] + w_a * b[10]) + w_f * (b[7] - w_v * b[9] - w_a * b[11])
        c = c + 1
    return a


def init_trajectory(N_coeff: int, N_lc: int, init_conditions, T:float, umin, umax, jitODE: JitParam, N_controllable):

    time_vec = np.linspace(0.0, T, 100)

    M_inv, a0, af, p0, r_inf, r_sup, v0 = _init_traj_gen(N_coeff, T, init_conditions, jitODE)
    #plt.ion()
    #plt.figure()
    for _ in range(100):
        a = _generate_random_traj(M_inv, N_coeff, N_lc, a0, af, p0, r_inf, r_sup, v0)
        #plot_bspline(a, N_coeff, True)
        jitODE["a"] = a
        jitODE.set_initial_value()
        jitODE.apply_parameters()
        try:
            states_vec = jitODE.integrate_on_time_vector(time_vec)
            if np.all(nonlcon(np.array([]), umin, umax, jitODE) < 0):
                return a
        except UnsuccessfulIntegration:
            pass

    raise RuntimeError("Could not generate a valid  trajectory")

def _pre_opti_cost(a_ctrl, grad, a_init, control_mask, model, time_vector):
    a = a_init.copy()
    a[control_mask] = a_ctrl

    model.ODE["a"] = a
    model.ODE.set_initial_value()
    states = model.ODE.integrate_on_time_vector(time_vector)

    return _pre_opti_cost_from_states(model, states, model.ODE.time_points)


def _pre_opti_cost_from_states(model: Model, states, time_vec):
    curve = states[:, model.output_indices()]
    dcurve = np.diff(curve.T).T
    # length
    length = np.sum(norm(dcurve, axis=1))
    # curvature
    max_curve = 1 / min_curvature_radius(curve, time_vec, dcurve)
    # print(length, max_curve, dispersion)
    kl = 5
    kc = 0.01
    return kl * length + kc * max_curve + 100*(time_vec[-1] - time_vec[0])


def pre_opti_trajectory(N_coeff: int, N_lc: int, init_conditions, T:float, umin, umax, jitODE: JitParam, control_mask,
                        nonlcon=lambda x,y: -1):
    N = 100
    time_vec = np.linspace(0.0, T, N)

    N_controllable = np.sum(control_mask)

    M_inv, a0, af, p0, r_inf, r_sup, v0 = _init_traj_gen(N_coeff, T, init_conditions, jitODE)
    a = _generate_random_traj(M_inv, N_coeff, N_lc, a0, af, p0, r_inf, r_sup, v0)

    a_ctrl = a[control_mask]
    opt = nlopt.opt(nlopt.LN_COBYLA, len(a_ctrl))
    def nlc(a_ctrl, grad):
        if np.any(jitODE["a"][control_mask] != a_ctrl) :
            raise RuntimeError("nlc computed on wrong state")
        states = jitODE.last_result
        cond = nonlcon(np.array([]), umin, umax, jitODE)
        return max(cond)

    opt.add_inequality_constraint(nlc, 1e-6)
    opt.set_ftol_rel(1e-4)
    opt.set_maxeval(100)

    # optimize
    f_cost = lambda a_ctrl, grad: _pre_opti_cost(a_ctrl, grad, a, control_mask, jitODE, time_vec)
    f_cost(a_ctrl, None)
    opt.set_min_objective(f_cost)
    a_opt = opt.optimize(a_ctrl)
    a_out = a.copy()
    a_out[control_mask] = a_opt
    print(opt.last_optimize_result(), opt.get_numevals())
    return a_out


def traj_subplots(xlim=(-2, 2), ylim=(-0.5, 1.5), zlim=(-0.5, 1.5), projection=None):
    if projection == "3d":
        fig = plt.figure()
        axs = np.reshape([fig.add_subplot(2, 2, i+1, projection='3d') for i in range(4)], (2, 2))
        for ax, title in zip(axs.flatten(), ("INIT", "PI", "THETA", "ALL")):
            ax.set_title(title)
            ax.set(xlim=xlim, ylim=ylim, zlim=zlim)
    else:
        fig, axs = plt.subplots(2, 2, sharex=True, sharey=True)
        for ax, title in zip(axs.flatten(), ("INIT", "PI", "THETA", "ALL")):
            ax.axis("square")
            ax.set_title(title)
            ax.set(xlim=xlim, ylim=ylim)
    return fig, axs


def min_curvature_radius(curve, time_vec, dcurve=None):
    """
    Input must be of form (N_samples, N_outputs)
    :param curve:
    :param time_vec:
    :param dcurve:
    :return:
    """
    dt_vec = np.diff(time_vec)
    if dcurve is None:
        dcurve = (np.diff(curve.T) / dt_vec).T
    ddcurve = (np.diff(dcurve.T) / dt_vec[:-1]).T
    dcurve = dcurve[:ddcurve.shape[0], :]
    #r_curve = (dcurve[:, 0] ** 2 + dcurve[:, 1] ** 2) ** (3 / 2) / (
    #            dcurve[:, 0] * ddcurve[:, 1] - dcurve[:, 1] * ddcurve[:, 0])
    if curve.shape[1] == 2:
        r_curve = norm(dcurve, axis=1)**3/np.cross(ddcurve, dcurve, axis=1)
    elif curve.shape[1] == 3:
        if np.any(norm(np.cross(ddcurve, dcurve, axis=1), axis=1) == 0.0):
            print(end="")
        r_curve = norm(dcurve, axis=1)**3/norm(np.cross(ddcurve, dcurve, axis=1), axis=1)
    else:
        raise ValueError("Must be of dimension (N_samples, 2) or (N_samples, 3)")
    #r_curve = norm(dcurve, axis=1) ** 3 / np.sqrt(
    #    np.sum(ddcurve**2, axis=1)*np.sum(dcurve**2, axis=1) - np.sum(ddcurve*dcurve, axis=1)**2)
    return norm(r_curve, ord=-np.inf)
