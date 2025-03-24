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

import numpy as np

# second order trajectory. bounded velocity and acceleration.
class Trajectory2:
	def __init__(self, ts = 1.0, vmax = 2.0 ,amax = 1.0):
		self.ts = ts
		self.vmax = vmax
		self.amax = amax
		self.x = float(0)
		self.target = 0
		self.v = 0 
		self.a = 0
		self.t = 0
		self.vn = 0 # next velocity
 
	def setTarget(self, T):
		self.target = T
 
	def setX(self, x):
		self.x = x
 
	def run(self):
		self.t = self.t + self.ts
		sig = numpy.sign( self.target - self.x ) # direction of move
 
		tm = 0.5*self.ts + math.sqrt( pow(self.ts,2)/4 - (self.ts*sig*self.v-2*sig*(self.target-self.x)) / self.amax )
		if tm >= self.ts:
			self.vn = sig*self.amax*(tm - self.ts)
			# constrain velocity
			if abs(self.vn) > self.vmax:
				self.vn = sig*self.vmax
		else:
			# done (almost!) with move
			self.a = float(0.0-sig*self.v)/float(self.ts)
			if not (abs(self.a) <= self.amax):
				# cannot decelerate directly to zero. this branch required due to rounding-error (?)
				self.a = numpy.sign(self.a)*self.amax
				self.vn = self.v + self.a*self.ts
				self.x = self.x + (self.vn+self.v)*0.5*self.ts
				self.v = self.vn
				assert( abs(self.a) <= self.amax )
				assert( abs(self.v) <= self.vmax )
				return True
			else:
				# end of move
				assert( abs(self.a) <= self.amax )
				self.v = self.vn
				self.x = self.target
				return False
 
		# constrain acceleration
		self.a = (self.vn-self.v)/self.ts
		if abs(self.a) > self.amax:
			self.a = numpy.sign(self.a)*self.amax
			self.vn = self.v + self.a*self.ts
 
		# update position
		#if sig > 0:
		self.x = self.x + (self.vn+self.v)*0.5*self.ts
		self.v = self.vn
		assert( abs(self.v) <= self.vmax )
		#else:
		#	self.x = self.x + (-vn+self.v)*0.5*self.ts
		#	self.v = -vn
		return True
 
	def zeropad(self):
		self.t = self.t + self.ts
 
	def prnt(self):
		print ("%.3f\t%.3f\t%.3f\t%.3f", self.t, self.x, self.v, self.a )
 
	def __str__(self):
		return "2nd order Trajectory."

class SingleAxisTrajectory:
    """A trajectory along one axis.
    
    This is used to construct the optimal trajectory in one axis, planning
    in the jerk to achieve position, velocity, and/or acceleration final
    conditions. The trajectory is initialised with a position, velocity and
    acceleration. 
    
    The trajectory is optimal with respect to the integral of jerk squared.
    
    Do not use this in isolation, this useful through the "RapidTrajectory"
    class, which wraps three of these and allows to test input/state 
    feasibility.

    """

    def __init__(self, pos0, vel0, acc0):
        """Initialise the trajectory with starting state."""
        self._p0 = pos0
        self._v0 = vel0
        self._a0 = acc0
        self._pf = 0
        self._vf = 0
        self._af = 0
        self.reset()

    def set_goal_position(self, posf):
        """Define the goal position for a trajectory."""
        self._posGoalDefined = True
        self._pf = posf

    def set_goal_velocity(self, velf):
        """Define the goal velocity for a trajectory."""
        self._velGoalDefined = True
        self._vf = velf

    def set_goal_acceleration(self, accf):
        """Define the goal acceleration for a trajectory."""
        self._accGoalDefined = True
        self._af = accf

    def generate(self, Tf):
        """ Generate a trajectory of duration Tf.

        Generate a trajectory, using the previously defined goal end states 
        (such as position, velocity, and/or acceleration).

        """
        #define starting position:
        delta_a = self._af - self._a0
        delta_v = self._vf - self._v0 - self._a0*Tf
        delta_p = self._pf - self._p0 - self._v0*Tf - 0.5*self._a0*Tf*Tf

        #powers of the end time:
        T2 = Tf*Tf
        T3 = T2*Tf
        T4 = T3*Tf
        T5 = T4*Tf

        #solve the trajectories, depending on what's constrained:
        if self._posGoalDefined and self._velGoalDefined and self._accGoalDefined:
            self._a = ( 60*T2*delta_a - 360*Tf*delta_v + 720* 1*delta_p)/T5
            self._b = (-24*T3*delta_a + 168*T2*delta_v - 360*Tf*delta_p)/T5
            self._g = (  3*T4*delta_a -  24*T3*delta_v +  60*T2*delta_p)/T5
        elif self._posGoalDefined and self._velGoalDefined:
            self._a = (-120*Tf*delta_v + 320*   delta_p)/T5
            self._b = (  72*T2*delta_v - 200*Tf*delta_p)/T5
            self._g = ( -12*T3*delta_v +  40*T2*delta_p)/T5
        elif self._posGoalDefined and self._accGoalDefined:
            self._a = (-15*T2*delta_a + 90*   delta_p)/(2*T5)
            self._b = ( 15*T3*delta_a - 90*Tf*delta_p)/(2*T5)
            self._g = (- 3*T4*delta_a + 30*T2*delta_p)/(2*T5)
        elif self._velGoalDefined and self._accGoalDefined:
            self._a = 0
            self._b = ( 6*Tf*delta_a - 12*   delta_v)/T3
            self._g = (-2*T2*delta_a +  6*Tf*delta_v)/T3
        elif self._posGoalDefined:
            self._a =  20*delta_p/T5
            self._b = -20*delta_p/T4
            self._g =  10*delta_p/T3
        elif self._velGoalDefined:
            self._a = 0
            self._b =-3*delta_v/T3
            self._g = 3*delta_v/T2
        elif self._accGoalDefined:
            self._a = 0
            self._b = 0
            self._g = delta_a/Tf
        else:
            #Nothing to do!
            self._a = self._b = self._g = 0

        #Calculate the cost:
        self._cost =  (self._g**2) + self._b*self._g*Tf + (self._b**2)*T2/3.0 + self._a*self._g*T2/3.0 + self._a*self._b*T3/4.0 + (self._a**2)*T4/20.0
                
    def reset(self):
        """Reset the trajectory parameters."""
        self._cost = float("inf")
        self._accGoalDefined = self._velGoalDefined = self._posGoalDefined = False
        self._accPeakTimes = [None,None]
        pass
    
    def get_jerk(self, t):
        """Return the scalar jerk at time t."""
        return self._g  + self._b*t  + (1.0/2.0)*self._a*t*t
    
    def get_acceleration(self, t):
        """Return the scalar acceleration at time t."""
        return self._a0 + self._g*t  + (1.0/2.0)*self._b*t*t  + (1.0/6.0)*self._a*t*t*t

    def get_velocity(self, t):
        """Return the scalar velocity at time t."""
        return self._v0 + self._a0*t + (1.0/2.0)*self._g*t*t  + (1.0/6.0)*self._b*t*t*t + (1.0/24.0)*self._a*t*t*t*t

    def get_position(self, t):
        """Return the scalar position at time t."""
        return self._p0 + self._v0*t + (1.0/2.0)*self._a0*t*t + (1.0/6.0)*self._g*t*t*t + (1.0/24.0)*self._b*t*t*t*t + (1.0/120.0)*self._a*t*t*t*t*t

    def get_min_max_acc(self, t1, t2):
        """Return the extrema of the acceleration trajectory between t1 and t2."""
        if self._accPeakTimes[0] is None:
            #uninitialised: calculate the roots of the polynomial
            if self._a:
                #solve a quadratic
                det = self._b*self._b - 2*self._g*self._a
                if det<0:
                    #no real roots
                    self._accPeakTimes[0] = 0
                    self._accPeakTimes[1] = 0
                else:
                    self._accPeakTimes[0] = (-self._b + np.sqrt(det))/self._a
                    self._accPeakTimes[1] = (-self._b - np.sqrt(det))/self._a
            else:
                #_g + _b*t == 0:
                if self._b:
                    self._accPeakTimes[0] = -self._g/self._b
                    self._accPeakTimes[1] = 0
                else:
                    self._accPeakTimes[0] = 0
                    self._accPeakTimes[1] = 0

        #Evaluate the acceleration at the boundaries of the period:
        aMinOut = min(self.get_acceleration(t1), self.get_acceleration(t2))
        aMaxOut = max(self.get_acceleration(t1), self.get_acceleration(t2))

        #Evaluate at the maximum/minimum times:
        for i in [0,1]:
            if self._accPeakTimes[i] <= t1: continue
            if self._accPeakTimes[i] >= t2: continue
            
            aMinOut = min(aMinOut, self.get_acceleration(self._accPeakTimes[i]))
            aMaxOut = max(aMaxOut, self.get_acceleration(self._accPeakTimes[i]))
        return (aMinOut, aMaxOut)
 
    def get_max_jerk_squared(self,t1, t2):
        """Return the extrema of the jerk squared trajectory between t1 and t2."""
        jMaxSqr = max(self.get_jerk(t1)**2,self.get_jerk(t2)**2)
        
        if self._a:
            tMax = -self._b/self._a
            if(tMax>t1 and tMax<t2):
                jMaxSqr = max(pow(self.get_jerk(tMax),2),jMaxSqr)

        return jMaxSqr


    def get_param_alpha(self):
        """Return the parameter alpha which defines the trajectory."""
        return self._a

    def get_param_beta (self):
        """Return the parameter beta which defines the trajectory."""
        return self._b

    def get_param_gamma(self):
        """Return the parameter gamma which defines the trajectory."""
        return self._g

    def get_initial_acceleration(self):
        """Return the start acceleration of the trajectory."""
        return self._a0

    def get_initial_velocity(self):
        """Return the start velocity of the trajectory."""
        return self._v0

    def get_initial_position(self):
        """Return the start position of the trajectory."""
        return self._p0

    def get_cost(self):
        """Return the total cost of the trajectory."""
        return self._cost