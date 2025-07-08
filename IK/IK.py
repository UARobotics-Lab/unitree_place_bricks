# numpy provides import array and linear algebra utilities
import numpy as np
# the robotics toolbox provides robotics specific functionality
import roboticstoolbox as rtb
# ansitable is a great package for printing tables in a terminal
from ansitable import ANSITable
# python mechanisms to create abstract classes
from abc import ABC, abstractmethod
# a package for creating dynamic progress bars
from progress.bar import Bar
# swift is a lightweight browser-based simulator which comes with the toolbox
from swift import Swift
# spatialgeometry is a utility package for dealing with geometric objects
import spatialgeometry as sg
# provides sleep functionaly
import time
# Typing utilities
from typing import Optional, Any
# A set of quadratic programming sovlers
import qpsolvers as qp
# Allows us make method wrappers
from functools import wraps
# suppress warnings
import warnings
warnings.filterwarnings('ignore')

""" from roboticstoolbox import models
from roboticstoolbox import Robot
from spatialmath import SE3
from swift import Swift
 """
# This is the base class for all Inverse Kinematics (IK) solvers.
# It provides the basic structure and functionality
# for performing numerical inverse kinematics.

#Visualization of the IK solutions



#Se presentan distintos metodos numericos para resolver el problema de cinemática inversa (IK). >6 DOF


class IK(ABC):
    """
    An abstract super class which provides basic functionality to perform numerical inverse
    kinematics (IK). Superclasses can inherit this class and implement the solve method.

    This class also provides a mechanism to collect data on performance for large scale
    experiments.
    """

    def __init__(
        self,
        name: str = "IK Solver",
        ilimit: int = 30,
        slimit: int = 100,
        tol: float = 1e-6, #tolerancia de error
        we: np.ndarray = np.ones(6),
        problems: int = 1000,
        reject_jl: bool = True,
        λΣ: float=0.0,
        λm: float=0.0, 
        ps: float=0.1,
        pi: Optional[np.ndarray]=None,
    ):
        """
        name: The name of the IK algorithm
        ilimit: How many iterations are allowed within a search before a new search is started
        slimit: How many searches are allowed before being deemed unsuccessful
        tol: Maximum allowed residual error E
        we: A 6 vector which assigns weights to Cartesian degrees-of-freedom
        problems: Total number of IK problems within the experiment
        reject_jl: Reject solutions with joint limit violations
        λΣ: The gain for joint limit avoidance. Setting to 0.0 will remove this completely from the solution
        λm: The gain for maximisation. Setting to 0.0 will remove this completely from the solution
        ps: The minimum angle/distance (in radians or metres) in which the joint is allowed to approach to its limit
        pi: The influence angle/distance (in radians or metres) in null space motion becomes active
        """

        # Solver parameters
        self.name = name
        self.slimit = int(slimit)
        self.ilimit = int(ilimit)
        self.tol = tol
        self.We = np.diag(we)
        self.reject_jl = reject_jl
        self.λΣ = λΣ
        self.λm = λm
        self.ps = ps
        self.pi = pi

        # Solver results
        self.success = np.zeros(problems)
        self.searches = np.zeros(problems)
        self.iterations = np.zeros(problems)
        self.valid_jl = np.zeros(problems)
        self.times = np.zeros(problems)

        # initialise with NaN
        self.searches[:] = np.nan
        self.iterations[:] = np.nan
        self.success[:] = np.nan
        self.valid_jl[:] = np.nan
        self.times[:] = np.nan

    def solve(self, ets: rtb.ETS, Tep: np.ndarray, q0: np.ndarray):
        """
        This method will attempt to solve the IK problem and obtain joint coordinates
        which result the the end-effector pose Tep.

        The method returns a tuple:
        q: The joint coordinates of the solution (ndarray). Note that these will not
            be valid if failed to find a solution
        success: True if a solution was found (boolean)
        iterations: The number of iterations it took to find the solution (int)
        searches: The number of searches it took to find the solution (int)
        residual: The residual error of the solution (float)
        jl_valid: True if joint coordinates q are within the joint limits
        total_t: The total time spent within the step method
        """

        # Iteration count
        i = 0
        total_i = 0
        total_t = 0.0

        for search in range(self.slimit):
            q = q0[search].copy()
            
            while i <= self.ilimit:
                i += 1

                # Attempt a step
                try:
                    t, E, q = self.step(ets, Tep, q)

                    # Acclumulate total time
                    total_t += t
                except np.linalg.LinAlgError:
                    # Abandon search and try again
                    break

                # Check if we have arrived
                if E < self.tol:

                    # Wrap q to be within +- 180 deg
                    # If your robot has larger than 180 deg range on a joint
                    # this line should be modified in incorporate the extra range
                    q = (q + np.pi) % (2 * np.pi) - np.pi

                    # Check if we have violated joint limits
                    jl_valid = self.check_jl(ets, q)

                    if not jl_valid and self.reject_jl:
                        # Abandon search and try again
                        break
                    else:
                        return q, True, total_i + i, search + 1, E, jl_valid, total_t

            total_i += i
            i = 0

        # If we make it here, then we have failed
        return q, False, np.nan, np.nan, E, np.nan, np.nan

    def error(self, Te: np.ndarray, Tep: np.ndarray):
        """
        Calculates the engle axis error between current end-effector pose Te and
        the desired end-effector pose Tep. Also calulates the quadratic error E
        which is weighted by the diagonal matrix We.

        Returns a tuple:
        e: angle-axis error (ndarray in R^6)
        E: The quadratic error weighted by We
        """
        e = rtb.angle_axis(Te, Tep) #Tep: Tgoal pose, Te: current pose
        E = 0.5 * e @ self.We @ e

        return e, E

    def check_jl(self, ets: rtb.ETS, q: np.ndarray):
        """
        Checks if the joints are within their respective limits

        Returns a True if joints within feasible limits otherwise False
        """

        # Loop through the joints in the ETS
        for i in range(ets.n):

            # Get the corresponding joint limits
            ql0 = ets.qlim[0, i]
            ql1 = ets.qlim[1, i]

            # Check if q exceeds the limits
            if q[i] < ql0 or q[i] > ql1:
                return False

        # If we make it here, all the joints are fine
        return True

    @abstractmethod
    def step(self, ets: rtb.ETS, Tep: np.ndarray, q: np.ndarray):
        """
        Superclasses will implement this method to perform a step of the implemented
        IK algorithm
        """
        pass

def timing(func):
    @wraps(func)
    def wrap(*args, **kw):
        t_start = time.time()
        E, q = func(*args, **kw)
        t_end = time.time()
        t = t_end - t_start
        return t, E, q
    return wrap

def null_Σ(ets: rtb.ETS, q: np.ndarray, ps: float, pi: Optional[np.ndarray]):
    """
    Formulates a relationship between joint limits and the joint velocity.
    When this is projected into the null-space of the differential kinematics
    to attempt to avoid exceeding joint limits

    q: The joint coordinates of the robot
    ps: The minimum angle/distance (in radians or metres) in which the joint is
        allowed to approach to its limit
    pi: The influence angle/distance (in radians or metres) in which the velocity
        damper becomes active

    returns: Σ 
    """

    # If pi wasn't supplied, set it to some default value
    if pi is None:
        pi = 0.3 * np.ones(ets.n)

    # Add cost to going in the direction of joint limits, if they are within
    # the influence distance
    Σ = np.zeros((ets.n, 1))

    for i in range(ets.n):
        qi = q[i]
        ql0 = ets.qlim[0, i]
        ql1 = ets.qlim[1, i]

        if qi - ql0 <= pi[i]:
            Σ[i, 0] = (
                -np.power(((qi - ql0) - pi[i]), 2) / np.power((ps - pi[i]), 2)
            )
        if ql1 - qi <= pi[i]:
            Σ[i, 0] = (
                np.power(((ql1 - qi) - pi[i]), 2) / np.power((ps - pi[i]), 2)
            )

    return -Σ

def calc_qnull(
        ets: rtb.ETS,
        q: np.ndarray,
        J: np.ndarray,
        λΣ: float,
        λm: float,
        ps: float,
        pi: Optional[np.ndarray]
    ):
    """
    Calculates the desired null-space motion according to the gains λΣ and λm.
    This is a helper method that is used within the `step` method of an IK solver

    Returns qnull: the desired null-space motion
    """

    qnull_grad = np.zeros(ets.n)
    qnull = np.zeros(ets.n)

    # Add the joint limit avoidance if the gain is above 0
    if λΣ > 0:
        Σ = null_Σ(ets, q, ps, pi)
        qnull_grad += (1.0 / λΣ * Σ).flatten()

    # Add the manipulability maximisation if the gain is above 0
    if λm > 0:
        Jm = ets.jacobm(q)
        qnull_grad += (1.0 / λm * Jm).flatten()

    # Calculate the null-space motion
    if λΣ > 0 or λΣ > 0:
        null_space = (np.eye(ets.n) - np.linalg.pinv(J) @ J)
        qnull = null_space @ qnull_grad

    return qnull.flatten()

class NR(IK):
    """Newton-Raphson Inverse Kinematics."""
    def __init__(self, pinv=False, **kwargs):
        super().__init__(**kwargs)
        self.pinv = pinv

        self.name = f"NR (pinv={pinv})"

        if self.λΣ > 0.0:
            self.name += ' Σ'

        if self.λm > 0.0:
            self.name += ' Jm'

    @timing
    def step(self, ets: rtb.ETS, Tep: np.ndarray, q: np.ndarray):
        Te = ets.eval(q)
        e, E = self.error(Te, Tep)

        J = ets.jacob0(q)

        # Null-space motion
        qnull = calc_qnull(ets, q, J, self.λΣ, self.λm, self.ps, self.pi)

        if self.pinv:
            q += np.linalg.pinv(J) @ e + qnull
        else:
            q += np.linalg.inv(J) @ e + qnull

        return E, q

class GN(IK):
    """Gauss-Newton Inverse Kinematics."""
    def __init__(self, pinv=False, **kwargs):
        super().__init__(**kwargs)
        self.pinv = pinv

        self.name = f"GN (pinv={pinv})"

        if self.λΣ > 0.0:
            self.name += ' Σ'

        if self.λm > 0.0:
            self.name += ' Jm'

    @timing
    def step(self, ets: rtb.ETS, Tep: np.ndarray, q: np.ndarray):
        Te = ets.eval(q)
        e, E = self.error(Te, Tep)

        J = ets.jacob0(q)
        g = J.T @ self.We @ e

        # Null-space motion
        qnull = calc_qnull(ets, q, J, self.λΣ, self.λm, self.ps, self.pi)

        if self.pinv:
            q += np.linalg.pinv(J.T @ self.We @ J) @ g + qnull
        else:
            q += np.linalg.inv(J.T @ self.We @ J) @ g + qnull

        return E, q        

class LM_Wampler(IK):
    
    """Levenberg-Marquardt Inverse Kinematics using Wampler's method.
    This method is a variant of the Levenberg-Marquardt algorithm that incorporates
    joint limit avoidance and manipulability maximisation.
     
     Es mas preciso que el NR y GN, pero mas lento.
     
    """
    def __init__(self, λ: float = 1.0, **kwargs: Any):
        super().__init__(**kwargs)
        
        self.name = f"LM (Wampler λ={λ})"
        self.λ = λ

        if self.λΣ > 0.0:
            self.name += ' Σ'

        if self.λm > 0.0:
            self.name += ' Jm'

    @timing
    def step(self, ets: rtb.ETS, Tep: np.ndarray, q: np.ndarray):
        Te = ets.eval(q)
        e, E = self.error(Te, Tep)

        Wn = self.λ * np.eye(ets.n)
        J = ets.jacob0(q)
        g = J.T @ self.We @ e

        # Null-space motion
        qnull = calc_qnull(ets, q, J, self.λΣ, self.λm, self.ps, self.pi)

        q += np.linalg.inv(J.T @ self.We @ J + Wn) @ g + qnull

        return E, q
    
class LM_Chan(IK):
    """Levenberg-Marquardt Inverse Kinematics using Chan's method."""

    def __init__(self, λ=1.0, **kwargs):
        super().__init__(**kwargs)
        
        self.name = f"LM (Chan λ={λ})"
        self.λ = λ

        if self.λΣ > 0.0:
            self.name += ' Σ'

        if self.λm > 0.0:
            self.name += ' Jm'

    @timing
    def step(self, ets: rtb.ETS, Tep: np.ndarray, q: np.ndarray):
        Te = ets.eval(q)
        e, E = self.error(Te, Tep)

        Wn = self.λ * E * np.eye(ets.n)
        J = ets.jacob0(q)
        g = J.T @ self.We @ e

        # Null-space motion
        qnull = calc_qnull(ets, q, J, self.λΣ, self.λm, self.ps, self.pi)

        q += np.linalg.inv(J.T @ self.We @ J + Wn) @ g + qnull

        return E, q

class LM_Sugihara(IK):

    """Levenberg-Marquardt Inverse Kinematics using Sugihara's method."""

    def __init__(self, name="LM (Sugihara)", λ=1.0, **kwargs):
        super().__init__(name, **kwargs)

        self.name = f"LM (Sugihara λ={λ})"
        self.λ = λ

        if self.λΣ > 0.0:
            self.name += ' Σ'

        if self.λm > 0.0:
            self.name += ' Jm'

    @timing
    def step(self, ets: rtb.ETS, Tep: np.ndarray, q: np.ndarray):
        Te = ets.eval(q)
        e, E = self.error(Te, Tep)

        Wn = E * np.eye(ets.n) + self.λ * np.eye(ets.n)
        J = ets.jacob0(q)
        g = J.T @ self.We @ e

        # Null-space motion
        qnull = calc_qnull(ets, q, J, self.λΣ, self.λm, self.ps, self.pi)

        q += np.linalg.inv(J.T @ self.We @ J + Wn) @ g + qnull

        return E, q

class QP(IK):

    """Quadratic Programming Inverse Kinematics using a quadratic cost function."""

    def __init__(self, name="QP", λj=1.0, λs=1.0, **kwargs):
        super().__init__(name, **kwargs)

        self.name = f"QP (λj={λj}, λs={λs})"
        self.λj = λj
        self.λs = λs

        if self.λΣ > 0.0:
            self.name += ' Σ'

        if self.λm > 0.0:
            self.name += ' Jm'

    @timing
    def step(self, ets: rtb.ETS, Tep: np.ndarray, q: np.ndarray):
        Te = ets.eval(q)
        e, E = self.error(Te, Tep)
        J = ets.jacob0(q)

        # Quadratic component of objective function
        Q = np.eye(ets.n + 6)

        # Joint velocity component of Q
        Q[: ets.n, : ets.n] *= self.λj

        # Slack component of Q
        Q[ets.n :, ets.n :] = self.λs * (1 / np.sum(np.abs(e))) * np.eye(6)

        # The equality contraints
        Aeq = np.concatenate((J, np.eye(6)), axis=1)
        beq = e.reshape((6,))

        # The inequality constraints for joint limit avoidance
        if self.λΣ > 0.0:
            Ain = np.zeros((ets.n + 6, ets.n + 6))
            bin = np.zeros(ets.n + 6)

            # Form the joint limit velocity damper
            Ain_l = np.zeros((ets.n, ets.n))
            Bin_l = np.zeros(ets.n)

            for i in range(ets.n):
                ql0 = ets.qlim[0, i]
                ql1 = ets.qlim[1, i]

                if ql1 - q[i] <= self.pi[i]:
                    Bin_l[i] = ((ql1 - q[i]) - self.ps) / (self.pi[i] - self.ps)
                    Ain_l[i, i] = 1

                if q[i] - ql0 <= self.pi[i]:
                    Bin_l[i] = -(((ql0 - q[i]) + self.ps) / (self.pi[i] - self.ps))
                    Ain_l[i, i] = -1

            Ain[: ets.n, : ets.n] = Ain_l
            bin[: ets.n] =  (1.0 / self.λΣ) * Bin_l
        else:
            Ain = None
            bin = None
        
        # Manipulability maximisation
        if self.λm > 0.0:
            Jm = ets.jacobm(q).reshape((ets.n,))
            c = np.concatenate(((1.0 / self.λm) * -Jm, np.zeros(6)))
        else:
            c = np.zeros(ets.n + 6)

        xd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=None, ub=None, solver='quadprog')

        q += xd[: ets.n]

        return E, q
    



