# -*- coding: utf-8 -*-
"""
Script to pre-calculate the paths for leg targets for biped robot.

This script uses the zero-moment-point method to calculate a biped 
robot movement path. The script simulates the robot's motion and 
then writes out the movement path of the center of mass which will 
keep the robot stable during movement. Also the leg position paths 
are exported.

The parameters g (gravity), z (height of center-of-mass) and m (mass of the robot)
needs to be set for correct simulation. Also the step pattern (create_step_pattern call)
should have correct step length and width.

The script generates three path files:
"biped_cog_path.csv", "biped_left_leg_path.csv", and "biped_right_leg_path.csv"
they all need to be imported into the V-REP model and attached under 
correct objects.


Copyright (C) 2019 Lauri Peltonen

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""


import time
from math import sin, pi, atan2, sqrt, ceil, cos

# Needs scipy to solve the gain values
from scipy.linalg import solve_discrete_are
import numpy as np
from numpy.linalg import inv

def create_system(Ts=5.e-3, Zc=0.8, G=9.81, Mg=16.0):
    """Create a dual axis (3D) state-space presentation of the cart-table model
    enhanced with the horizontal angular momentum.
    
    Model is according to publication [1]
    General ZMP preview control for bipedal walking
    Jonghoon Park, Youngil Youm
    IEEE international conference on robotics and automation
    April 2007, p. 2682-2687
    The system is in form of
    X(k+1) = A*X(k) + B*U(k)
    P(k+1) = C*X(k)
    
    where
         
        | I   T*I  0  (T^2)/2*I   0 |        | (T^3)/6*I       0    |
        | 0   I    0     T        0 |        | (T^2)/2*I       0    |
    A = | 0   0    I     0      T*I |    B = |    0       (T^2)/2*I |
        | 0   0    0     I        0 |        |   T*I           0    |
        | 0   0    0     0        I |        |    0           T*I   |
        
    C = | I  0  0  -Zc/G*I  1/Mg*S |
    
    where I = 2x2 identity matrix, 0 = 2x2 zero matrix and S = | 0  -1 |
                                                               | 1   0 |
    
    X = [ x y dx dy Hx Hy ddx ddy dHx dHy ]^T  (location, speed, acceleration
        of center of mass and angular momentum in X and Y and their derivative)
    U = [ ux uy  ]^T  (input, acceleration of center of mass)
    P = [ px py ]^T  (output, location of the zero moment point)
    
    Input, state and output vectors are initialized to zero.
    
    The output vector contains the zero moment point (ZMP) location in
    X and Y axis. The state vector describes the location of center of mass
    (CoM) in X,Y plane at the pre-defined height Zc. It is assumed to travel
    horizontally. The input is acceleration of the CoM in X and Y directions.
    
    Parameters:
        Ts (float, > 0): Sampling time (step time)
        Zc (float, > 0): Height of the center of mass (CoM)
        G (float, > 0): Acceleration due to gravity (e.g. 9.81)
        Mg (float, > 0): Mass
        
    Returns:
        A: State transition matrix (10x10)
        B: Input matrix (10x4)
        C: Output matrix (2x10)
        X: State vector (10x1), [X Y dX dY Hx Hy ddX ddY dHx dHy]^T of CoM
        U: Input vector (2x1), [ddX dHx ddY dHy]^T acceleration of CoM
        P: Output vector (2x1), [px py]^T location of the zero moment point (ZMP)
    """
    # Dual axis system with horizontal angular momentum
    A = np.array([[1, 0, Ts,  0,  0, 0, Ts*Ts/2,       0,  0,  0],
                  [0, 1,  0, Ts,  0, 0,       0, Ts*Ts/2,  0,  0],
                  [0, 0,  1,  0,  0, 0,      Ts,       0,  0,  0],
                  [0, 0,  0,  1,  0, 0,       0,      Ts,  0,  0],
                  [0, 0,  0,  0,  1, 0,       0,       0, Ts,  0],
                  [0, 0,  0,  0,  0, 1,       0,       0,  0, Ts],
                  [0, 0,  0,  0,  0, 0,       1,       0,  0,  0],
                  [0, 0,  0,  0,  0, 0,       0,       1,  0,  0],
                  [0, 0,  0,  0,  0, 0,       0,       0,  1,  0],
                  [0, 0,  0,  0,  0, 0,       0,       0,  0,  1]])
    
    B = np.array([[Ts*Ts*Ts/6, 0,          0,       0],
                  [0,          Ts*Ts*Ts/6, 0,       0],
                  [Ts*Ts/2,    0,          0,       0],
                  [0,          Ts*Ts/2,    0,       0],
                  [0,          0,          Ts*Ts/2, 0],
                  [0,          0,          0,       Ts*Ts/2],
                  [Ts,         0,          0,       0],
                  [0,          Ts,         0,       0],
                  [0,          0,          Ts,      0],
                  [0,          0,          0,       Ts]])
    
    C = np.array([[1, 0, 0, 0, 0, 0, -Zc/G, 0,     0,    -1/Mg],
                  [0, 1, 0, 0, 0, 0, 0,     -Zc/G, 1/Mg,     0]])
    X = np.zeros(shape=(10,1))   # X, Y, dX, dY, Hx, Hy, ddX, ddY, dHx, dHy
    U = np.zeros(shape=(4,1))   # X, Hx, Y, Hy
    P = np.zeros(shape=(2,1))   # Output X, Y
    
    return (A, B, C, X, U, P)



# Also referenced in e.g. 
# General ZMP preview control for bipedal walking
# Jonghoon Park, Youngil Youm
# IEEE international conference on robotics and automation, 2007, p. 2682-2687
def create_controller(A, B, C, qex, rx, lh, N):
    """Creates an optimal LQI controller for a state-space system.
    
    The method is done according to publication [2]
    Design of an optimal controller for discrete-time system
        subject to previewable demand
    Tohru Katayama
    International Journal of Control, March 1985, vol 41, no. 3, p. 677-699    
 
    Solves an optimal control problem by minimizing
        J = sum(i=k...inf)[ Qe*(pd(i) - p(i))^2 + (dX^T)*Qx*dX + R*dU(i)^2 ]
    
    which leads to a controller of type
        u(k) = -Gi*sum(i=0...k)[p(i) - pd(i)] - Gx*X(k) - sum(i=1...N)[G(i)*pd(k+i)]
    
    where
        pd is the desired state of the output (i.e. reference)
        p is the actual output
        Qe is the loss due to tracking error
        Qx is the loss due to incremental state (zero used here)
        R is the loss due to control
        u is the controller output
        Gi is the integrator gain
        Gx is the state control gain
        G is the preview (look-ahead) gain vector
        X is the state vector
        
    This function solves the problem and then outputs the gains of the
    optimal controller. Using this method is referenced also in [1].
    
    qex, R and N can be used to tune the output performance of the controller.
    
    Inputs:
        A: State transition matrix (n x n)
        B: Input matrix (n x r)
        C: Output matrix (1 x p)
        qex (float, > 0): Optimizer loss due to tracking error (default 1.0)
        rx (float, > 0): Optimizer loss due to control (default 1.0e-7)
        N (int, >= 0): Preview controller n. of look-ahead samples (default 320)
        
    Outputs:
        Tuple of:
            Gi: Integrator gain
            Gx: State control gain
            G: Array of look-ahead gains
    """
    
    assert (qex >= 0), "Controller: Qex must be positive"
    assert (rx >= 0), "Controller: Rx must be positive"
    assert (lh >= 0), "Controller: lh (lambda h) must be positive"
    assert (lh != 0), "Controller: lh (lambda h) is zero. System wont be stable"
    
    rn = A.shape[0]     # Size of state matrix
    rr = B.shape[1]     # Length of input vector
    rp = C.shape[0]     # Length of output vector
   
    Ip = np.identity(rp)
    
    # Describe the incremental time system
    # Ã = [[Ip, CA],[0, A]] where Ip is pxp unit (identity) matrix
    # Ã is then n+p x n+p matrix
    Ai = np.zeros(shape=(rn+rp, rn+rp))
    for i in range(rp):
        Ai[i, i] = Ip[i, i] # = 1
    
    CA = np.matmul(C, A)    # p x n
    for i in range(rn):
        for j in range(rp):
            Ai[j, i+rp] = CA[j, i]
        for j in range(rn):
            Ai[i+rp, j+rp] = A[i, j]
    
    # ~B = [[CB],[B]]
    Bi = np.zeros(shape=(rp+rn, rr))
    CB = np.matmul(C, B)    # p x r
    for i in range(rp):
        for j in range(rr):
            Bi[i, j] = CB[i, j]
    for i in range(rn):
        for j in range(rr):
            Bi[i+rp, j] = B[i, j]
    
    # Qe is pxp matrix
    # Qx is nxn matrix
    # R is rxr matrix
    Qe = qex * np.identity(rp)
    Qx = np.zeros(shape=(rn,rn))
    assert (rn==10), "Controller: Lambda h requires A matrix to be 10x10"
    Qx[4, 4] = lh   # TODO: Works only with the original system (10x10)
    Qx[5, 5] = lh   # --"--
    R = rx * np.identity(rr)

    
    # ~Q = [[Qe, 0], [0, Qx]] = n+p x n+p
    Qi = np.zeros(shape=(rn+rp, rn+rp))
    for i in range(rp):
        for j in range(rp):
            Qi[i, j] = Qe[i, j]
    for i in range(rn):
        for j in range(rn):
            Qi[i+rp, j+rp] = Qx[i, j]
               
    # Solve the riccati equation
    Ki = solve_discrete_are(Ai, Bi, Qi, R)
    
    # Calculate the controller gains
    
    # First some constants used in furher calculations
    mTemp = np.matmul(inv(R + np.matmul(np.matmul(Bi.T, Ki), Bi)), Bi.T)
    
    # ~I = [[Ip],[0]] = p+n, p matrix
    Ii = np.zeros(shape=(rp+rn, rp))
    for i in range(rp):
        Ii[i, i] = 1.
    
    # ~F = [[CA], [A]] = right side of Ã
    Fi = np.zeros(shape=(rn+rp,rn))
    for i in range(rn+rp):
        for j in range(rn):
            Fi[i, j] = Ai[i, j+rp]
    
    # Integral gain
    Gi = np.matmul(np.matmul(mTemp, Ki), Ii)
    
    # State feedback gain
    Gx = np.matmul(np.matmul(mTemp, Ki), Fi)
    
    # Then the look-ahead gains
    mTemp2 = inv(R + np.matmul(np.matmul(Bi.T, Ki), Bi))
    Ac = Ai - np.matmul(np.matmul(np.matmul(np.matmul(Bi, mTemp2), Bi.T), Ki), Ai)
    Xx = -np.matmul(np.matmul(Ac.T, Ki), Ii)
    
    G = [-Gi]       # Matrix rxr
    for i in range(N-1):
        mG = np.matmul(mTemp, Xx)
        Xx = np.matmul(Ac.T, Xx)
        
        G.append(mG)
        
    return (Gi, Gx, G)


# Return a trapezoidal function from 0 to 1
# with given rise and fall times
def trapez(begin, end, rise, fall, pos, linear=False):
    if pos < 0.0 or pos < begin:
        return 0.0
    elif pos > 1.0 or pos > end:
        return 0.0
    elif pos < rise:
        if linear:
            return (pos-begin) / (rise - begin)
        else:
            return 0.5 * (1.0 - cos(pi * (pos-begin) / (rise - begin)))
    elif pos > fall:
        if linear:
            return 1.0 - ((pos - fall) / (end - fall))
        else:
            return 0.5 * (1.0 - cos(pi * (1.0 - ((pos - fall) / (end - fall)))))
    else:
        return 1.0

# Do a linear interpolation with selectable begin and end times
def lerp(begin, end, pos, linear=False):
    if pos < 0.0 or pos < begin:
        return 0.0
    elif pos > 1.0 or pos > end:
        return 1.0
    else:
        if linear:
            return (pos - begin) / (end-begin)
        else:
            return 0.5 * (1.0 - cos(pi * (pos - begin) / (end-begin)))

# Advances in Y direction
# dX = leg positions in sideways direction
# dY = step length
# N = number of steps
# Tstep = step duration
# Ts = time step (dt)
# Tend = 
def create_step_pattern(dX, dY, N, Tstep, Ts, Tend):
    # ZMP location
    pxref = []
    pyref = []
    
    # Which foot is the support foot
    stance = []     # 0 = LEFT on ground, 1 = RIGHT on ground
    
    # Foot locations (in world coordinates)
    left_x = []
    left_y = []
    left_z = []
    right_x = []
    right_y = []
    right_z = []
    
    # initial values
    xr = 0
    yr = 0
    stance_leg = 0
    
    # initial foot positions
    lx = dX
    ly = 0
    lz = 0
    rx = -dX
    ry = 0
    rz = 0
    
    steps = int(Tend / Ts)
    nstep = int(Tstep / Ts)
    nlatest = nstep       # Idle one step period
    nlast = (N+2)*nstep         # Last step to do
    
    step_pos = 0.0
    first_step = True
    
    for i in range(steps):
        if i > (nlatest + nstep):
            if i > nlast:
                xr = 0
            elif xr == 0:
                xr = -dX
            else:
                xr = -xr
                yr += dY
                first_step = False

            nlatest = i
        
        if xr == 0:
            step_pos = 0.0
        else:
            step_pos = float(i - nlatest) / nstep
        
        if xr > 0:
            stance_leg = 0  # Left foot on ground
            
            # move right foot
            # Rise above ground
            rz = trapez(0.25, 0.75, 0.4, 0.6, step_pos) * 0.04
            # Move in Y
            ry = lerp(0.3, 0.7, step_pos)
            if first_step:
                ry *= dY
            else:
                 ry = ry * 2. * dY + yr - dY
        else:
            stance_leg = 1  # Right foot (only) on ground
            
            # move left foot
            # Rise above ground
            lz = trapez(0.25, 0.75, 0.4, 0.6, step_pos) * 0.04
            # Move in Y
            ly = lerp(0.3, 0.7, step_pos)
            if first_step:
                ly *= dY
            else:
                ly = ly * 2. * dY + yr - dY


        pxref.append(xr)
        pyref.append(yr)
        stance.append(stance_leg)

        left_x.append(lx)
        left_y.append(ly)
        left_z.append(lz)
        right_x.append(rx)
        right_y.append(ry)
        right_z.append(rz)

    return (pxref, pyref, stance, left_x, left_y, left_z, right_x, right_y, right_z)


def calculate_controller(Gi, Gx, G, X, P, ei, pd, step):
    """Calculates the controller and outputs the control vector.
    
    Controller equation is:
    u(k) = -Gi*sum(i=0...k)[p(i) - pd(i)] - Gx*X(k) - sum(i=1...N)[G(i)*pd(k+i)]
    
    X and pd (state and reference) are assumed to be 0 for step < 0
    pd is assumed to retain its last value when step > N (after simulation time)
    
    Inputs:
        Gi: Integrator gain
        Gx: State controller gain
        G: Array of preview (look-ahead) gains
        X: Current state vector
        P: Current output vector
        ei: Error integrator value
        pd: Array of references
        step: Current simulation step number
        
    Outputs:
        Tuple of
            U: New control value
            ei: New error integrator value
    """
    # Calculate and integrate the error
    err = P - np.array([pd[:, step]]).T
    ei += err
    
    # Calculate the controller output
    # State feedback and integrator
    U = -np.matmul(Gx, X)
    U -= np.matmul(Gi, ei)
    
    # Preview part
    steps = pd.shape[1]
    for j, gx in enumerate(G):
        index = step + j + 1
        if index >= steps:
            index = -1      # Use last reference value after the simulation

        U -= np.matmul(gx, np.array([pd[:, index]]).T)

    return (U, ei)


# Calculates the next state using the equations
# X(k+1) = A*x(k) + B * u(k)
# P(k+1) = C*x(k)
def calculate_state(A, B, C, X, U):
    """Calculates the state transition.
    
    Equations:
    X(k+1) = A*X(k) + B*U(k)
    P(k+1) = C*X(k)
    
    Inputs:
        A, B, C: State space representation matrices of the system
        X, U: Current state and input vectors
        
    Outputs:
        Tuple of
            Xn: New state vector
            PN: New output vector
    """
    Xn = np.matmul(A, X) + np.matmul(B, U)
    Pn = np.matmul(C, X)
    
    return (Xn, Pn)




    
dt = 5.0e-3
store_interval = 10     # Store every nth control point

# Simulation parameters and cart parameters
time_end = 15   # in seconds
g = 9.8        # gravity, m/s^2
z = 0.3       # Height of center of gravity
m = 3.306   # Weight of the robot, in kg


leg_offset_x = 0.05 # X axis offset from COG to leg base point
leg_offset_y = 0.0 # Y axis offset from COG to leg base point
leg_offset_z = 0.0956 # Z axis offset from COG to leg base point
cog_height = z    # Target height for COG measured from ground
step_length = 0.05


steps = int(time_end / dt)


(A, B, C, X, U, P) = create_system(dt, z, g, m)

(Gi, Gx, G) = create_controller(A, B, C, 1.0, 1.e-6, 10, int(1.6/dt))

# Legs are 20 cm apart, step length could be 10 cm and duration 500 ms
(pxref, pyref, stance, lf_x, lf_y, lf_z, rf_x, rf_y, rf_z) = create_step_pattern(leg_offset_x, step_length, 20, 0.5, dt, time_end)
pref = np.array([pxref, pyref])

# Error sums, required for the controller
esum = np.zeros(shape=(2,1))

# open the output files for writing
try:
    cog_file = open("biped_cog_path.csv", "w")
    left_file = open("biped_left_leg_path.csv", "w")
    right_file = open("biped_right_leg_path.csv", "w")
except:
    print("Could not open files for writing")
    exit(-1)



# We use write index to offset the path distance calculation
# such that moving with constant velocity along the path follows
# one point/time unit sufficiently well
write_index = 0.0

# Main loop
for step in range(steps):
    # Calculate controller inputs
    (U, esum) = calculate_controller(Gi, Gx, G, X, P, esum, pref, step)
    

    # Calculate output, location of center of gravity (Xx, Xy)
    (X, P) = calculate_state(A, B, C, X, U)
    
    # Store the path into file
    # x,y,z,alpha,beta,gamma,relativeVelocity,BezierPointCount,interpolationFactor1,interpolationFactor2,
    #  virtualDistance,auxiliaryFlags,auxiliaryChannel1,auxiliaryChannel2,auxiliaryChannel3,auxiliaryChannel4
    # At the moment only store x,y,z, and use linear interpolation. auxiliary channel 1 contains the movement velocity
    if (step % store_interval) == 0:
        # NOTE! X and Y are inverted because the model is done like that!
        # Also Y axis is inverted in the model compared to this script
        cog_step = "{},{},{},0.0,0.0,0.0,1.0,1,0.5,0.5,{},0,0.0,0.0,0.0,0.0\n".format(X[1,0],X[0,0],z, write_index)
        
        left_x = lf_x[step] - X[0, 0] - leg_offset_x   # TODO: Add leg offsets
        left_y = lf_y[step] - X[1, 0] - leg_offset_y
        left_z = lf_z[step] - (cog_height - leg_offset_z)
        left_step = "{},{},{},0.0,0.0,0.0,1.0,1,0.5,0.5,{},0,0.0,0.0,0.0,0.0\n".format(left_y,left_x,left_z, write_index)
    
        right_x = rf_x[step] - X[0, 0] + leg_offset_x
        right_y = rf_y[step] - X[1, 0] - leg_offset_y
        right_z = rf_z[step] - (cog_height - leg_offset_z)
        right_step = "{},{},{},0.0,0.0,0.0,1.0,1,0.5,0.5,{},0,0.0,0.0,0.0,0.0\n".format(right_y,right_x,right_z, write_index)
    
        write_index += 1
    
        cog_file.write(cog_step)
        left_file.write(left_step)
        right_file.write(right_step)

cog_file.close()
left_file.close()
right_file.close()