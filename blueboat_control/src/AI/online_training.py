#!/usr/bin/env python3

import time
import math
import torch
import numpy as np
from scipy.spatial.transform import Rotation as R
import custom_functions as cf

def theta_s(x, y): # Angle skew, used to prevent the singularity in x=0
    return math.tanh(5.*x)*math.atan(10.*y)

def dynamicSkew(x,y,psi):
    delta_x = x >= 0
    delta_y = y >= 0
    delta_psi = psi >= 0
    abs_psi = abs(psi) >= np.pi/2

    atan2 = abs_psi or delta_x or (delta_y and delta_psi)

    if atan2:
        return np.arctan2(y,x)
    else:
        return np.arctan(y/x)

def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def transformationMatrix(in_coordinates):
    # Inverse transformation matrix that converts world coordinates to robot coordinates
    x_w,y_w,theta = in_coordinates[0:3].ravel()
    c = np.cos
    s = np.sin

    tf_mat = np.array([[ c(theta), s(theta), -c(theta)*x_w - s(theta)*y_w], 
                        [-s(theta), c(theta),  s(theta)*x_w - c(theta)*y_w],
                        [     0   ,     0   ,               1             ]])
                        
    return -tf_mat # The minus sign is because this method assumes the error is 'target - state' which is opposite here

class PyTorchOnlineTrainer:
    def __init__(self, robot, nn_model, in_learning_rate = 5e-4, Q=np.eye(6), R=np.eye(3), use_quat = False):

        self.robot = robot
        self.network = nn_model

        # Weighting matrices
        self.Q = Q
        self.R = R
        
        # Use quaternion for error comutation
        self.use_quaternions = use_quat

        # Training state
        self.running = False
        self.training = True

        self.learning_rate = in_learning_rate

        # Set up optimizer (partially replaces backpropagation)
        self.optimizer = torch.optim.Adam(self.network.parameters(), 
                                          lr=self.learning_rate)

        # Variables init
        self.state = None
        self.error = None
        self.previous_target = None
        self.previous_state = None
        self.target = None
        self.u = np.zeros(2)
        self.loss = None

        ## Modeling
        # Compute B matrix (constant)
        self.radius = 0.15 # radius of the robot

        # NED not yet implemented, so the last row is reversed
        self.B = np.array([[1.        ,1.],
                           [0.        ,0.],
                           [self.radius,-self.radius]]) 

        # Coefficients for gradient computation (reduces unnecessary temp variable attribution in loop since it's constant)
        self.m = self.robot.mass

        self.Xudot = self.robot.added_masses[0]
        self.Yvdot = self.robot.added_masses[1]
        self.Nrdot = self.robot.added_masses[5]

        self.Xu = self.robot.viscous_drag[0]
        self.Yv = self.robot.viscous_drag[1]
        self.Nr = self.robot.viscous_drag[5]

        self.Iz = self.robot.inertia[-1]
        
        self.grad_xdk = np.array([[0.                              , 0.                            ],
                                  [0.                              , 0.                            ],
                                  [0.                              , 0.                            ],
                                  [1/(self.m - self.Xudot)         , 1/(self.m - self.Xudot)       ],
                                  [0.                              , 0.                            ],
                                  [self.radius/(self.Iz - self.Nrdot)   , -self.radius/(self.Iz - self.Nrdot)]])

        self.trainer_set = False # Make sure inputs have been computed before recording data

        # Monitoring variables, meant to be displayed in terminal
        self.gradient_display = None
        self.input_display = None 
        self.error_display = None 
        self.delta_t_display = None
        self.skew = None
        self.state_display = None
        self.target_display = None
        self.loss_display = np.zeros(2)
        self.robot_frame_display = np.zeros(3)

    def updateTarget(self, in_target):
        temp_target = in_target
        # if self.previous_target is not None:
        #     temp_target[2] = np.unwrap([self.previous_target[2],temp_target[2]])[-1]
        if self.use_quaternions:
            self.target_display = cf.planeFromQuaternion(in_target)
        else: 
            self.target_display = in_target

        self.target = temp_target

    def updateState(self, in_state):
        temp_state = in_state
        # if self.previous_state is not None:
        #     temp_state[2] = np.unwrap([self.previous_state[2],temp_state[2]])[-1]
        if self.use_quaternions:
            self.state_display = cf.planeFromQuaternion(in_state.ravel())
        else: 
            self.state_display = in_state

        self.state = temp_state

    def robotCoordinates(self, in_error):
        x_r,y_r,theta_r = self.state[0:3]
        x_t,y_t,theta_t = self.target[0:3]

        delta_x = -in_error[0]
        delta_y = -in_error[1]

        theta = -in_error[2]

        x =  np.cos(theta_r)*delta_x + np.sin(theta_r)*delta_y
        y = -np.sin(theta_r)*delta_x + np.cos(theta_r)*delta_y

        return x,y,theta

    def computeError(self):
        # Compute error as a column vector
        if self.use_quaternions:
            quat_error = cf.quaternion_error(self.state[3:7], self.target[3:7])
            rotation = R.from_quat(quat_error.ravel())
            roll, pitch, yaw = rotation.as_euler('xyz')
            error = np.array([self.state[0] - self.target[0],
                              self.state[1] - self.target[1],
                              yaw,
                              self.state[7] - self.target[7],
                              self.state[8] - self.target[8],
                              self.state[9] - self.target[9]]).reshape((6,1))
            
        else:
            error = self.state - np.array(self.target).reshape(-1, 1)
        
        self.error_display = error

        robot_coordinates = self.robotCoordinates(error)
        self.robot_frame_display = robot_coordinates

        x_r,y_r,psi_r = robot_coordinates

        # psi_r = wrap(psi_r)
        # psi_skew = dynamicSkew(x_r, y_r, psi_r)
        psi_skew = -np.arctan2(y_r,x_r)
        # psi_skew = theta_s(x_r,y_r)
        self.skew = psi_skew # Monitoring

        d = np.sqrt(x_r**2+y_r**2)
        d_w = 1

        # error[2] = self.state[2] - (np.exp(-(d*d_w)**2)*self.target[2] + (1 - np.exp(-(d*d_w)**2))*psi_skew)
        # error[2] = wrap(error[2])
        # error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))
        # error[2] = 2*np.sin(error[2]/2)

        return error

    def computeNetworkInput(self, error):
        x,y,psi,u,v,r = error.squeeze()

        d = np.sqrt(x**2+y**2)

        # Weight matrix used for input normalization
        weight_matrix = np.diag([1/100, 1/np.pi, 1/5, 1/5, 1/np.pi])

        network_input = weight_matrix @ np.array([d,psi,u,v,r]).reshape((5,1))
        
        return network_input.ravel()

    def computeGradient(self, delta_t, error, alpha1 = 0, alpha2 = 1000):
        if self.use_quaternions:
            x,y,psi,u,v,r = cf.planeFromQuaternion(self.state).ravel()
        else:
            x,y,psi,u,v,r = self.state.ravel()

        gradxJ = 2 * (self.Q @ error)
        graduJ = 2 * (self.R @ self.u)

        fod_grad = self.grad_xdk # first order derivative gradient

        # second order derivative gradient, done in multiple steps to increase readability and ease of debugging
        cos = np.cos
        sin = np.sin

        fracmXu = self.m + self.Xudot
        fracmYv = self.m + self.Yvdot
        fracIzNr = self.Iz + self.Nrdot

        grad3_A = self.radius*v*(self.m - self.Yvdot)/fracIzNr # each "A" element is dependent on the radius and may need a change of sign when NED is implemented
        grad3_B = self.Xu/fracmXu

        grad4_A = u*r/fracIzNr
        grad4_B = r/fracmXu

        grad5_A = (self.radius*self.Nr)/fracIzNr
        grad5_B = v*(self.Yvdot-self.Xudot)/fracmXu

        sod_grad = np.array([[cos(psi)/fracmXu                                 , cos(psi)/fracmXu                                  ],
                            [sin(psi)/fracmXu                                  , sin(psi)/fracmXu                                  ],
                            [self.radius/fracIzNr                              , -self.radius/fracIzNr                             ],
                            [(-grad3_A + grad3_B)/fracmXu                      , (grad3_A + grad3_B)/fracmXu                       ],
                            [(-grad4_A + grad4_B)*(self.Xudot - self.m)/fracmYv, (grad4_A + grad4_B)*(self.Xudot - self.m)/fracmYv ],
                            [(-grad5_A + grad5_B)/fracIzNr                     , (grad5_A + grad5_B)/fracIzNr                      ]]) 

        # cost function gradient
        time_gradient = alpha1 * delta_t * fod_grad + alpha2 * 0.5*delta_t**2 * sod_grad

        grad = (time_gradient.transpose() @ gradxJ) + graduJ
        grad = grad.squeeze(-1) # Removes the dimensions of size 1

        return grad

    def train(self, target):
        # Training loop
        while self.running:
            if self.state is not None and self.target is not None:
                # Get initial time for gradient computation later
                start_time = time.time()

                error = self.computeError()
                network_input = self.computeNetworkInput(error.ravel())
                
                # Prepare input
                input_tensor = torch.tensor(network_input, dtype=torch.float32, requires_grad=self.training)

                # Forward pass
                if self.training:
                    u_tensor = self.network(input_tensor)
                else:
                    with torch.no_grad():
                        u_tensor = self.network(input_tensor)

                # Scale output
                input_coefficient = 40.0
                u_tensor = input_coefficient * u_tensor

                # Apply control input (convert ONLY for the robot)
                self.u = u_tensor.detach().cpu().numpy().reshape(-1, 1)
                self.robot.move([self.u[0], self.u[1], 0, 0],
                                [0 for i in range(1, 5)])


                # Compute loss, both for monitoring and later for backpropagation
                crit_x = error.transpose() @ self.Q @ error
                crit_u = self.u.transpose() @ self.R @ self.u
                self.loss = crit_x + crit_u

                self.loss_display = np.array([crit_x, crit_u])

                ### Training step
                if self.training:
                    delta_t = (time.time() - start_time)
                    self.delta_t_display = delta_t

                    # Manual gradient computation
                    grad = self.computeGradient(delta_t, error)
                    self.gradient_display = grad
                    
                    # Convert to tensor grad
                    grad_tensor = torch.tensor(grad, dtype=torch.float32)

                    # Backprop using external gradient
                    self.optimizer.zero_grad()
                    u_tensor.backward(gradient=grad_tensor)
                    self.optimizer.step()

                # Monitoring data for debugging purposes
                self.error_display = error
                self.input_display = network_input

                self.previous_target = self.target
                self.previous_state = self.state

                if not self.trainer_set: # Used for data recording purposes
                    self.trainer_set = True

            # Stop the robot after learning
            self.robot.move([0,0,0,0],
                        [0 for i in range(1,5)])
            # self.running = False