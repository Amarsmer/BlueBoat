#!/usr/bin/env python3

import time
import math
import torch
import numpy as np

def theta_s(x, y): # Angle skew, used to prevent the singularity in x=0
    return math.tanh(5.*x)*math.atan(10.*y)

class PyTorchOnlineTrainer:
    def __init__(self, robot, nn_model, in_learning_rate = 5e-4, Q=np.eye(6), R=np.eye(3)):

        self.robot = robot
        self.network = nn_model

        # Weighting matrices
        self.Q = Q
        self.R = R
        
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
        self.loss_display = np.zeros(2)

    def updateTarget(self, in_target):
        self.target = in_target

    def updateState(self, in_state):
        self.state = in_state

    def computeError(self):
        # Compute error as a column vector
        error = self.state - np.array(self.target).reshape(-1, 1)

        # Apply angle disambiguation
        error[2] = 2*np.sin(error[2]/2)

        skew = theta_s(self.state[0], self.state[1])
        # error[2] -= skew # Yaw skew

        self.skew = skew # Monitoring
        
        return error

    def computeNetworkInput(self, error):
        # Weight matrix used for input normalization
        weight_matrix = np.diag([1/10, 1/10, 1/np.pi, 1/5, 1/5, 1/np.pi])
        network_input = weight_matrix @ error
        
        return network_input.ravel()

    def computeGradient(self, delta_t, error, alpha1 = 1, alpha2 = 1000):
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
            # Get initial time for gradient computation later
            start_time = time.time()

            error = self.computeError()
            network_input = self.computeNetworkInput(error)
            
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

                # Normalize magnitude, preserve direction
                # grad_tensor = grad_tensor / (grad_tensor.norm() + 1e-6)

                # Backprop using external gradient
                self.optimizer.zero_grad()
                u_tensor.backward(gradient=grad_tensor)
                self.optimizer.step()

            # Monitoring data for debugging purposes
            self.state_train_display = self.state
            self.error_display = error
            self.input_display = network_input

            if not self.trainer_set: # Used for data recording purposes
                self.trainer_set = True

        # Stop the robot after learning
        self.robot.move([0,0,0,0],
                      [0 for i in range(1,5)])
        # self.running = False