import math
import numpy as np
# import time

class EKF():
    def __init__(self) -> None:
        self.Q = (np.diag([
            0.1,  # variance of location on x-axis
            0.1,  # variance of location on y-axis
            0.2,  # variance of yaw angle
            0.5,  # variance of velocity
            ]) ** 2)  # predict state covariance
        self.jH = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
        
    def motion_model(self, x, u, dt):
        F = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 0]])
        B = np.array(
            [
                [dt * math.cos(x[2, 0]), 0],
                [dt * math.sin(x[2, 0]), 0],
                [0.0, dt],
                [1.0, 0.0],
            ]
        )
        x = F @ x + B @ u
        return x
    
    def observation_model(self, x):
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
        z = H @ x
        return z
    
    def jacob_f(self, x, u, dt):
        """
        Jacobian of Motion Model

        Kinematic Motion Model
        x_{t+1} = x_t+v*dt*cos(yaw)
        y_{t+1} = y_t+v*dt*sin(yaw)
        yaw_{t+1} = yaw_t+omega*dt
        v_{t+1} = v{t}

        Jacobian
        dx/dyaw = -v*dt*sin(yaw)
        dx/dv = dt*cos(yaw)
        dy/dyaw = v*dt*cos(yaw)
        dy/dv = dt*sin(yaw)
        """
        yaw = x[2, 0]
        v = u[0, 0]
        jF = np.array(
            [
                [1.0, 0.0, -dt * v * math.sin(yaw), dt * math.cos(yaw)],
                [0.0, 1.0, dt * v * math.cos(yaw), dt * math.sin(yaw)],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        return jF
    
    def predict(self, xEst, PEst, u, dt):
        xPred = self.motion_model(xEst, u, dt)
        jF = self.jacob_f(xEst, u, dt)
        PPred = jF @ PEst @ jF.T + self.Q
        return xPred, PPred

    def angle_sum(self, angle0, angle1):
        out = np.arctan2(np.sin(angle0 + angle1), np.cos(angle0 + angle1))
        if out < 0:
            out += 2 * np.pi
        if np.abs(out - 2 * np.pi) < 0.0001 * 2 * np.pi:
            out = 0
        return out
    
    def update(self, xPred, PPred, z, R):
        #  Update
        zPred = self.observation_model(xPred)
        y = z - zPred
        y[2] = self.angle_sum(z[2], -zPred[2])
        if y[2] > 5:
            y[2] -= 2 * np.pi
        elif y[2] < -5:
            y[2] += 2 * np.pi
            
        S = self.jH @ PPred @ self.jH.T + R
        
        K = PPred @ self.jH.T @ np.linalg.inv(S)
        
        Ky = K @ y
        xEst = xPred + K @ y
        xEst[2] = self.angle_sum(xPred[2], Ky[2])
        PEst = (np.eye(len(xEst)) - K @ self.jH) @ PPred
        return xEst, PEst, K