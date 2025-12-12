#UR3 Inverse Kinematics Simulation Code
#Author: Prathmesh Barapatre

#Importing Libraries
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

#UR3 Robot Class Parameters
class UR3Robot:
    def __init__(self):
        #Standard DH Parameters for UR3
        #[theta_offset, d, a, alpha]
        self.dh_params = [
            [0,         0.1519,   0,        np.pi/2], #Joint 1
            [0,         0,       -0.24365,  0      ], #Joint 2
            [0,         0,       -0.21325,  0      ], #Joint 3
            [0,         0.11235,  0,        np.pi/2], #Joint 4
            [0,         0.08535,  0,       -np.pi/2], #Joint 5
            [0,         0.0819,   0,        0      ]  #Joint 6
        ]
        self.num_joints = 6
        self.q = np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])

    def dh_transform(self, theta, d, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st*ca,  st*sa,  a*ct],
            [st,  ct*ca, -ct*sa,  a*st],
            [0,   sa,     ca,     d   ],
            [0,   0,      0,      1   ]
        ])

    def forward_kinematics(self, q):
        T = np.eye(4)
        positions = [T[:3, 3]]
        z_vectors = [T[:3, 2]] 
        origins = [T[:3, 3]]
        
        for i in range(self.num_joints):
            theta_offset, d, a, alpha = self.dh_params[i]
            theta = q[i] + theta_offset
            T_i = self.dh_transform(theta, d, a, alpha)
            T = T @ T_i
            positions.append(T[:3, 3])
            if i < self.num_joints - 1:
                z_vectors.append(T[:3, 2])
                origins.append(T[:3, 3])
                
        return T, np.array(positions), z_vectors, origins

    def compute_jacobian(self, q, p_eff):
        _, _, z_vectors, origins = self.forward_kinematics(q)
        J = np.zeros((6, self.num_joints))
        for i in range(self.num_joints):
            z_i = z_vectors[i]
            p_i = origins[i]
            J[:3, i] = np.cross(z_i, (p_eff - p_i))
            J[3:, i] = z_i
        return J

    def inverse_kinematics_newton(self, target_T, max_iter=50, tol=1e-3):
        q_current = self.q.copy()
        target_pos = target_T[:3, 3]
        target_rot = target_T[:3, :3]
        
        for _ in range(max_iter):
            T_curr, _, _, _ = self.forward_kinematics(q_current)
            curr_pos = T_curr[:3, 3]
            curr_rot = T_curr[:3, :3]
            
            err_pos = target_pos - curr_pos
            err_rot = np.zeros(3)
            #Vector cross product orientation error
            for i in range(3):
                err_rot += np.cross(curr_rot[:, i], target_rot[:, i])
            err_rot *= 0.5
            
            error_vector = np.concatenate((err_pos, err_rot))
            
            if np.linalg.norm(error_vector) < tol:
                return q_current, True #Converged
                
            J = self.compute_jacobian(q_current, curr_pos)
            #Damped Least Squares for stability near singularities
            lambda_val = 0.01
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lambda_val**2 * np.eye(6))
            dq = J_pinv @ error_vector
            q_current += dq
            
        return q_current, False 


def run_simulation():
    #Setting up Plot
    plt.ion() #for Interactive Mode
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("UR3 Inverse Kinematics Trajectory (Red=X, Green=Y, Blue=Z)")
    
    #Setting fixed axis limits
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])
    ax.set_zlim([0, 0.8])
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')

    #Initializing Visuals
    robot_line, = ax.plot([], [], [], 'o-', lw=3, color='gray', markersize=6)
    path_trace, = ax.plot([], [], [], '--', lw=1, color='orange')
    
    #Initializing Quivers for End Effector Frame
    qx = ax.quiver(0,0,0,0,0,0, color='r')
    qy = ax.quiver(0,0,0,0,0,0, color='g')
    qz = ax.quiver(0,0,0,0,0,0, color='b')

    #2. Generating Circular Path
    robot = UR3Robot()
    trajectory = []
    center = np.array([0.25, 0.1, 0.3])
    radius = 0.1
    steps = 50
    
    for t in np.linspace(0, 2*np.pi, steps):
        y = center[1] + radius * np.cos(t)
        z = center[2] + radius * np.sin(t)
        x = center[0]
        T = np.eye(4)
        T[:3, 3] = [x, y, z]
        trajectory.append(T)

    path_x, path_y, path_z = [], [], []

    print("Starting Animation loop...")

    #Adding Animation Loop
    for i, target_T in enumerate(trajectory):
        #Solving Inverse Kinematics
        q_sol, success = robot.inverse_kinematics_newton(target_T)
        robot.q = q_sol
        
        if not success:
            print(f"Frame {i}: IK did not converge.")

        #Getting FK for Plotting
        T_ee, joints, _, _ = robot.forward_kinematics(q_sol)
        
        #Updating Robot Body
        robot_line.set_data(joints[:, 0], joints[:, 1])
        robot_line.set_3d_properties(joints[:, 2])
        
        #Updating Trace
        p_ee = T_ee[:3, 3]
        path_x.append(p_ee[0])
        path_y.append(p_ee[1])
        path_z.append(p_ee[2])
        path_trace.set_data(path_x, path_y)
        path_trace.set_3d_properties(path_z)
        
        #Updating End Effector Frame
        qx.remove(); qy.remove(); qz.remove()
        R = T_ee[:3, :3]
        len_ = 0.1
        qx = ax.quiver(p_ee[0], p_ee[1], p_ee[2], R[0,0], R[1,0], R[2,0], color='r', length=len_)
        qy = ax.quiver(p_ee[0], p_ee[1], p_ee[2], R[0,1], R[1,1], R[2,1], color='g', length=len_)
        qz = ax.quiver(p_ee[0], p_ee[1], p_ee[2], R[0,2], R[1,2], R[2,2], color='b', length=len_)
        
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.05) #Pause for GUI to update

    print("Done. Keep window open.")
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    run_simulation()