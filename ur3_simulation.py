# UR3 Inverse Kinematics Simulation Code
# Author: Prathmesh Barapatre
# Project 2: Robot Mechanics and Control

#Description: This script simulates a UR3 robot performing task-space trajectories (Square, Circle)and manual control using a Newton-Raphson Inverse Kinematics solver from scratch.


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Button
import time

#Creating Robot Class
class UR3Robot:
    def __init__(self):
        #Standard DH Parameters for UR3 Robot
        #Derived from UR3 specifications.
        #Format: [theta_offset, d, a, alpha]
        #Units: meters (d, a), radians (alpha, theta)
        self.dh_params = [
            
            [0,         0.1519,   0,        np.pi/2], #Joint 1: Base to Shoulder
            [0,         0,       -0.24365,  0      ], #Joint 2: Shoulder to Elbow
            [0,         0,       -0.21325,  0      ], #Joint 3: Elbow to Wrist 1
            [0,         0.11235,  0,        np.pi/2], #Joint 4: Wrist 1 to Wrist 2
            [0,         0.08535,  0,       -np.pi/2], #Joint 5: Wrist 2 to Wrist 3
            [0,         0.0819,   0,        0      ]  #Joint 6: Wrist 3 to End-Effector
        ]
        self.num_joints = 6
        #Initialising 'Home' Configuration (Upright)
        self.q = np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])

    def dh_transform(self, theta, d, a, alpha):
        """
        Calculates the Homogeneous Transformation Matrix for a single link 
        using Denavit-Hartenberg parameters.
        """
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        
        #Standard DH Matrix
        return np.array([
            [ct, -st*ca,  st*sa,  a*ct],
            [st,  ct*ca, -ct*sa,  a*st],
            [0,   sa,     ca,     d   ],
            [0,   0,      0,      1   ]
        ])

    def forward_kinematics(self, q):
        """
        Computes the Forward Kinematics to find the end-effector position and orientation.
        Returns:
            T: Final homogeneous transform (4x4)
            positions: List of joint origins for plotting
            z_vectors: List of Z-axes for Jacobian calculation
            origins: List of frame origins for Jacobian calculation
        """
        T = np.eye(4)
        positions = [T[:3, 3]]
        z_vectors = [T[:3, 2]] 
        origins = [T[:3, 3]]
        
        for i in range(self.num_joints):
            theta_offset, d, a, alpha = self.dh_params[i]
            theta = q[i] + theta_offset
            
            T_i = self.dh_transform(theta, d, a, alpha)
            T = T @ T_i #Chain transformations: T_0_i = T_0_(i-1) @ T_(i-1)_i
            
            positions.append(T[:3, 3])
            
            #Store Z-vectors and Origins for Geometric Jacobian
            if i < self.num_joints - 1:
                z_vectors.append(T[:3, 2])
                origins.append(T[:3, 3])
                
        return T, np.array(positions), z_vectors, origins

    def compute_jacobian(self, q, p_eff):
        """
        Computes the Geometric Jacobian Matrix J(q).
        J relates joint velocities to end-effector spatial velocity (v, w).
        dim(J) = 6 x 6
        """
        _, _, z_vectors, origins = self.forward_kinematics(q)
        J = np.zeros((6, self.num_joints))
        
        for i in range(self.num_joints):
            z_i = z_vectors[i]      #Axis of rotation for joint i
            p_i = origins[i]        #Position of joint i
            
            #Linear Velocity component (v = w x r)
            J[:3, i] = np.cross(z_i, (p_eff - p_i))
            
            #Angular Velocity component (w = z)
            J[3:, i] = z_i
            
        return J

    def inverse_kinematics_newton(self, target_T, max_iter=50, tol=1e-3):
        """
        Solves Inverse Kinematics using the Newton-Raphson numerical method.
        Iteratively updates q to minimize error between current and target pose.
        """
        q_current = self.q.copy()
        target_pos = target_T[:3, 3]
        target_rot = target_T[:3, :3]
        
        for _ in range(max_iter):
            #Forward Kinematics
            T_curr, _, _, _ = self.forward_kinematics(q_current)
            curr_pos = T_curr[:3, 3]
            curr_rot = T_curr[:3, :3]
            
            #Computing Error
            err_pos = target_pos - curr_pos
            
            #Orientation Error using Vector Cross Product method
            #Approximation for small angles
            err_rot = np.zeros(3)
            for i in range(3):
                err_rot += np.cross(curr_rot[:, i], target_rot[:, i])
            err_rot *= 0.5
            
            error_vector = np.concatenate((err_pos, err_rot))
            
            #Checking Convergence
            if np.linalg.norm(error_vector) < tol:
                return q_current, True #Converged
                
            #Computing Jacobian
            J = self.compute_jacobian(q_current, curr_pos)
            
            #Solving for dq using Damped Least Squares (DLS)
            #DLS is more robust near singularities than pure pseudo-inverse
            lambda_val = 0.01 #Damping factor
            #J_dls = J^T * (J * J^T + lambda^2 * I)^-1
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lambda_val**2 * np.eye(6))
            dq = J_pinv @ error_vector
            
            #Update Joints
            q_current += dq
            
        return q_current, False 


def rotation_matrix_to_rpy(R):
    """
    Converts a Rotation Matrix to Roll-Pitch-Yaw angles (XYZ convention).
    Returns angles in degrees for easier visualization.
    """
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.degrees([x, y, z])

#Main Simulation
def run_simulation():
    #Setting up Figure and 3D Axis
    plt.ion() 
    fig = plt.figure(figsize=(16, 9)) #Wide layout
    ax = fig.add_axes([0.05, 0.05, 0.65, 0.9], projection='3d') #Plot on Left
    
    # Title
    ax.set_title("UR3 Inverse Kinematics Trajectory (Red=X, Green=Y, Blue=Z)", fontsize=14)
    
    # Standard Plot Settings
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])
    ax.set_zlim([0, 0.8])
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')

    # Visual Elements initialize
    robot_line, = ax.plot([], [], [], 'o-', lw=4, color='gray', markersize=8, label='Robot Link')
    path_trace, = ax.plot([], [], [], '--', lw=1, color='orange', label='Path Trace')
    
    #End-Effector Frame Quivers
    qx = ax.quiver(0,0,0,0,0,0, color='r', linewidth=2)
    qy = ax.quiver(0,0,0,0,0,0, color='g', linewidth=2)
    qz = ax.quiver(0,0,0,0,0,0, color='b', linewidth=2)

    #Initializing Robot
    robot = UR3Robot()
    path_x, path_y, path_z = [], [], []
    current_T_ee = np.eye(4)

    #Info Panel Text
    info_text = fig.text(0.75, 0.80, "Robot Status", fontsize=11, family='monospace', 
                         verticalalignment='top', bbox=dict(facecolor='white', alpha=0.8))

    #Plot Update Function
    def update_plot(q, clear_trace=False):
        nonlocal qx, qy, qz, path_x, path_y, path_z
        
        if clear_trace:
            path_x, path_y, path_z = [], [], [] #Reset trace

        #Calculate FK
        T_ee, joints, _, _ = robot.forward_kinematics(q)
        
        #Update Robot Links
        robot_line.set_data(joints[:, 0], joints[:, 1])
        robot_line.set_3d_properties(joints[:, 2])
        
        #Update Trace
        p_ee = T_ee[:3, 3]
        path_x.append(p_ee[0])
        path_y.append(p_ee[1])
        path_z.append(p_ee[2])
        path_trace.set_data(path_x, path_y)
        path_trace.set_3d_properties(path_z)
        
        #Update End Effector Frame
        qx.remove(); qy.remove(); qz.remove() #Clear old quivers
        R = T_ee[:3, :3]
        L = 0.1 #Axis length
        qx = ax.quiver(p_ee[0], p_ee[1], p_ee[2], R[0,0], R[1,0], R[2,0], color='r', length=L)
        qy = ax.quiver(p_ee[0], p_ee[1], p_ee[2], R[0,1], R[1,1], R[2,1], color='g', length=L)
        qz = ax.quiver(p_ee[0], p_ee[1], p_ee[2], R[0,2], R[1,2], R[2,2], color='b', length=L)
        
        #Update Info Panel
        rpy = rotation_matrix_to_rpy(R)
        txt = f"End-Effector Pose:\n" + "-"*20 + "\n"
        txt += f"Pos [X, Y, Z] (m):\n[{p_ee[0]:.3f}, {p_ee[1]:.3f}, {p_ee[2]:.3f}]\n\n"
        txt += f"Rot [R, P, Y] (deg):\n[{rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}]\n"
        info_text.set_text(txt)

        fig.canvas.draw()
        fig.canvas.flush_events()
        return T_ee

    #Initial Draw
    current_T_ee = update_plot(robot.q)

    def run_trajectory(traj_points, label):
        nonlocal current_T_ee
        print(f"Executing {label} Trajectory...")
        
        #Move to Start Configuration first
        if len(traj_points) > 0:
            q_start, success = robot.inverse_kinematics_newton(traj_points[0])
            if success:
                robot.q = q_start
                update_plot(q_start, clear_trace=True) #Clear old trace
                plt.pause(0.2)
        
        #Execute Path
        for target_T in traj_points[1:]:
            q_sol, success = robot.inverse_kinematics_newton(target_T)
            #Update robot state
            robot.q = q_sol 
            if success:
                update_plot(q_sol, clear_trace=False)
            else:
                print("IK Singularity/Limit reached.")
            plt.pause(0.01) # Animation Speed
            
        print(f"{label} Complete.")
        current_T_ee, _, _, _ = robot.forward_kinematics(robot.q)

    def btn_circle_cb(event):
        #Parametric Circle in Y-Z plane
        center = np.array([0.25, 0.1, 0.3])
        radius = 0.10
        steps = 60
        trajectory = []
        for t in np.linspace(0, 2*np.pi, steps):
            #x fixed, y and z vary
            x = center[0]
            y = center[1] + radius * np.cos(t)
            z = center[2] + radius * np.sin(t)
            T = np.eye(4)
            T[:3, 3] = [x, y, z] #Orientation stays Identity
            trajectory.append(T)
        run_trajectory(trajectory, "Circle")

    def btn_square_cb(event):
        #Square Path
        center = np.array([0.25, 0.1, 0.3])
        side = 0.15
        x = center[0]
        #Define Corners
        c1 = [x, center[1]-side/2, center[2]-side/2]
        c2 = [x, center[1]+side/2, center[2]-side/2]
        c3 = [x, center[1]+side/2, center[2]+side/2]
        c4 = [x, center[1]-side/2, center[2]+side/2]
        corners = [c1, c2, c3, c4, c1] #Loop back to start
        
        trajectory = []
        steps_per_side = 20
        for k in range(4):
            start_p = np.array(corners[k])
            end_p = np.array(corners[k+1])
            for t in np.linspace(0, 1, steps_per_side):
                p = start_p + (end_p - start_p) * t
                T = np.eye(4)
                T[:3, 3] = p
                trajectory.append(T)
        run_trajectory(trajectory, "Square")

    def move_callback(direction_vec):
        nonlocal current_T_ee
        #Manual Control Step
        target_T = current_T_ee.copy()
        delta = 0.02 #2cm step
        target_T[:3, 3] += direction_vec * delta
        
        q_sol, success = robot.inverse_kinematics_newton(target_T)
        
        if success:
            robot.q = q_sol
            #Keeping trace enabled to draw with keys
            current_T_ee = update_plot(q_sol, clear_trace=False) 
        else:
            print("Cannot move: IK Solution not found.")
            
    def btn_home_cb(event):
        nonlocal current_T_ee
        robot.q = np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]) # Home
        current_T_ee = update_plot(robot.q, clear_trace=True)
        print("Robot homed.")

    #UI Layout
    #Coordinates: 
    x_base = 0.70
    btn_w = 0.08
    btn_h = 0.05
    gap = 0.01
    
    #Manual Control Section 
    y_manual = 0.45
    fig.text(x_base, y_manual + 0.08, "Manual Control", fontweight='bold', fontsize=12)
    
    #Row 1: X+, Y+, Z+
    y_row1 = y_manual
    btn_xp = Button(plt.axes([x_base, y_row1, btn_w, btn_h]), 'X+')
    btn_yp = Button(plt.axes([x_base + btn_w + gap, y_row1, btn_w, btn_h]), 'Y+')
    btn_zp = Button(plt.axes([x_base + 2*(btn_w + gap), y_row1, btn_w, btn_h]), 'Z+')
    
    #Row 2: X-, Y-, Z-
    y_row2 = y_manual - btn_h - gap
    btn_xn = Button(plt.axes([x_base, y_row2, btn_w, btn_h]), 'X-')
    btn_yn = Button(plt.axes([x_base + btn_w + gap, y_row2, btn_w, btn_h]), 'Y-')
    btn_zn = Button(plt.axes([x_base + 2*(btn_w + gap), y_row2, btn_w, btn_h]), 'Z-')
    
    #Trajectory Section 
    y_traj = 0.15
    fig.text(x_base, y_traj + 0.08, "Trajectory Tasks", fontweight='bold', fontsize=12)
    
    #Horizontal Layout: Home | Circle | Square
    t_btn_w = 0.08
    
    btn_home = Button(plt.axes([x_base, y_traj, t_btn_w, btn_h]), 'Home')
    btn_circ = Button(plt.axes([x_base + t_btn_w + gap, y_traj, t_btn_w, btn_h]), 'Circle')
    btn_sq   = Button(plt.axes([x_base + 2*(t_btn_w + gap), y_traj, t_btn_w, btn_h]), 'Square')

    #Assigning Callbacks
    btn_xp.on_clicked(lambda x: move_callback(np.array([1, 0, 0])))
    btn_xn.on_clicked(lambda x: move_callback(np.array([-1, 0, 0])))
    btn_yp.on_clicked(lambda x: move_callback(np.array([0, 1, 0])))
    btn_yn.on_clicked(lambda x: move_callback(np.array([0, -1, 0])))
    btn_zp.on_clicked(lambda x: move_callback(np.array([0, 0, 1])))
    btn_zn.on_clicked(lambda x: move_callback(np.array([0, 0, -1])))
    
    btn_sq.on_clicked(btn_square_cb)
    btn_circ.on_clicked(btn_circle_cb)
    btn_home.on_clicked(btn_home_cb)

    fig._buttons = [btn_xp, btn_xn, btn_yp, btn_yn, btn_zp, btn_zn, btn_sq, btn_circ, btn_home]

    print("UR3 Simulation Ready.")
    print("Controls: Use arrows on screen to move. 'Z' controls height.")
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    run_simulation()