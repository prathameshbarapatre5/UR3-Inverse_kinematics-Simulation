# UR3 Robot Inverse Kinematics Simulation

**Author:** Prathmesh Barapatre  
**Course:** Robot Mechanics and Control (Project 2)

## 1. Overview
This project simulates a **UR3 6-DOF robot arm** performing task-space trajectories (Square, Circle) and interactive manual control using a custom **Newton-Raphson Inverse Kinematics (IK)** solver. The simulation is implemented entirely in Python from scratch, utilizing **NumPy** for matrix operations and **Matplotlib** for 3D visualization.

## 2. Features
- **Analytical Forward Kinematics (FK):** Standard Denavit-Hartenberg (DH) implementation for UR3.
- **Numerical Inverse Kinematics (IK):** Newton-Raphson solver with **Damped Least Squares (DLS)** for singularity robustness.
- **Trajectories:**
  - **Square:** Linear path interpolation with sharp turns.
  - **Circle:** Parametric circular path in the Y-Z plane.
- **Interactive Control:** GUI buttons for manual Cartesian control (X, Y, Z translation).
- **Visualization:** Real-time 3D stick-figure animation with end-effector frame (RGB quivers) and path tracing.

## 3. Project Structure
```
├── assets/                 # Images and video demos
├── ur3_simulation.py       # MAIN APPLICATION (Run this)
├── simple_demo.py          # Simplified circular trajectory demo (No GUI)
├── requirements.txt        # Python dependencies
├── README.md               # Project documentation
├── Project_Report.md       # Detailed theoretical report
└── .gitignore              # Git ignore file
```

## 4. Installation

### Prerequisites
- Python 3.x
- `pip` (Python package manager)

### Setup
1. Clone the repository:
   ```bash
   git clone <repository_url>
   cd <repository_folder>
   ```
2. Create a virtual environment (optional but recommended):
   ```bash
   python -m venv venv
   # On Windows:
   venv\Scripts\activate
   # On macOS/Linux:
   source venv/bin/activate
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## 5. Usage

### Run the Main Simulation
Launch the full interactive simulation with GUI controls:
```bash
python ur3_simulation.py
```
**Controls:**
- **UI Panel (Right):**
  - `Circle` / `Square`: Run automated trajectories.
  - `Home`: Reset robot to initial configuration.
  - `X+ / X-`, `Y+ / Y-`, `Z+ / Z-`: Manually jog the end-effector.

### Run Simple Demo
For a quick test of the IK solver without the full GUI:
```bash
python simple_demo.py
```

## 6. Implementation Details
The core logic is encapsulated in the `UR3Robot` class.
- **FK:** Computed via chain multiplication of DH transformation matrices.
- **IK:** Solved iteratively: $q_{k+1} = q_k + J^\dagger e$, where $J^\dagger$ is the DLS inverse of the Jacobian.
- **Damping:** A damping factor $\lambda = 0.01$ is used to handle singularities (e.g., wrist singularity).

For a deep dive into the math and methodology, please refer to [Project_Report.md](./Project_Report.md).

## 7. License
This project is for educational purposes.
