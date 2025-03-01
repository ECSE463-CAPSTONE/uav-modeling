import numpy as np
from parameters import initialize_system_components
from simulation import Simulation, Equilibrium

# Initialize System Components
rigid_body, control_forces, tow_force, hull_force = initialize_system_components()

# Initialize Simulation
simulation = Simulation(rigid_body, control_forces, tow_force, hull_force)

# Initialize Equilibrium Solver
equilibrium_solver = Equilibrium(simulation)

# Define Initial State (12 State Variables)
initial_state = np.zeros(12)  # [x, y, z, roll, pitch, yaw, Vx, Vy, Vz, p, q, r]

# Solve for Equilibrium
equilibrium_state = equilibrium_solver.solve(initial_state)

# Run Simulation if Equilibrium is Found
if equilibrium_state is not None:
    final_state, logs, flattened_logs = simulation.run_simulation(
        initial_state=equilibrium_state,
        dt=0.01,
        num_iterations=100,
        method='euler'
    )
    print("Simulation Complete.")
