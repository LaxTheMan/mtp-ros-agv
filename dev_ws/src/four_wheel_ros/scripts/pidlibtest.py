import time
import numpy as np
import matplotlib.pyplot as plt
from simple_pid import PID

# Simulated system to control (e.g., temperature)
class SimulatedSystem:
    def __init__(self, initial_temp=25.0):
        self.temperature = initial_temp

    def update(self, power):
        # Simulate temperature change based on power input
        dt = 1  # Time step (seconds)
        ambient_temp = 20.0
        tau = 10.0  # Time constant

        # Calculate temperature change based on power and ambient temperature
        delta_temp = dt * ((power - self.temperature - ambient_temp) / tau)
        self.temperature += delta_temp

        # Limit temperature to a realistic range
        self.temperature = np.clip(self.temperature, 0, 100)

        return self.temperature

# Initialize PID controller
pid = PID(1.0, 0.1, 0.05, setpoint=30.0)

# Initialize simulated system
system = SimulatedSystem()

# Lists to store data for plotting
times = []
temperatures = []
setpoints = []

# Simulation parameters
total_time = 300  # Total simulation time (seconds)
start_time = time.time()

# Simulation loop
while time.time() - start_time < total_time:
    # Get current temperature
    current_temp = system.temperature

    # Calculate PID output (power)
    power = pid(current_temp)

    # Update the simulated system
    new_temp = system.update(power)

    # Record data for plotting
    times.append(time.time() - start_time)
    temperatures.append(current_temp)
    setpoints.append(pid.setpoint)

    # Print current temperature and PID output
    print(f"Time: {times[-1]:.1f} s, Temperature: {current_temp:.2f} °C, Power: {power:.2f}")

    # Sleep for a short time (simulate real-time)
    time.sleep(0.1)

# Plot results
plt.figure()
plt.plot(times, temperatures, label='Temperature')
plt.plot(times, setpoints, label='Setpoint', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Temperature (°C)')
plt.title('PID Control of Temperature')
plt.legend()
plt.grid(True)
plt.show()
