import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Define PID Controller class with resistance
class PIDControllerWithResistance:
    def __init__(self, Kp, Ki, set_point=0, resistance_factor=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.set_point = set_point
        self.integral = 0
        self.resistance_factor = resistance_factor # Resistance to throttle (e.g., air resistance, friction)

    def update(self, current_value, dt):
        # Apply the same PID control logic but factor in resistance
        error = self.set_point - current_value
        self.integral += error * dt
        output = self.Kp * error + self.Ki * self.integral
        return output - self.resistance_factor * current_value # Reduce output by a resistance factor
    
# Simulation parameters
dt = 0.1 # Time step
time = np.arange(0, 50, dt) # Simulation time

# Initialize the PID controller with disturbance (resistance)
pid_with_resistance = PIDControllerWithResistance(Kp=1.0, Ki=0.05, set_point=50, resistance_factor=0.01) # Resistance factor lowered so graph overshoots

# Initial conditions
speed = 0
throttle_with_resistance = []
speed_record_with_resistance = []

# Simulate the system with resistance
for t in time:
    control = pid_with_resistance.update(speed, dt)
    speed += control * dt  # Speed is affected by throttle control and resistance
    throttle_with_resistance.append(control)
    speed_record_with_resistance.append(speed)

# Plot setup
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.1, bottom=0.3)

l, = plt.plot(time, speed_record_with_resistance, label="Velocity Output (With Resistance)")
plt.axhline(pid_with_resistance.set_point, color='r', linestyle='--', label='Set Point')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.title('PID Cruise Control with Resistance')
plt.legend()
plt.show()