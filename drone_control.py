import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.integral = 0
        self.prev_error = 0
        self.last_time = time.time()

    def compute(self, setpoint, current_value):
        now = time.time()
        dt = max(now - self.last_time, 0.01)
        self.last_time = now
        
        error = setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return output

class DroneController:
    def __init__(self):
        self.alt_pid = PIDController(kp=1.5, ki=0.1, kd=0.5)
        self.target_altitude = 5.0
        
        self.state = {
            'position': [0.0, 0.0, 0.0], # x, y (alt), z
            'velocity': [0.0, 0.0, 0.0],
            'heading': 0.0,
            'speed': 0.0,
            'status': 'ARMED',
            'last_update': time.time(),
            'path': [] # Traced path log
        }

    def update_physics(self, dt):
        """Called repeatedly to update simulated physics"""
        
        # Apply PID control to vertical velocity to reach target_altitude
        # The PID output acts as vertical thrust adjustment
        alt_thrust = self.alt_pid.compute(self.target_altitude, self.state['position'][1])
        
        # Add a tiny bit of gravity/instability
        gravity = -0.5 
        
        self.state['velocity'][1] = alt_thrust + gravity
        
        # Apply velocities to position
        self.state['position'][0] += self.state['velocity'][0] * dt
        self.state['position'][1] += self.state['velocity'][1] * dt
        self.state['position'][2] += self.state['velocity'][2] * dt
        
        # Floor collision
        if self.state['position'][1] < 0:
            self.state['position'][1] = 0
            self.state['velocity'][1] = 0
            
        # Logging path
        # Log roughly every integer coordinate change to save memory
        px, py, pz = self.state['position']
        if not self.state['path'] or (abs(self.state['path'][-1][0] - px) > 0.5 or abs(self.state['path'][-1][2] - pz) > 0.5):
            self.state['path'].append((px, py, pz))
            
        # Friction
        self.state['velocity'][0] *= 0.90
        self.state['velocity'][2] *= 0.90
