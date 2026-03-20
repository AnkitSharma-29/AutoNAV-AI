import time
import threading
import math
from flask import Flask, jsonify, request
from flask_cors import CORS

from environment import Environment
from drone_control import DroneController
from sensor import SimulatedLiDAR
from planner import AStarPlanner

app = Flask(__name__)
CORS(app)

# Initialize Hackathon Architecture Modules
env = Environment(size=30, obstacle_density=0.15)
drone = DroneController()
lidar = SimulatedLiDAR(environment=env, max_range=15.0, rays=72) # High res LiDAR
planner = AStarPlanner(environment=env)

# Global autonomous state
autonomous_path = []

def physics_loop():
    global autonomous_path
    last_loop = time.time()
    
    while True:
        now = time.time()
        dt = now - last_loop
        last_loop = now
        
        # 1. Update PID and Kinematics
        drone.update_physics(dt)
        drone.state['speed'] = math.sqrt(drone.state['velocity'][0]**2 + drone.state['velocity'][2]**2)
        
        # 2. Autonomous A* Path Follower
        if autonomous_path:
            target = autonomous_path[0]
            dx = target[0] - drone.state['position'][0]
            dz = target[1] - drone.state['position'][2]
            dist = math.sqrt(dx**2 + dz**2)
            
            if dist < 0.5:
                # Reached waypoint
                autonomous_path.pop(0)
            else:
                # Fly towards waypoint
                speed_cap = 2.0
                drone.state['velocity'][0] = (dx / dist) * speed_cap
                drone.state['velocity'][2] = (dz / dist) * speed_cap
                
                # Update heading to face waypoint
                target_heading_rad = math.atan2(-dx, -dz) 
                drone.state['heading'] = math.degrees(target_heading_rad) % 360

        time.sleep(0.05) # 20Hz

threading.Thread(target=physics_loop, daemon=True).start()

@app.route('/api/map', methods=['GET'])
def get_map():
    # Send the static environment grid
    return jsonify({
        'size': env.size,
        'grid': env.grid.tolist()
    })

@app.route('/api/telemetry', methods=['GET'])
def get_telemetry():
    # Inject active LiDAR scan and path into state
    px, py, pz = drone.state['position']
    scan_data = lidar.scan(px, pz, drone.state['heading'])
    
    payload = drone.state.copy()
    payload['lidar'] = scan_data
    payload['planned_path'] = autonomous_path
    return jsonify(payload)

@app.route('/api/command', methods=['POST'])
def post_command():
    global autonomous_path
    data = request.json
    cmd = data.get('command', '').strip().upper()
    
    if cmd.startswith('NAV '):
        # Format: NAV x z
        parts = cmd.split(' ')
        if len(parts) == 3:
            gx, gz = float(parts[1]), float(parts[2])
            sx, sz = drone.state['position'][0], drone.state['position'][2]
            
            path = planner.plan((sx, sz), (gx, gz))
            if path:
                autonomous_path = path
                return jsonify({"status": "ack", "msg": f"Path planned with {len(path)} waypoints"})
            else:
                return jsonify({"status": "error", "msg": "No path found!"})

    elif cmd == 'STOP':
        autonomous_path = []
        drone.state['velocity'] = [0,0,0]
        return jsonify({"status": "ack", "msg": "Emergency Stop"})
        
    # Keyboard overrides
    elif cmd.startswith('M_CTRL:'):
        ctrl = cmd.split(':')[1].strip()
        speed_boost = 3.0
        autonomous_path = [] # Break autonomy on manual override
        
        rad = math.radians(drone.state['heading'])
        if ctrl == 'FWD':
            drone.state['velocity'][2] -= math.cos(rad) * speed_boost
            drone.state['velocity'][0] -= math.sin(rad) * speed_boost
        elif ctrl == 'BWD':
            drone.state['velocity'][2] += math.cos(rad) * speed_boost
            drone.state['velocity'][0] += math.sin(rad) * speed_boost
        elif ctrl == 'LEFT':
            drone.state['heading'] = (drone.state['heading'] + 5) % 360
        elif ctrl == 'RIGHT':
            drone.state['heading'] = (drone.state['heading'] - 5) % 360
        elif ctrl == 'UP':
            drone.target_altitude += 1.0 # Use PID setpoint instead of raw velocity
        elif ctrl == 'DOWN':
            drone.target_altitude = max(0, drone.target_altitude - 1.0)
            
    return jsonify({"status": "ack"})

if __name__ == '__main__':
    print("V2 Modular Architecture Active on http://127.0.0.1:5000")
    app.run(debug=False, port=5000, host='0.0.0.0')
