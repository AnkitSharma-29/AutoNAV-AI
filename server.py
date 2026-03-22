from flask import Flask, render_template, jsonify, send_from_directory
import os
from astar_navigation import run_astar
from pid_altitude_controller import run_pid
from gps_denied_slam import run_slam

# Serve static files from the current directory
app = Flask(__name__, static_url_path='/static', static_folder='.')
app.config['TEMPLATES_AUTO_RELOAD'] = True
app.jinja_env.auto_reload = True

# Directory where the plots are saved
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/run/<sim_type>')
def run_simulation(sim_type):
    from flask import request
    
    # Check if we should save files (only for local dev)
    save_files = os.environ.get('VERCEL') is None

    if sim_type == 'pid':
        target_alt = float(request.args.get('altitude', '10.0'))
        wind = float(request.args.get('wind', '0.0'))
        data = run_pid(target_alt, wind, save_files=save_files)
        return jsonify({'success': True, 'data': data})
    
    elif sim_type == 'astar':
        density = float(request.args.get('density', '0.2'))
        start_str = request.args.get('start', '0,0')
        goal_str = request.args.get('goal', '19,19')
        manual_obs = request.args.get('manual_obs', '')
        
        try:
            start = tuple(int(float(x)) for x in start_str.split(','))
            goal = tuple(int(float(x)) for x in goal_str.split(','))
        except:
            start, goal = (0, 0), (19, 19)
            
        data = run_astar(density, start, goal, manual_obs, save_files=save_files)
        return jsonify({'success': True, 'data': data})
        
    elif sim_type == 'slam':
        noise = float(request.args.get('noise', '0.08'))
        drift = float(request.args.get('drift', '0.04'))
        data = run_slam(noise, drift, save_files=save_files)
        return jsonify({'success': True, 'data': data})
    
    return jsonify({'success': False, 'error': 'Invalid simulation type'})

if __name__ == '__main__':
    print("Dashboard starting at http://127.0.0.1:5000")
    app.run(debug=False, port=5000)
