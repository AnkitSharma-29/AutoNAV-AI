from flask import Flask, render_template, jsonify, send_from_directory
import subprocess
import os

# Serve static files from the current directory
app = Flask(__name__, static_url_path='/static', static_folder='.')

# Directory where the plots are saved
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/run/<sim_type>')
def run_simulation(sim_type):
    from flask import request
    scripts = {
        'pid': 'pid_altitude_controller.py',
        'astar': 'astar_navigation.py',
        'slam': 'gps_denied_slam.py'
    }
    
    if sim_type in scripts:
        script_path = os.path.join(BASE_DIR, scripts[sim_type])
        cmd = ['python', script_path]
        
        # Map web params to CLI args
        if sim_type == 'pid':
            cmd += ['--altitude', request.args.get('altitude', '10.0')]
            cmd += ['--wind', request.args.get('wind', '0.0')]
        elif sim_type == 'astar':
            cmd += ['--density', request.args.get('density', '0.2')]
            cmd += ['--start', request.args.get('start', '0,0')]
            cmd += ['--goal', request.args.get('goal', '19,19')]
            cmd += ['--manual_obs', request.args.get('manual_obs', '')]
        elif sim_type == 'slam':
            cmd += ['--noise', request.args.get('noise', '0.1')]
            cmd += ['--drift', request.args.get('drift', '0.08')]

        try:
            # Run the python script
            subprocess.run(cmd, check=True, cwd=BASE_DIR)
            return jsonify({'success': True})
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)})
    
    return jsonify({'success': False, 'error': 'Invalid simulation type'})

if __name__ == '__main__':
    print("Dashboard starting at http://127.0.0.1:5000")
    app.run(debug=False, port=5000)
