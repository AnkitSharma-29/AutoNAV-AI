from flask import Flask, render_template, jsonify, request
from sim_logic import run_astar_sim, run_pid_sim, run_slam_sim

# Serve static files from the current directory
app = Flask(__name__, static_url_path='/static', static_folder='.')
app.config['TEMPLATES_AUTO_RELOAD'] = True

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/run/<sim_type>')
def run_simulation(sim_type):
    # Map sim types to their logic functions
    sim_funcs = {
        'astar': run_astar_sim,
        'pid': run_pid_sim,
        'slam': run_slam_sim
    }
    
    if sim_type in sim_funcs:
        try:
            # Execute simulation logic directly
            result = sim_funcs[sim_type](request.args)
            
            # Return JSON directly to the frontend
            return jsonify({
                'success': True,
                'data': result
            })
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)})
    
    return jsonify({'success': False, 'error': 'Invalid simulation type'})

# For Vercel deployment
app = app

if __name__ == '__main__':
    print("Dashboard starting at http://127.0.0.1:5000")
    app.run(debug=True, port=5000)
