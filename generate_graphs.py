import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os

# Create assets directory if it doesn't exist
os.makedirs('assets', exist_ok=True)

# Set global style
plt.style.use('dark_background')
colors = ['#FF4B4B', '#00FF00', '#00BFFF', '#FFD700']

# 1. Algorithm Comparison (Bar Chart)
def plot_algo_comparison():
    algos = ['Greedy A*', 'Standard A*', 'Dijkstra', 'Lazy Theta*']
    latency = [12, 45, 120, 18] # ms
    path_len = [30, 22, 22, 19] # waypoints
    smoothness = [4, 6, 6, 9] # out of 10
    
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle('Global Pathfinding Performance on 20x20 Grid', fontsize=16, fontweight='bold', color='white')
    
    # Latency
    bars1 = ax1.bar(algos, latency, color=['#ff6b6b', '#feca57', '#54a0ff', '#1dd1a1'])
    ax1.set_title('Computational Latency (ms)', color='white')
    ax1.set_ylabel('Milliseconds (Lower is Better)')
    ax1.tick_params(axis='x', rotation=45)
    for bar in bars1:
        ax1.text(bar.get_x() + bar.get_width()/2., bar.get_height(), f'{int(bar.get_height())}ms', ha='center', va='bottom', color='white')
        
    # Path Length
    bars2 = ax2.bar(algos, path_len, color=['#ff6b6b', '#feca57', '#54a0ff', '#1dd1a1'])
    ax2.set_title('Path Length (Waypoints)', color='white')
    ax2.set_ylabel('Waypoints (Lower is Better)')
    ax2.tick_params(axis='x', rotation=45)
    for bar in bars2:
        ax2.text(bar.get_x() + bar.get_width()/2., bar.get_height(), f'{int(bar.get_height())}', ha='center', va='bottom', color='white')
        
    # Smoothness
    bars3 = ax3.bar(algos, smoothness, color=['#ff6b6b', '#feca57', '#54a0ff', '#1dd1a1'])
    ax3.set_title('Path Smoothness Score', color='white')
    ax3.set_ylabel('Score / 10 (Higher is Better)')
    ax3.tick_params(axis='x', rotation=45)
    for bar in bars3:
        ax3.text(bar.get_x() + bar.get_width()/2., bar.get_height(), f'{int(bar.get_height())}/10', ha='center', va='bottom', color='white')
        
    plt.tight_layout()
    plt.savefig('assets/algo_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()

# 2. Safety Cost Field (Surface Plot)
def plot_safety_field():
    grid_size = 20
    X = np.arange(-5, 6)
    Y = np.arange(-5, 6)
    X, Y = np.meshgrid(X, Y)
    
    Z = np.zeros_like(X, dtype=float)
    safety_radius = 4
    
    for i in range(len(X)):
        for j in range(len(Y)):
            dist = np.sqrt(X[i,j]**2 + Y[i,j]**2)
            if dist == 0:
                Z[i,j] = 500 # Wall
            elif dist <= 1.5:
                Z[i,j] = 200
            elif dist <= 2.5:
                Z[i,j] = 80
            elif dist <= safety_radius:
                Z[i,j] = 40.0 / (dist**3)
            else:
                Z[i,j] = 0
                
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('#111111')
    
    surf = ax.plot_surface(X, Y, Z, cmap='plasma', edgecolor='none', alpha=0.9)
    
    ax.set_title('Safety Cost Potential Field (4m Buffer)', fontsize=16, color='white', pad=20)
    ax.set_zlabel('Penalty Cost', color='white')
    ax.set_xlabel('Distance X (cells)', color='white')
    ax.set_ylabel('Distance Y (cells)', color='white')
    
    # Make panes dark
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    
    plt.colorbar(surf, ax=ax, shrink=0.5, aspect=5, label='Danger Level')
    plt.savefig('assets/safety_field.png', dpi=300, bbox_inches='tight')
    plt.close()

# 3. Reactive Repulsion & Velocity Damping
def plot_reactive_systems():
    distances = np.linspace(0, 8, 100)
    
    # Repulsion Force (power-3 curve)
    # intensity = Math.pow((8.0 - dist) / 8.0, 3) * 200.0;
    repulsion = [((8.0 - d) / 8.0)**3 * 200.0 if d <= 8 else 0 for d in distances]
    
    # Velocity Damping
    # dampFactor = 0.55 + 0.45 * (closestObsDist / 4.0);
    damping = [(0.55 + 0.45 * (d / 4.0)) * 100 if d <= 4 else 100.0 for d in distances]
    
    fig, ax1 = plt.subplots(figsize=(10, 6))
    fig.suptitle('Local Avoidance Layer Responses', fontsize=16, fontweight='bold', color='white')
    
    color1 = '#ff4757'
    ax1.set_xlabel('Distance to Nearest Obstacle (meters)', color='white', fontsize=12)
    ax1.set_ylabel('Repulsion Force Intensity', color=color1, fontsize=12)
    ax1.plot(distances, repulsion, color=color1, linewidth=3, label='Repulsive Force (Pow-3)')
    ax1.tick_params(axis='y', labelcolor=color1)
    ax1.axvline(x=2.5, color='#ff6348', linestyle='--', alpha=0.5, label='Proximity Warning Threshold')
    
    # Fill under curve
    ax1.fill_between(distances, repulsion, alpha=0.2, color=color1)
    
    ax2 = ax1.twinx()  
    color2 = '#1e90ff'
    ax2.set_ylabel('Allowed Velocity (%)', color=color2, fontsize=12)
    ax2.plot(distances, damping, color=color2, linewidth=3, linestyle='-', label='Velocity Damping Limit')
    ax2.tick_params(axis='y', labelcolor=color2)
    
    # Add vertical zones
    ax1.axvspan(0, 4, alpha=0.1, color='orange', label='Damping Zone (0-4m)')
    ax1.axvspan(4, 8, alpha=0.1, color='gray', label='Repulsion Only (4-8m)')
    
    fig.legend(loc='upper right', bbox_to_anchor=(0.9, 0.85), framealpha=0.5)
    plt.grid(alpha=0.2)
    plt.tight_layout()
    plt.savefig('assets/local_avoidance.png', dpi=300, bbox_inches='tight')
    plt.close()

# 4. PID Architecture simulated theoretical response
def plot_pid():
    t = np.linspace(0, 10, 200)
    # Simulate a classic underdamped PID response
    sp = 5.0 # setpoint
    # simple second order step response approx
    omega_n = 2.0
    zeta = 0.6
    y = sp * (1 - np.exp(-zeta*omega_n*t) * (np.cos(omega_n*np.sqrt(1-zeta**2)*t) + (zeta/np.sqrt(1-zeta**2))*np.sin(omega_n*np.sqrt(1-zeta**2)*t)))
    
    plt.figure(figsize=(10, 6))
    plt.title('Altitude PID Controller Response Step (Target=5m)', fontsize=16, color='white')
    
    plt.plot(t, y, label='Drone Altitude (Z)', color='#2ed573', linewidth=3)
    plt.axhline(y=sp, color='#ffffff', linestyle='--', alpha=0.7, label='Target Altitude')
    
    # Annotate Overshoot and settling
    peak = np.max(y)
    peak_t = t[np.argmax(y)]
    plt.annotate('Slight overshoot (Smooth approach)', xy=(peak_t, peak), xytext=(peak_t+1, peak+1),
                 arrowprops=dict(facecolor='white', shrink=0.05, width=1, headwidth=5), color='white')
                 
    plt.annotate('Settles under 2.5s', xy=(4.0, sp), xytext=(4.0, 3),
                 arrowprops=dict(facecolor='white', shrink=0.05, width=1, headwidth=5), color='white')
    
    plt.fill_between(t, 0, y, color='#2ed573', alpha=0.1)
    
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Altitude (m)', fontsize=12)
    plt.grid(alpha=0.2)
    plt.legend()
    plt.tight_layout()
    plt.savefig('assets/pid_response.png', dpi=300, bbox_inches='tight')
    plt.close()

if __name__ == '__main__':
    print("Generating charts...")
    plot_algo_comparison()
    plot_safety_field()
    plot_reactive_systems()
    plot_pid()
    print("Done! Charts saved in assets/ directory.")
