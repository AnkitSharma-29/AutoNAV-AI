import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Environment, Box, Stats, Line } from '@react-three/drei';
import { useRef, useMemo } from 'react';
import * as THREE from 'three';

// 1. Realistic Drone Model Placeholder
function DroneModel({ position, heading }: { position: number[], heading: number }) {
  const group = useRef<THREE.Group>(null);
  
  useFrame((state) => {
    if (group.current) {
      // Hover animation + apply heading
      group.current.position.set(position[0], position[1] + Math.sin(state.clock.elapsedTime * 4) * 0.05, position[2]);
      group.current.rotation.y = THREE.MathUtils.degToRad(heading);
    }
  });

  return (
    <group ref={group}>
      <Box args={[1.2, 0.2, 1.2]}>
        <meshStandardMaterial color="#334155" metalness={0.8} />
      </Box>
      {/* Front indicator */}
      <Box args={[0.2, 0.3, 0.2]} position={[0, 0.15, -0.6]}>
        <meshStandardMaterial color="#22d3ee" emissive="#22d3ee" emissiveIntensity={2} />
      </Box>
    </group>
  );
}

// 2. Simulated Environment Walls (Tunnel/Rooms)
function GridEnvironment({ envMap }: { envMap: any }) {
  if (!envMap) return null;
  const walls = [];
  const size = envMap.size;
  const grid = envMap.grid;

  for (let z = 0; z < size; z++) {
    for (let x = 0; x < size; x++) {
      if (grid[z][x] === 1) {
        walls.push(
          <Box key={`${x}-${z}`} args={[1, 4, 1]} position={[x, 2, z]}>
            <meshStandardMaterial color="#0f172a" transparent opacity={0.8} />
          </Box>
        );
      }
    }
  }
  return <group>{walls}</group>;
}

// 3. LiDAR Scan Visualization
function LidarCloud({ lidar, dronePos }: { lidar: any[], dronePos: number[] }) {
  const points = useMemo(() => {
    if (!lidar || lidar.length === 0) return new Float32Array(0);
    const pts = new Float32Array(lidar.length * 3);
    lidar.forEach((scan, i) => {
      const angleRad = THREE.MathUtils.degToRad(scan.angle);
      const dist = scan.distance;
      // Calculate local hit point
      const hx = dronePos[0] - Math.sin(angleRad) * dist;
      const hz = dronePos[2] - Math.cos(angleRad) * dist;
      pts[i*3] = hx;
      pts[i*3+1] = dronePos[1]; // at drone height
      pts[i*3+2] = hz;
    });
    return pts;
  }, [lidar, dronePos]);

  if (points.length === 0) return null;

  return (
    <points>
      <bufferGeometry>
        <bufferAttribute attach="attributes-position" count={points.length / 3} array={points} itemSize={3} />
      </bufferGeometry>
      <pointsMaterial size={0.3} color="#ef4444" sizeAttenuation transparent opacity={0.8} />
    </points>
  );
}

export default function DroneScene({ telemetry, envMap }: { telemetry: any, envMap: any }) {
  // Convert A* path to 3D Line points
  const plannedPoints = useMemo(() => {
    if (!telemetry.planned_path) return [];
    return telemetry.planned_path.map((pt: number[]) => [pt[0], 0.5, pt[1]]);
  }, [telemetry.planned_path]);

  // Convert Historical Path to 3D Line points
  const historyPoints = useMemo(() => {
    if (!telemetry.path) return [];
    return telemetry.path.map((pt: number[]) => [pt[0], pt[1], pt[2]]);
  }, [telemetry.path]);

  return (
    <div className="absolute inset-0 bg-slate-900 bg-gradient-to-t from-slate-950 to-slate-900">
      <Canvas camera={{ position: [telemetry.position[0], 15, telemetry.position[2] + 15], fov: 60 }}>
        <fog attach="fog" args={['#0f172a', 10, 50]} />
        <ambientLight intensity={0.5} />
        <directionalLight position={[10, 10, 5]} intensity={1} />
        
        {/* Drone & Sensors */}
        <DroneModel position={telemetry.position} heading={telemetry.heading} />
        <LidarCloud lidar={telemetry.lidar} dronePos={telemetry.position} />
        
        {/* Navigation Lines */}
        {plannedPoints.length > 1 && (
          <Line points={plannedPoints} color="#22d3ee" lineWidth={3} dashed />
        )}
        {historyPoints.length > 1 && (
          <Line points={historyPoints} color="#f59e0b" lineWidth={2} opacity={0.5} transparent />
        )}
        
        {/* Environment Obstacles */}
        <GridEnvironment envMap={envMap} />
        
        <Grid infiniteGrid fadeDistance={50} sectionColor="#1e293b" cellColor="#0f172a" />
        <Environment preset="city" />
        
        {/* Follow camera orbit */}
        <OrbitControls target={[telemetry.position[0], 0, telemetry.position[2]]} maxPolarAngle={Math.PI / 2 - 0.1} />
        <Stats />
      </Canvas>

      <div className="absolute top-4 left-4 font-mono text-emerald-400 text-xs shadow-[0_0_10px_rgba(0,0,0,0.8)] bg-slate-950/80 p-2 rounded pointer-events-none border border-emerald-500/30">
        <div>VSLAM SYNC: ACTIVE</div>
        <div>LIDAR RAYS: {telemetry.lidar?.length || 0}</div>
        <div>A* WAYPOINTS: {telemetry.planned_path?.length || 0}</div>
      </div>
    </div>
  );
}
