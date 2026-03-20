import { useState, useEffect } from 'react';
import DroneScene from './DroneScene';
import Dashboard from './Dashboard';

export default function App() {
  const [telemetry, setTelemetry] = useState({
    altitude: 0,
    heading: 0,
    speed: 0,
    status: 'CONNECTING...',
    position: [0, 0, 0],
    lidar: [],
    planned_path: [],
    path: []
  });

  const [envMap, setEnvMap] = useState<any>(null);
  const [commandLog, setCommandLog] = useState<string[]>([]);

  // Fetch Static Map Once
  useEffect(() => {
    fetch('http://127.0.0.1:5000/api/map')
      .then(res => res.json())
      .then(data => setEnvMap(data))
      .catch(e => console.error("Could not fetch map", e));
  }, []);

  // Telemetry Polling Loop
  useEffect(() => {
    let active = true;
    const pollTelemetry = async () => {
      while (active) {
        try {
          const res = await fetch('http://127.0.0.1:5000/api/telemetry');
          if (res.ok) {
            const data = await res.json();
            setTelemetry(data);
          } else {
            setTelemetry(prev => ({ ...prev, status: 'DISCONNECTED' }));
          }
        } catch (e) {
          setTelemetry(prev => ({ ...prev, status: 'BACKEND OFFLINE' }));
        }
        await new Promise(r => setTimeout(r, 100)); // 10Hz
      }
    };
    pollTelemetry();
    return () => { active = false; };
  }, []);

  const handleCommand = async (cmd: string) => {
    setCommandLog(prev => [`> ${cmd}`, ...prev].slice(0, 10));
    try {
      await fetch('http://127.0.0.1:5000/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command: cmd })
      });
    } catch (e) {
      setCommandLog(prev => [`Error sending command`, ...prev].slice(0, 10));
    }
  };

  return (
    <div className="flex h-screen w-full bg-slate-900 text-white overflow-hidden font-sans">
      <div className="flex-1 relative">
        <DroneScene telemetry={telemetry} envMap={envMap} />
      </div>

      <div className="w-[350px] bg-slate-950 border-l border-slate-800 p-4 flex flex-col gap-4 z-10 shadow-2xl">
        <div className="flex items-center gap-3 border-b border-slate-800 pb-4">
          <div className={`w-3 h-3 rounded-full ${telemetry.status === 'ARMED' ? 'bg-emerald-500 animate-pulse' : 'bg-red-500'}`}></div>
          <h1 className="text-xl font-bold tracking-wider text-slate-200">RAPTOR-OS</h1>
        </div>
        <Dashboard telemetry={telemetry} onCommand={handleCommand} commandLog={commandLog} />
      </div>
    </div>
  );
}
