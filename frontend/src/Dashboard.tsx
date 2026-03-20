import { useState, useEffect } from 'react';

export default function Dashboard({ telemetry, onCommand, commandLog }: any) {
  const [cmdInput, setCmdInput] = useState('');

  const sendCommand = (e: React.FormEvent) => {
    e.preventDefault();
    if (!cmdInput.trim()) return;
    onCommand(cmdInput.trim());
    setCmdInput('');
  };

  useEffect(() => {
    // Keyboard listener for manual control
    const handleKeyDown = (e: KeyboardEvent) => {
      // Don't trigger if user is typing in the command box
      if (document.activeElement?.tagName === 'INPUT') return;

      const keyMap: Record<string, string> = {
        'ArrowUp': 'FWD',
        'w': 'FWD',
        'ArrowDown': 'BWD',
        's': 'BWD',
        'ArrowLeft': 'LEFT',
        'a': 'LEFT',
        'ArrowRight': 'RIGHT',
        'd': 'RIGHT',
        'q': 'YAW_L',
        'e': 'YAW_R',
        'Shift': 'UP',
        'Control': 'DOWN',
        ' ': 'STOP'
      };

      if (keyMap[e.key]) {
        e.preventDefault();
        onCommand(`M_CTRL: ${keyMap[e.key]}`);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [onCommand]);

  return (
    <div className="flex flex-col h-full gap-6 text-sm font-mono">
      {/* Telemetry Block */}
      <div className="bg-slate-900 p-4 rounded-lg border border-slate-800">
        <h2 className="text-slate-400 mb-2 font-bold text-xs">TELEMETRY</h2>
        <div className="grid grid-cols-2 gap-2">
          <div className="text-slate-500">STATUS</div>
          <div className={`${telemetry.status === 'ARMED' ? 'text-emerald-400' : 'text-amber-400'} text-right`}>
            {telemetry.status}
          </div>
          <div className="text-slate-500">ALTITUDE</div>
          <div className="text-slate-200 text-right">{(telemetry.position ? telemetry.position[1] : 0).toFixed(2)} m</div>
          <div className="text-slate-500">SPEED</div>
          <div className="text-slate-200 text-right">{telemetry.speed.toFixed(2)} m/s</div>
          <div className="text-slate-500">HEADING</div>
          <div className="text-slate-200 text-right">{telemetry.heading.toFixed(1)}°</div>
        </div>
      </div>

      {/* Manual Control Mapping */}
      <div className="bg-slate-900 p-4 rounded-lg border border-slate-800">
        <h2 className="text-slate-400 mb-2 font-bold text-xs">MANUAL OVERRIDE</h2>
        <div className="grid grid-cols-2 gap-x-2 gap-y-1 text-slate-300">
          <div>W / ↑</div><div className="text-right text-slate-500">Pitch Fwd</div>
          <div>S / ↓</div><div className="text-right text-slate-500">Pitch Bwd</div>
          <div>A / ←</div><div className="text-right text-slate-500">Roll Left</div>
          <div>D / →</div><div className="text-right text-slate-500">Roll Right</div>
          <div>Q / E</div><div className="text-right text-slate-500">Yaw Left/Right</div>
          <div>Shift / Ctrl</div><div className="text-right text-slate-500">Alt Up/Down</div>
          <div>Spacebar</div><div className="text-right text-slate-500">Emergency Stop</div>
        </div>
      </div>

      {/* Command Input */}
      <div className="flex flex-col gap-2">
        <h2 className="text-slate-400 font-bold text-xs">EXECUTE COMMAND</h2>
        <form onSubmit={sendCommand} className="flex gap-2">
          <input 
            type="text" 
            value={cmdInput}
            onChange={(e) => setCmdInput(e.target.value)}
            className="flex-1 bg-slate-900 border border-slate-700 rounded px-3 py-2 text-slate-200 focus:outline-none focus:border-emerald-500 placeholder-slate-600"
            placeholder="e.g. MOVE 5m N"
          />
          <button type="submit" className="bg-emerald-600 hover:bg-emerald-500 text-white px-4 py-2 rounded font-bold transition-colors">
            EXEC
          </button>
        </form>
      </div>

      {/* Command Log */}
      <div className="flex-1 bg-black p-3 rounded-lg border border-slate-800 overflow-y-auto min-h-[150px]">
        <h2 className="text-slate-600 mb-2 font-bold text-xs">SYSTEM LOG</h2>
        <div className="flex flex-col gap-1">
          {commandLog.map((log: string, idx: number) => (
            <div key={idx} className="text-emerald-500">{log}</div>
          ))}
          {commandLog.length === 0 && <div className="text-slate-700 italic">Waiting for input...</div>}
        </div>
      </div>
    </div>
  );
}
