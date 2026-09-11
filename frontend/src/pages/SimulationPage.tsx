function SimulationPage() {
  return (
    <div className="space-y-5">
      <div>
        <div className="text-[12px] uppercase tracking-[0.28em] text-text/60">Future integration</div>
        <h1 className="mt-2 text-[32px] uppercase tracking-[0.14em] text-text">SIMULATION</h1>
      </div>

      <div className="panel p-5">
        <div className="mb-5 flex flex-wrap gap-3 text-[10px] uppercase tracking-[0.18em] text-text/70">
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">Start Simulation</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">Pause</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">Reset</button>
        </div>

        <div className="grid gap-4 md:grid-cols-2">
          <div>
            <div className="mb-3 text-[10px] uppercase tracking-[0.2em] text-text/60">Failure injection</div>
            <div className="space-y-2 text-[10px] uppercase tracking-[0.16em] text-text/75">
              <button className="block w-full rounded border border-white/10 bg-white/5 px-3 py-2 text-left">GPS LOSS</button>
              <button className="block w-full rounded border border-white/10 bg-white/5 px-3 py-2 text-left">COMMUNICATION LOSS</button>
              <button className="block w-full rounded border border-white/10 bg-white/5 px-3 py-2 text-left">LOW BATTERY</button>
              <button className="block w-full rounded border border-white/10 bg-white/5 px-3 py-2 text-left">SENSOR FAILURE</button>
            </div>
          </div>
          <div>
            <div className="mb-3 text-[10px] uppercase tracking-[0.2em] text-text/60">Scenario injection</div>
            <div className="space-y-2 text-[10px] uppercase tracking-[0.16em] text-text/75">
              <button className="block w-full rounded border border-white/10 bg-white/5 px-3 py-2 text-left">SPAWN VICTIM</button>
              <button className="block w-full rounded border border-white/10 bg-white/5 px-3 py-2 text-left">SPAWN FIRE</button>
              <button className="block w-full rounded border border-white/10 bg-white/5 px-3 py-2 text-left">SPAWN FLOOD</button>
              <button className="block w-full rounded border border-white/10 bg-white/5 px-3 py-2 text-left">SPAWN SMOKE</button>
            </div>
          </div>
        </div>

        <div className="mt-6 grid gap-3 md:grid-cols-2 xl:grid-cols-4 text-[10px] uppercase tracking-[0.18em] text-text/70">
          {[
            ['Gazebo', 'CONNECTED'],
            ['PX4', 'CONNECTED'],
            ['ROS 2', 'CONNECTED'],
            ['MAVLink', 'CONNECTED'],
          ].map(([label, value]) => (
            <div key={label} className="rounded border border-white/10 bg-white/5 p-3">
              <div>{label}</div>
              <div className="mt-2 text-[14px] text-text">{value}</div>
            </div>
          ))}
        </div>
      </div>
    </div>
  )
}

export default SimulationPage
