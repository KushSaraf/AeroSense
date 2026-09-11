function ReplayPage() {
  return (
    <div className="space-y-5">
      <div>
        <div className="text-[12px] uppercase tracking-[0.28em] text-text/60">Mission timeline</div>
        <h1 className="mt-2 text-[32px] uppercase tracking-[0.14em] text-text">REPLAY</h1>
      </div>

      <div className="panel p-4">
        <div className="mb-4 flex gap-2 text-[10px] uppercase tracking-[0.18em] text-text/70">
          {['00:00', '05:00', '10:00', '15:00'].map((stamp) => (
            <button key={stamp} className="rounded border border-white/10 bg-white/5 px-3 py-2">{stamp}</button>
          ))}
        </div>
        <div className="rounded border border-white/10 bg-white/5 p-4 text-[11px] uppercase tracking-[0.16em] text-text/75">
          TAKEOFF — SEARCH START — VICTIM DETECTED — HAZARD DETECTED — GPS LOST — VIO ACTIVE — ROUTE GENERATED — COMMS LOST — COMMS RESTORED — MISSION COMPLETE
        </div>
        <div className="mt-5 flex gap-2 text-[10px] uppercase tracking-[0.18em] text-text/70">
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">Play</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">Pause</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">Stop</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">10s Back</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">10s Forward</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">1x / 2x / 4x</button>
        </div>
      </div>
    </div>
  )
}

export default ReplayPage
