function LiveFeedPage() {
  return (
    <div className="space-y-4">
      <div className="flex items-center justify-between">
        <div>
          <div className="text-[12px] uppercase tracking-[0.28em] text-text/60">Monitoring</div>
          <h1 className="mt-2 text-[32px] uppercase tracking-[0.14em] text-text">LIVE FEED</h1>
        </div>
        <div className="flex gap-2 text-[10px] uppercase tracking-[0.18em] text-text/70">
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">REC</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">SNAPSHOT</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">PAUSE</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">FULLSCREEN</button>
        </div>
      </div>

      <div className="grid gap-4 xl:grid-cols-2">
        <div className="panel p-3">
          <div className="mb-3 text-[10px] uppercase tracking-[0.2em] text-text/60">RGB</div>
          <div className="h-[320px] rounded border border-white/10 bg-[radial-gradient(circle_at_center,_rgba(255,255,255,0.18),_rgba(78,86,108,0.5)),linear-gradient(180deg,_#5f6a82,_#3c475d)]"> </div>
          <div className="mt-4 flex justify-between text-[10px] uppercase tracking-[0.16em] text-text/70">
            <span>FPS 24</span>
            <span>Resolution 1920 × 1080</span>
            <span>Inference 28 ms</span>
          </div>
        </div>
        <div className="panel p-3">
          <div className="mb-3 text-[10px] uppercase tracking-[0.2em] text-text/60">Thermal</div>
          <div className="h-[320px] rounded border border-white/10 bg-[linear-gradient(135deg,_rgba(255,149,0,0.22),_rgba(19,32,49,0.8),_rgba(255,68,68,0.2)),linear-gradient(180deg,_#2f4058,_#252e3b)]"> </div>
          <div className="mt-4 flex justify-between text-[10px] uppercase tracking-[0.16em] text-text/70">
            <span>Model 96%</span>
            <span>Thermal view</span>
            <span>Confidence 91%</span>
          </div>
        </div>
      </div>

      <div className="panel p-4">
        <div className="mb-3 text-[10px] uppercase tracking-[0.2em] text-text/60">Detections</div>
        <div className="grid gap-4 md:grid-cols-4">
          {[
            ['PERSON', '96%'],
            ['HAZARD', '89%'],
            ['V001', 'P1'],
            ['V002', 'P2'],
          ].map(([label, value]) => (
            <div key={label} className="rounded border border-white/10 bg-white/5 p-3 text-center">
              <div className="text-[10px] uppercase tracking-[0.18em] text-text/60">{label}</div>
              <div className="mt-2 text-[20px] font-semibold tracking-[0.08em] text-text">{value}</div>
            </div>
          ))}
        </div>
      </div>
    </div>
  )
}

export default LiveFeedPage
