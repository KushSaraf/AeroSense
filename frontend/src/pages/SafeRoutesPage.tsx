function SafeRoutesPage() {
  return (
    <div className="space-y-5">
      <div>
        <div className="text-[12px] uppercase tracking-[0.28em] text-text/60">Route planning</div>
        <h1 className="mt-2 text-[32px] uppercase tracking-[0.14em] text-text">SAFE ROUTES</h1>
      </div>

      <div className="panel p-4">
        <div className="mb-4 flex items-center justify-between">
          <div className="text-[10px] uppercase tracking-[0.2em] text-text/60">Target: V001</div>
          <div className="text-[10px] uppercase tracking-[0.2em] text-text/60">Risk: LOW</div>
        </div>
        <div className="h-[380px] rounded border border-white/10 bg-[radial-gradient(circle_at_center,_rgba(139,194,150,0.2),_rgba(50,60,70,0.9)),linear-gradient(180deg,_#2a3444,_#1d2532)]" />
        <div className="mt-4 grid gap-3 md:grid-cols-3">
          {[
            ['Distance', '420 m'],
            ['ETA', '6 min'],
            ['Risk', 'LOW'],
          ].map(([label, value]) => (
            <div key={label} className="rounded border border-white/10 bg-white/5 p-3 text-center">
              <div className="text-[10px] uppercase tracking-[0.18em] text-text/60">{label}</div>
              <div className="mt-2 text-[18px] font-semibold tracking-[0.08em] text-text">{value}</div>
            </div>
          ))}
        </div>
        <div className="mt-5 flex gap-3 text-[10px] uppercase tracking-[0.18em] text-text/70">
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">Send to ground team</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">View route</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">Compare routes</button>
        </div>
      </div>
    </div>
  )
}

export default SafeRoutesPage
