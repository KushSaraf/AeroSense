const alerts = [
  { id: 'A-01', time: '04:12:18', type: 'P1', location: '28.6139, 77.2208', confidence: '96%', evidence: 'Thermal spike', rationale: 'Strong thermal signature near active fire. No clear egress route.', status: 'OPEN' },
  { id: 'A-02', time: '04:11:42', type: 'P2', location: '28.6168, 77.2244', confidence: '89%', evidence: 'Movement in debris', rationale: 'Possible survivor behind collapsed wall. Route assessment in progress.', status: 'ACKNOWLEDGED' },
  { id: 'A-03', time: '04:09:54', type: 'P3', location: '28.6105, 77.2162', confidence: '81%', evidence: 'Signal intermittently visible', rationale: 'Accessible via north corridor. Recommended for clearance.', status: 'ASSIGNED' },
]

function AlertsPage() {
  return (
    <div className="space-y-5">
      <div className="flex items-center justify-between">
        <div>
          <div className="text-[12px] uppercase tracking-[0.28em] text-text/60">Alert center</div>
          <h1 className="mt-2 text-[32px] uppercase tracking-[0.14em] text-text">ALERTS</h1>
        </div>
        <div className="flex gap-2 text-[10px] uppercase tracking-[0.18em] text-text/70">
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">All</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">P1</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">P2</button>
        </div>
      </div>

      <div className="grid gap-4">
        {alerts.map((alert) => (
          <div key={alert.id} className="panel p-4">
            <div className="flex flex-wrap items-center justify-between gap-3">
              <div className="flex items-center gap-3">
                <span className="rounded border border-white/10 bg-white/5 px-2 py-1 text-[10px] uppercase tracking-[0.18em] text-text">{alert.type}</span>
                <span className="text-[11px] uppercase tracking-[0.18em] text-text/70">{alert.id}</span>
              </div>
              <div className="text-[10px] uppercase tracking-[0.18em] text-text/60">{alert.time}</div>
            </div>
            <div className="mt-3 grid gap-4 md:grid-cols-[1.4fr_1fr_1fr]">
              <div>
                <div className="text-[11px] uppercase tracking-[0.18em] text-text/70">Location</div>
                <div className="mt-1 text-[13px] text-text">{alert.location}</div>
                <div className="mt-3 text-[10px] leading-5 text-text/75">{alert.rationale}</div>
              </div>
              <div>
                <div className="text-[11px] uppercase tracking-[0.18em] text-text/70">Confidence</div>
                <div className="mt-1 text-[13px] text-text">{alert.confidence}</div>
                <div className="mt-3 text-[11px] uppercase tracking-[0.16em] text-text/70">Evidence</div>
                <div className="mt-1 text-text">{alert.evidence}</div>
              </div>
              <div>
                <div className="text-[11px] uppercase tracking-[0.18em] text-text/70">Status</div>
                <div className="mt-1 text-[12px] text-text">{alert.status}</div>
                <div className="mt-3 flex gap-2 text-[9px] uppercase tracking-[0.14em] text-text/70">
                  <button className="rounded border border-white/10 bg-white/5 px-2 py-1">Acknowledge</button>
                  <button className="rounded border border-white/10 bg-white/5 px-2 py-1">View map</button>
                </div>
              </div>
            </div>
          </div>
        ))}
      </div>
    </div>
  )
}

export default AlertsPage
