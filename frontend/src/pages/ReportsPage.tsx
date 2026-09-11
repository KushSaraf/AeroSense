function ReportsPage() {
  return (
    <div className="space-y-5">
      <div>
        <div className="text-[12px] uppercase tracking-[0.28em] text-text/60">Mission reporting</div>
        <h1 className="mt-2 text-[32px] uppercase tracking-[0.14em] text-text">REPORTS</h1>
      </div>

      <div className="panel p-5">
        <div className="grid gap-4 md:grid-cols-2 xl:grid-cols-4">
          {[
            ['Mission Summary', 'Earthquake SAR'],
            ['Area surveyed', '4.8 km²'],
            ['Flight duration', '14:32'],
            ['Coverage %', '62%'],
            ['Victims detected', '7'],
            ['P1', '2'],
            ['P2', '3'],
            ['P3', '2'],
            ['Hazards detected', '12'],
            ['Critical hazards', '3'],
            ['Safe routes', '2'],
            ['GPS-denied duration', '8 min'],
          ].map(([label, value]) => (
            <div key={label} className="rounded border border-white/10 bg-white/5 p-3">
              <div className="text-[10px] uppercase tracking-[0.18em] text-text/60">{label}</div>
              <div className="mt-2 text-[18px] font-medium tracking-[0.08em] text-text">{value}</div>
            </div>
          ))}
        </div>

        <div className="mt-6 flex flex-wrap gap-3 text-[10px] uppercase tracking-[0.18em] text-text/70">
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">Export PDF</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">Export CSV</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">Export JSON</button>
          <button className="rounded border border-white/10 bg-white/5 px-3 py-2">Mission Log</button>
        </div>
      </div>
    </div>
  )
}

export default ReportsPage
