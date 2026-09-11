const hazards = [
  { id: 'H001', type: 'FIRE', severity: 'CRITICAL', confidence: '96%', location: '28.6152, 77.2230', detected: '04:08:32', spread: 'Rapid westward', nearby: '2' },
  { id: 'H002', type: 'STRUCTURAL', severity: 'HIGH', confidence: '88%', location: '28.6181, 77.2252', detected: '04:07:14', spread: 'Partial collapse', nearby: '1' },
  { id: 'H003', type: 'FLOOD', severity: 'MODERATE', confidence: '74%', location: '28.6110, 77.2180', detected: '04:06:11', spread: 'Pooling', nearby: '1' },
]

const severityStyles: Record<string, string> = {
  CRITICAL: 'border-red-400/40 bg-red-500/10 text-red-100',
  HIGH: 'border-orange-400/40 bg-orange-500/10 text-orange-100',
  MODERATE: 'border-yellow-400/40 bg-yellow-500/10 text-yellow-100',
  CLEARED: 'border-emerald-400/40 bg-emerald-500/10 text-emerald-100',
}

function HazardsPage() {
  return (
    <div className="space-y-5">
      <div>
        <div className="text-[12px] uppercase tracking-[0.28em] text-text/60">Hazard intelligence</div>
        <h1 className="mt-2 text-[32px] uppercase tracking-[0.14em] text-text">HAZARDS</h1>
      </div>

      <div className="panel overflow-hidden">
        <table className="w-full text-left text-[11px] uppercase tracking-[0.12em] text-text/80">
          <thead className="bg-[#2b3346] text-text/70">
            <tr>
              <th className="px-4 py-3">Hazard ID</th>
              <th className="px-4 py-3">Type</th>
              <th className="px-4 py-3">Severity</th>
              <th className="px-4 py-3">Confidence</th>
              <th className="px-4 py-3">Location</th>
              <th className="px-4 py-3">Detected</th>
              <th className="px-4 py-3">Spread</th>
              <th className="px-4 py-3">Nearby Victims</th>
            </tr>
          </thead>
          <tbody>
            {hazards.map((hazard) => (
              <tr key={hazard.id} className="border-t border-white/10 bg-white/[0.02]">
                <td className="px-4 py-3">{hazard.id}</td>
                <td className="px-4 py-3">{hazard.type}</td>
                <td className="px-4 py-3"><span className={`inline-flex rounded-full border px-2 py-1 ${severityStyles[hazard.severity]}`}>{hazard.severity}</span></td>
                <td className="px-4 py-3">{hazard.confidence}</td>
                <td className="px-4 py-3">{hazard.location}</td>
                <td className="px-4 py-3">{hazard.detected}</td>
                <td className="px-4 py-3">{hazard.spread}</td>
                <td className="px-4 py-3">{hazard.nearby}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  )
}

export default HazardsPage
