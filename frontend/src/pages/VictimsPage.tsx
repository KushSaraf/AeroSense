const victims = [
  { id: 'V001', priority: 'P1', confidence: '96%', location: '28.6139, 77.2208', thermal: 'STRONG', movement: 'STATIC', hazard: 'HIGH', accessibility: 'NO SAFE ROUTE', status: 'PENDING' },
  { id: 'V002', priority: 'P2', confidence: '89%', location: '28.6168, 77.2244', thermal: 'MEDIUM', movement: 'MOVING', hazard: 'MEDIUM', accessibility: 'ACCESSIBLE', status: 'ROUTE' },
  { id: 'V003', priority: 'P3', confidence: '81%', location: '28.6105, 77.2162', thermal: 'WEAK', movement: 'WAVING', hazard: 'LOW', accessibility: 'RESTRICTED', status: 'PENDING' },
]

function VictimsPage() {
  return (
    <div className="space-y-5">
      <div>
        <div className="text-[12px] uppercase tracking-[0.28em] text-text/60">Survivor intelligence</div>
        <h1 className="mt-2 text-[32px] uppercase tracking-[0.14em] text-text">VICTIMS</h1>
      </div>

      <div className="panel overflow-hidden">
        <table className="w-full text-left text-[11px] uppercase tracking-[0.12em] text-text/80">
          <thead className="bg-[#2b3346] text-text/70">
            <tr>
              <th className="px-4 py-3">ID</th>
              <th className="px-4 py-3">Priority</th>
              <th className="px-4 py-3">Confidence</th>
              <th className="px-4 py-3">Location</th>
              <th className="px-4 py-3">Thermal</th>
              <th className="px-4 py-3">Movement</th>
              <th className="px-4 py-3">Hazard Risk</th>
              <th className="px-4 py-3">Accessibility</th>
              <th className="px-4 py-3">Status</th>
            </tr>
          </thead>
          <tbody>
            {victims.map((victim) => (
              <tr key={victim.id} className="border-t border-white/10 bg-white/[0.02]">
                <td className="px-4 py-3">{victim.id}</td>
                <td className="px-4 py-3">{victim.priority}</td>
                <td className="px-4 py-3">{victim.confidence}</td>
                <td className="px-4 py-3">{victim.location}</td>
                <td className="px-4 py-3">{victim.thermal}</td>
                <td className="px-4 py-3">{victim.movement}</td>
                <td className="px-4 py-3">{victim.hazard}</td>
                <td className="px-4 py-3">{victim.accessibility}</td>
                <td className="px-4 py-3"><span className="rounded border border-white/10 bg-white/5 px-2 py-1">{victim.status}</span></td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  )
}

export default VictimsPage
