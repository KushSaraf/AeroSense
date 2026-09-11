import { CalendarDays, Eye, Filter, Pencil, Plus, Settings2, Trash2, UserRound } from 'lucide-react'
import { useMemo, useState } from 'react'
import { useNavigate } from 'react-router-dom'

const missionRows = [
  { name: 'Urban Search — Grid', status: 'ACTIVE', date: '10-09-25 , 12:40', owner: 'KU2025', type: 'Disaster Survey', tone: 'from-[#40604f] via-[#6b744b] to-[#273b38]', image: '/mission-tiles/satellite-center.jpg' },
  { name: 'Disaster Survey — Adaptive', status: 'ETA 12:45', date: '06-08-25 , 2:25', owner: 'KU2024', type: 'Adaptive Search', tone: 'from-[#687449] via-[#a87543] to-[#334b35]', image: '/mission-tiles/satellite-west.jpg' },
  { name: 'Perimeter Recon — Point-to-Point', status: 'DONE', date: '19-07-25 , 7:30', owner: 'KU2023', type: 'Recon', tone: 'from-[#a45f40] via-[#8e6b4b] to-[#4c3d32]', image: '/mission-tiles/satellite-east.jpg' },
  { name: 'Flood Assessment (Lawn-Mower)', status: 'COMPLETED', date: '14-07-25 , 9:10', owner: 'KU2022', type: 'Assessment', tone: 'from-[#3b7147] via-[#51785c] to-[#244252]', image: '/mission-tiles/satellite-south.jpg' },
  { name: 'Landslide Inspection (Custom)', status: 'COMPLETED', date: '02-07-25 , 6:50', owner: 'KU2021', type: 'Inspection', tone: 'from-[#315a61] via-[#4c6b54] to-[#253b4a]', image: '/mission-tiles/satellite-north.jpg' },
]

const statusColors: Record<string, string> = {
  ACTIVE: 'bg-[#39e65a] text-[#0e2415]',
  'ETA 12:45': 'bg-[#f2d72d] text-[#2c2910]',
  DONE: 'bg-[#b9c0c9] text-[#252d39]',
  COMPLETED: 'bg-[#b9c0c9] text-[#252d39]',
}

function MissionsPage() {
  const navigate = useNavigate()
  const [statusFilter, setStatusFilter] = useState('ALL')
  const [removedMissions, setRemovedMissions] = useState<string[]>([])
  const filteredMissions = useMemo(() => missionRows.filter((mission) => {
    if (removedMissions.includes(mission.name)) return false
    return statusFilter === 'ALL' || mission.status === statusFilter
  }), [removedMissions, statusFilter])

  return (
    <div className="flex h-full min-h-0 flex-col gap-3 overflow-auto p-4 md:p-5">
      <header className="flex shrink-0 items-center justify-between rounded-xl border border-white/10 bg-[#596278] px-6 py-4 shadow-[0_8px_20px_rgba(0,0,0,0.12)]">
        <div>
          <div className="text-[10px] uppercase tracking-[0.3em] text-white/65">Mission command</div>
          <h1 className="aero-heading mt-1 text-[38px] uppercase leading-none text-white">MISSIONS</h1>
        </div>
        <div className="hidden items-center gap-4 text-[10px] uppercase tracking-[0.24em] text-white/70 md:flex">
          <span>PLAN</span><span>|</span><span>DEPLOY</span><span>|</span><span>MONITOR</span><span>|</span><span>SAVE LIVES</span>
        </div>
      </header>

      <div className="flex min-h-0 flex-1 flex-col gap-4 xl:flex-row">
        <aside className="grid shrink-0 gap-3 sm:grid-cols-3 xl:flex xl:w-[255px] xl:flex-col">
          <div className="mission-side-card min-h-[120px] bg-[linear-gradient(135deg,rgba(67,93,73,0.95),rgba(26,44,48,0.96))]">
            <div className="mission-side-label">ACTIVE MISSION</div>
            <div className="mt-5 text-[30px] font-bold tracking-[0.04em] text-white">ETA 12:45</div>
            <div className="mt-3 flex justify-between text-[10px] uppercase tracking-[0.2em] text-white/75"><span>START</span><span>REPLAY</span></div>
          </div>
          <div className="mission-side-card min-h-[145px] bg-[linear-gradient(135deg,rgba(85,95,120,0.96),rgba(34,43,65,0.98))]">
            <div className="mission-side-label">DRONE REPORT</div>
            <div className="mt-5 text-center text-[34px] text-white">⌁</div>
            <div className="mt-2 text-center text-[10px] uppercase leading-[1.7] tracking-[0.18em] text-white/75">Name : AS-01<br />Model No : KU202509</div>
          </div>
          <div className="mission-side-card hidden min-h-[120px] bg-[linear-gradient(135deg,rgba(30,50,65,0.96),rgba(40,52,75,0.96))] xl:block">
            <div className="text-[10px] uppercase leading-[1.8] tracking-[0.2em] text-white/75">Autonomous<br />search and rescue</div>
            <div className="mt-4 text-right text-[10px] uppercase tracking-[0.18em] text-white/55">From disaster to hope</div>
          </div>
        </aside>

        <section className="min-w-0 flex-1 overflow-hidden rounded-xl border border-white/15 bg-[#596278] p-3">
          <div className="mb-3 flex flex-wrap gap-2">
            <button type="button" className="mission-filter"><Filter size={15} /> FILTER..</button>
            <button type="button" onClick={() => setStatusFilter(statusFilter === 'ACTIVE' ? 'ALL' : 'ACTIVE')} className={`mission-filter ${statusFilter === 'ACTIVE' ? 'mission-filter-active' : ''}`}><Settings2 size={15} /> STATUS..</button>
            <button type="button" className="mission-filter"><UserRound size={15} /> OWNER..</button>
            <button type="button" className="mission-filter"><CalendarDays size={15} /> DATE..</button>
            <button type="button" onClick={() => navigate('/dashboard/missions/new')} className="mission-filter ml-auto bg-[#69748c] text-white"><Plus size={16} /> NEW MISSION</button>
          </div>

          <div className="h-[calc(100%-52px)] overflow-auto rounded border border-white/20">
            <table className="w-full min-w-[850px] border-separate border-spacing-0 text-left text-[12px] uppercase tracking-[0.12em] text-white/90">
              <thead className="sticky top-0 z-10 bg-[#4d566c] text-[14px] text-white">
                <tr><th className="border-r border-white/20 px-4 py-4">Template</th><th className="border-r border-white/20 px-4 py-4">Status</th><th className="border-r border-white/20 px-4 py-4">Date / Time</th><th className="border-r border-white/20 px-4 py-4">Owner</th><th className="px-4 py-4">Actions</th></tr>
              </thead>
              <tbody>
                {filteredMissions.map((mission) => (
                  <tr key={mission.name} className="bg-[#626b80]/60 transition hover:bg-[#6d778d]">
                    <td className="border-r border-t border-white/20 px-3 py-2"><div className="flex items-center gap-3"><div className={`relative h-14 w-[190px] overflow-hidden rounded-md border border-white/50 bg-gradient-to-br ${mission.tone}`}><img src={mission.image} alt={`${mission.name} satellite area`} loading="lazy" className="mission-thumbnail h-full w-full object-cover" /><div className="absolute inset-0 bg-[linear-gradient(135deg,rgba(7,18,24,0.05),rgba(7,18,24,0.5))]" /><div className="absolute inset-0 bg-[linear-gradient(135deg,transparent_40%,rgba(255,255,255,0.2)_41%,transparent_43%)]" /></div><div className="min-w-[185px] text-[11px] font-bold tracking-[0.08em]">{mission.name}<div className="mt-1 text-[9px] font-normal text-white/60">{mission.type}</div></div></div></td>
                    <td className="border-r border-t border-white/20 px-4"><span className="flex items-center gap-3 font-bold"><i className={`h-4 w-4 rounded-full ${statusColors[mission.status]}`} />{mission.status}</span></td>
                    <td className="border-r border-t border-white/20 px-4 font-bold tracking-[0.08em]">{mission.date}</td>
                    <td className="border-r border-t border-white/20 px-4"><div className="flex items-center gap-3"><span className="flex h-10 w-10 items-center justify-center rounded-full border-2 border-white/55 bg-white/15"><UserRound size={21} /></span><strong>{mission.owner}</strong></div></td>
                    <td className="border-t border-white/20 px-3"><div className="flex gap-2"><button type="button" title="View mission" onClick={() => navigate('/dashboard')} className="mission-action"><Eye size={17} /></button><button type="button" title="Edit mission" onClick={() => navigate('/dashboard/missions/new')} className="mission-action"><Pencil size={17} /></button><button type="button" title="Delete mission" onClick={() => setRemovedMissions((current) => [...current, mission.name])} className="mission-action"><Trash2 size={17} /></button></div></td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </section>
      </div>
    </div>
  )
}

export default MissionsPage
