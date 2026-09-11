import { FileText, Play } from 'lucide-react'
import { useState } from 'react'
import { missions } from '../data/mockMissionData'

function ReportsPage() {
  const [selectedMissionId, setSelectedMissionId] = useState(missions[0].id)
  const [generatedMissionId, setGeneratedMissionId] = useState<string | null>(null)
  const selectedMission = missions.find(({ id }) => id === selectedMissionId) ?? missions[0]
  const generatedMission = missions.find(({ id }) => id === generatedMissionId)

  return (
    <div className="space-y-5">
      <div>
        <div className="text-[12px] uppercase tracking-[0.28em] text-text/60">Mission reporting</div>
        <h1 className="mt-2 text-[32px] uppercase tracking-[0.14em] text-text">REPORTS</h1>
      </div>

      <div className="grid gap-5 xl:grid-cols-[minmax(280px,0.75fr)_minmax(0,2fr)]">
        <section className="panel p-4">
          <div className="mb-4 flex items-center justify-between">
            <div className="text-[10px] uppercase tracking-[0.2em] text-text/60">Select mission</div>
            <FileText size={16} className="text-[#8ae0ff]" />
          </div>
          <div className="space-y-2">
            {missions.map((mission) => (
              <button
                key={mission.id}
                type="button"
                onClick={() => { setSelectedMissionId(mission.id); setGeneratedMissionId(null) }}
                className={`w-full rounded-lg border p-3 text-left transition ${selectedMission.id === mission.id ? 'border-[#8ae0ff]/70 bg-[#8ae0ff]/12' : 'border-white/10 bg-white/5 hover:bg-white/10'}`}
              >
                <div className="text-[11px] font-semibold uppercase tracking-[0.12em] text-text">{mission.name}</div>
                <div className="mt-2 flex justify-between text-[9px] uppercase tracking-[0.14em] text-text/55">
                  <span>{mission.status}</span>
                  <span>{mission.progress}% complete</span>
                </div>
              </button>
            ))}
          </div>
        </section>

        <section className="panel p-5">
          <div className="flex flex-wrap items-start justify-between gap-4 border-b border-white/10 pb-4">
            <div>
              <div className="text-[10px] uppercase tracking-[0.2em] text-text/60">Report target</div>
              <h2 className="aero-heading mt-2 text-2xl uppercase tracking-[0.08em] text-text">{selectedMission.name}</h2>
              <div className="mt-2 text-[10px] uppercase tracking-[0.16em] text-text/55">{selectedMission.disasterType} · {selectedMission.owner}</div>
            </div>
            <button type="button" onClick={() => setGeneratedMissionId(selectedMission.id)} className="flex items-center gap-2 rounded border border-[#8ae0ff]/50 bg-[#8ae0ff]/15 px-4 py-3 text-[10px] uppercase tracking-[0.16em] text-text hover:bg-[#8ae0ff]/25">
              <Play size={14} /> Generate report
            </button>
          </div>

          {generatedMission && (
            <div className="mt-4 rounded border border-emerald-300/30 bg-emerald-400/10 px-3 py-2 text-[10px] uppercase tracking-[0.16em] text-emerald-100">
              Report generated for {generatedMission.name}
            </div>
          )}

          <div className="mt-5 grid gap-4 md:grid-cols-2 xl:grid-cols-4">
          {[
            ['Mission Summary', selectedMission.name],
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
        </section>
        </div>
    </div>
  )
}

export default ReportsPage
