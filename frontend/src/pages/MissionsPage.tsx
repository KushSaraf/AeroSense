import { Eye, Loader2, OctagonX, Play, Radar } from 'lucide-react'
import { useNavigate } from 'react-router-dom'
import SectorThumbnail from '../components/SectorThumbnail'
import { useMission } from '../hooks/useMission'
import { useMissions } from '../hooks/useMissions'
import type { MissionCard } from '../hooks/useMissions'
import type { Victim } from '../types'
import { useSimulation } from '../hooks/useSimulation'

const statusTone: Record<string, string> = {
  ACTIVE: 'bg-[#86e2a4]',
  COMPLETED: 'bg-white/45',
  READY: 'bg-[#8ae0ff]',
}

/** The sectors this world contains, as cards an operator can fly. */
function MissionRow({ mission, busy, onStart, onView, victims, drone }: {
  mission: MissionCard
  busy: boolean
  onStart: (scenario: string) => void
  onView: () => void
  victims: Victim[]
  drone?: { x: number; y: number } | null
}) {
  const running = mission.status === 'ACTIVE'
  const inSector = victims.filter((victim) => victim.position
    && victim.position.x >= mission.bounds.minX && victim.position.x <= mission.bounds.maxX
    && victim.position.y >= mission.bounds.minY && victim.position.y <= mission.bounds.maxY)
  return (
    <tr className="text-[11px] uppercase tracking-[0.12em] text-white/85">
      <td className="flex items-center gap-3 border-t border-white/20 px-3 py-3">
        <SectorThumbnail bounds={mission.bounds} victims={inSector} active={running}
                         drone={running ? drone : null} />
        <div>
        <div className="font-semibold text-white">{mission.name}</div>
        <div className="text-[10px] text-white/55">
          {mission.disasterType} · {(mission.areaKm2 * 1e6 / 1e4).toFixed(1)} ha · {mission.type}
        </div>
        </div>
      </td>
      <td className="border-t border-white/20 px-3">
        <span className="flex items-center gap-2">
          <span className={`h-2.5 w-2.5 rounded-full ${statusTone[mission.status] ?? 'bg-white/40'}`} />
          {running ? mission.state ?? 'ACTIVE' : mission.status}
        </span>
        {running && mission.reason && (
          <div className="mt-1 max-w-[260px] truncate text-[9px] normal-case tracking-normal text-white/50" title={mission.reason}>
            {mission.reason}
          </div>
        )}
      </td>
      <td className="border-t border-white/20 px-3">
        <div className="h-1.5 w-28 overflow-hidden rounded-full bg-white/15">
          <div className="h-full rounded-full bg-[#8ae0ff]" style={{ width: `${Math.min(100, mission.coverage)}%` }} />
        </div>
        <div className="mt-1 text-[10px] text-white/60">{mission.coverage.toFixed(0)}% searched</div>
      </td>
      <td className="border-t border-white/20 px-3">
        {mission.victimsFound}
        {mission.p1 ? <span className="ml-2 rounded bg-[#e2707a]/25 px-1.5 py-0.5 text-[9px] text-[#ffb9bf]">P1 {mission.p1}</span> : null}
      </td>
      <td className="border-t border-white/20 px-3 text-white/70">{mission.elapsed ?? '—'}</td>
      <td className="border-t border-white/20 px-3">
        <div className="flex gap-2">
          <button type="button" title="Open the live dashboard" onClick={onView} className="mission-action"><Eye size={17} /></button>
          <button
            type="button"
            title={running ? 'This sector is being flown' : 'Start this mission'}
            disabled={busy || running}
            onClick={() => onStart(mission.scenario)}
            className={`mission-action ${busy || running ? 'opacity-40' : 'text-[#86e2a4]'}`}
          >
            {busy ? <Loader2 size={17} className="animate-spin" /> : <Play size={17} />}
          </button>
        </div>
      </td>
    </tr>
  )
}

function MissionsPage() {
  const navigate = useNavigate()
  const { missions, active, busy, error, startMission, abortMission } = useMissions()
  const simulation = useSimulation()
  const { victims, drone } = useMission()

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
        <aside className="grid shrink-0 gap-3 sm:grid-cols-2 xl:flex xl:w-[255px] xl:flex-col">
          <div className="mission-side-card min-h-[150px] bg-[linear-gradient(135deg,rgba(67,93,73,0.95),rgba(26,44,48,0.96))]">
            <div className="mission-side-label">ACTIVE MISSION</div>
            {active ? (
              <>
                <div className="mt-4 text-[22px] font-bold leading-tight text-white">{active.name}</div>
                <div className="mt-1 text-[11px] tracking-[0.12em] text-white/70">{active.state} · {active.elapsed}</div>
                <div className="mt-3 flex items-center justify-between text-[10px] uppercase tracking-[0.16em] text-white/80">
                  <span>{active.coverage.toFixed(0)}% searched</span>
                  <span>{active.victimsFound} found</span>
                </div>
                <button type="button" onClick={() => void abortMission()} className="mt-3 flex w-full items-center justify-center gap-2 rounded bg-[#e2707a]/25 px-2 py-2 text-[10px] uppercase tracking-[0.16em] text-[#ffc7cb] transition hover:bg-[#e2707a]/40">
                  <OctagonX size={14} /> Abort and return
                </button>
              </>
            ) : (
              <div className="mt-5 text-[12px] leading-relaxed tracking-[0.1em] text-white/70">
                No mission is flying. Start one from the list to bring up the simulation and search a sector.
              </div>
            )}
          </div>

          <div className="mission-side-card min-h-[130px] bg-[linear-gradient(135deg,rgba(85,95,120,0.96),rgba(34,43,65,0.98))]">
            <div className="mission-side-label">SIMULATION</div>
            <div className="mt-4 text-[13px] tracking-[0.14em] text-white">
              {simulation.status?.running ? `${simulation.status.processes} processes live` : 'Not running'}
            </div>
            <button
              type="button"
              disabled={simulation.phase === 'unknown' || simulation.phase === 'starting' || simulation.phase === 'stopping'}
              onClick={() => (simulation.status?.running ? void simulation.stop() : void simulation.start({ quality: 'low', gui: true }))}
              className="mt-3 w-full rounded bg-white/10 px-2 py-2 text-[10px] uppercase tracking-[0.16em] text-white transition hover:bg-white/20"
            >
              {simulation.status?.running ? 'Stop simulation' : 'Start simulation'}
            </button>
          </div>
        </aside>

        <section className="min-w-0 flex-1 overflow-hidden rounded-xl border border-white/15 bg-[#596278] p-3">
          {error && (
            <div className="mb-3 rounded border border-amber-300/30 bg-amber-300/10 px-3 py-2 text-[10px] uppercase tracking-[0.12em] text-amber-200">
              {error}
            </div>
          )}
          <div className="mb-3 flex flex-wrap items-center gap-2 text-[10px] uppercase tracking-[0.16em] text-white/70">
            <Radar size={15} />
            <span>Sectors in this world</span>
            <span className="ml-auto text-white/50">{missions.length} available</span>
          </div>
          <div className="overflow-auto">
            <table className="w-full border-collapse text-left">
              <thead>
                <tr className="text-[10px] uppercase tracking-[0.18em] text-white/60">
                  <th className="px-3 pb-2">Sector</th>
                  <th className="px-3 pb-2">Status</th>
                  <th className="px-3 pb-2">Coverage</th>
                  <th className="px-3 pb-2">Casualties</th>
                  <th className="px-3 pb-2">Elapsed</th>
                  <th className="px-3 pb-2">Actions</th>
                </tr>
              </thead>
              <tbody>
                {missions.map((mission) => (
                  <MissionRow
                    key={mission.id}
                    mission={mission}
                    busy={busy === mission.scenario}
                    onStart={(scenario) => void startMission(scenario)}
                    onView={() => navigate('/dashboard')}
                    victims={victims}
                    drone={drone?.position ? { x: drone.position.x, y: drone.position.y } : null}
                  />
                ))}
                {missions.length === 0 && !error && (
                  <tr><td colSpan={6} className="border-t border-white/20 px-3 py-6 text-center text-[11px] uppercase tracking-[0.14em] text-white/50">
                    Waiting for the dashboard bridge…
                  </td></tr>
                )}
              </tbody>
            </table>
          </div>
        </section>
      </div>
    </div>
  )
}

export default MissionsPage
