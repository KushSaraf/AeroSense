import { Cpu, Eye, Radar } from 'lucide-react'
import { useApi } from '../hooks/useApi'
import { useMission } from '../hooks/useMission'
import { IS_REPLAY } from '../services/replay'

interface PerceptionStats {
  detector: string
  tracker: string
  thermalResolution: string | null
  /** null while replaying: a recording keeps the final counts, not the count at each moment. */
  framesProcessed: number | null
  rawDetectionsPerFrame: number | null
  confirmedVictims: number
  cameras: Record<string, number>
}

function Stat({ label, value, hint }: { label: string; value: string; hint?: string }) {
  return (
    <div className="rounded-lg border border-white/12 bg-[#33405c]/60 p-4">
      <div className="text-[12px] uppercase tracking-[0.1em] text-white/78">{label}</div>
      <div className="mt-2 text-[22px] font-bold text-white">{value}</div>
      {hint && <div className="mt-1 text-[11px] uppercase tracking-[0.07em] text-white/40">{hint}</div>}
    </div>
  )
}

function AiPerceptionPage() {
  const { data, error } = useApi<PerceptionStats>('/api/perception', 2000)
  const { victims } = useMission()

  return (
    <div className="min-h-full space-y-5 p-4 md:p-6">
      <header>
        <div className="aero-micro text-[12px] tracking-[0.09em] text-white/70">Perception</div>
        <h1 className="aero-heading mt-1 text-[38px] uppercase leading-none text-white">AI &amp; PERCEPTION</h1>
        <p className="mt-2 max-w-[720px] text-[12px] uppercase tracking-[0.07em] text-white/78">
          What the drone is running right now, measured from its own topics.
        </p>
      </header>

      {error && <div className="rounded border border-amber-300/30 bg-amber-300/10 px-4 py-3 text-[13px] tracking-[0.1em] text-amber-200">{error}</div>}

      <section className="rounded-xl border border-white/12 bg-[#2c374f]/70 p-5">
        <div className="flex items-center gap-2 text-[13px] uppercase tracking-[0.09em] text-white">
          <Cpu size={16} /> Detection pipeline
        </div>
        <div className="mt-4 grid gap-3 md:grid-cols-2 xl:grid-cols-4">
          <Stat label="Detector" value={data?.detector ?? '—'} hint="thermal, not RGB: body heat is the cue" />
          <Stat label="Thermal frame" value={data?.thermalResolution ?? '—'} hint="as configured by the quality profile" />
          <Stat label="Frames processed" value={data?.framesProcessed != null ? data.framesProcessed.toLocaleString() : '—'}
                hint={IS_REPLAY ? 'not recorded moment by moment' : undefined} />
          <Stat label="Raw blobs / frame" value={data?.rawDetectionsPerFrame != null ? data.rawDetectionsPerFrame.toFixed(2) : '—'}
                hint={IS_REPLAY ? 'not recorded moment by moment' : 'before corroboration'} />
        </div>
      </section>

      <section className="rounded-xl border border-white/12 bg-[#2c374f]/70 p-5">
        <div className="flex items-center gap-2 text-[13px] uppercase tracking-[0.09em] text-white">
          <Radar size={16} /> Tracking
        </div>
        <div className="mt-4 grid gap-3 md:grid-cols-3">
          <Stat label="Association" value={data?.tracker ?? '—'} />
          <Stat label="Confirmed casualties" value={String(data?.confirmedVictims ?? 0)}
                hint="published only after repeated looks agree" />
          <Stat label="Mean confidence" value={victims.length
            ? `${((victims.reduce((total, victim) => total + victim.confidence, 0) / victims.length) * 100).toFixed(0)}%`
            : '—'} />
        </div>
      </section>

      <section className="rounded-xl border border-white/12 bg-[#2c374f]/70 p-5">
        <div className="flex items-center gap-2 text-[13px] uppercase tracking-[0.09em] text-white"><Eye size={16} /> Confirmed casualties</div>
        <div className="mt-3 overflow-auto">
          <table className="w-full text-left text-[13px] uppercase tracking-[0.1em] text-white/80">
            <thead><tr className="text-[11px] text-white/78">
              <th className="pb-2">ID</th><th className="pb-2">Priority</th><th className="pb-2">Confidence</th>
              <th className="pb-2">Thermal</th><th className="pb-2">Evidence</th><th className="pb-2">Position</th>
            </tr></thead>
            <tbody>
              {victims.map((victim) => (
                <tr key={victim.id} className="border-t border-white/10">
                  <td className="py-2">{victim.id}</td>
                  <td>{victim.priority}</td>
                  <td>{(victim.confidence * 100).toFixed(0)}%</td>
                  <td>{victim.thermalStrength}</td>
                  <td className="normal-case tracking-normal text-white/75">{victim.rationale}</td>
                  <td className="text-white/75">
                    {victim.position ? `${victim.position.x.toFixed(0)}, ${victim.position.y.toFixed(0)} m` : '—'}
                  </td>
                </tr>
              ))}
              {victims.length === 0 && (
                <tr><td colSpan={6} className="py-6 text-center text-white/78">Nothing confirmed yet.</td></tr>
              )}
            </tbody>
          </table>
        </div>
      </section>
    </div>
  )
}

export default AiPerceptionPage
