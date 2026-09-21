import { Play, RefreshCw, RotateCcw, Square } from 'lucide-react'
import { useCallback, useEffect, useState } from 'react'
import { IS_REPLAY } from '../services/replay'
import { API_URL, simulationControl } from '../services/apiServices'
import type { SimulationOptions, SimulationStatus } from '../services/apiServices'
import { useDataSource } from '../hooks/useDataSource'

const QUALITIES = ['low', 'medium', 'high'] as const
const CRUISE_SPEEDS = [4, 6, 8, 12] as const
const STATUS_POLL_MS = 4000

function Row({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <div className="flex flex-wrap items-center justify-between gap-3 border-b border-white/10 py-3 last:border-b-0">
      <span className="text-[12px] uppercase tracking-[0.09em] text-text/75">{label}</span>
      <div className="flex flex-wrap gap-2">{children}</div>
    </div>
  )
}

function Choice({ active, onClick, children }: { active: boolean; onClick: () => void; children: React.ReactNode }) {
  return (
    <button type="button" onClick={onClick}
            className={`rounded border px-3 py-2 text-[12px] uppercase tracking-[0.08em] transition ${
              active ? 'border-[#8ae0ff]/60 bg-[#8ae0ff]/18 text-white' : 'border-white/12 bg-white/5 text-text/70 hover:bg-white/10'}`}>
      {children}
    </button>
  )
}

/**
 * The settings that actually change what runs: the simulation's launch options and where the
 * dashboard is reading from. Everything here is applied the next time the simulation starts.
 */
function SettingsPage() {
  const source = useDataSource()
  const [status, setStatus] = useState<SimulationStatus | null>(null)
  const [options, setOptions] = useState<Required<SimulationOptions>>({
    quality: 'low', gui: true, rviz: false, cruiseSpeed: 8,
  })
  const [busy, setBusy] = useState(false)
  const [message, setMessage] = useState<string | null>(null)

  const refresh = useCallback(async () => {
    try {
      setStatus(await simulationControl.status())
      setMessage(null)
    } catch {
      setStatus(null)
      setMessage('Dashboard bridge unreachable — run: ros2 run aero_sense_bridge dashboard_bridge')
    }
  }, [])

  useEffect(() => {
    void refresh()
    const timer = setInterval(() => void refresh(), STATUS_POLL_MS)
    return () => clearInterval(timer)
  }, [refresh])

  const act = async (what: 'start' | 'stop' | 'restart') => {
    if (what === 'restart' && !window.confirm('Restart the simulation? The mission in progress will be stopped.')) return
    setBusy(true)
    try {
      const result = what === 'start' ? await simulationControl.start(options)
        : what === 'restart' ? await simulationControl.restart(options)
          : await simulationControl.stop()
      if (result.reason) setMessage(result.reason)
      await refresh()
    } catch {
      setMessage(`could not ${what} the simulation`)
    } finally {
      setBusy(false)
    }
  }

  return (
    <div className="space-y-5 p-4 md:p-6">
      <div>
        <div className="text-[12px] uppercase tracking-[0.09em] text-text/75">System configuration</div>
        <h1 className="aero-heading mt-1 text-[36px] uppercase leading-none text-text">SETTINGS</h1>
      </div>

      {message && <div className="rounded border border-amber-300/30 bg-amber-300/10 px-4 py-3 text-[13px] tracking-[0.1em] text-amber-200">{message}</div>}

      <div className="grid gap-4 xl:grid-cols-2">
        <section className="panel p-5">
          <div className="text-[13px] uppercase tracking-[0.1em] text-white">Simulation</div>
          <div className="mt-3">
            <Row label="Status">
              <span className={`rounded px-3 py-2 text-[12px] uppercase tracking-[0.08em] ${
                status?.running ? 'bg-emerald-500/20 text-emerald-300' : 'bg-white/10 text-text/70'}`}>
                {IS_REPLAY ? 'NO SIMULATION · PLAYING A RECORDING'
                  : status ? (status.running ? `RUNNING · ${status.processes} processes` : 'STOPPED') : 'UNKNOWN'}
              </span>
            </Row>
            <Row label="Sensor quality">
              {QUALITIES.map((quality) => (
                <Choice key={quality} active={options.quality === quality}
                        onClick={() => setOptions((current) => ({ ...current, quality }))}>{quality}</Choice>
              ))}
            </Row>
            <Row label="Cruise speed (m/s)">
              {CRUISE_SPEEDS.map((speed) => (
                <Choice key={speed} active={options.cruiseSpeed === speed}
                        onClick={() => setOptions((current) => ({ ...current, cruiseSpeed: speed }))}>{speed}</Choice>
              ))}
            </Row>
            <Row label="Windows at launch">
              <Choice active={options.gui} onClick={() => setOptions((c) => ({ ...c, gui: !c.gui }))}>Gazebo</Choice>
              <Choice active={options.rviz} onClick={() => setOptions((c) => ({ ...c, rviz: !c.rviz }))}>RViz</Choice>
            </Row>
            <Row label="Control">
              <button type="button" disabled={busy || status?.running} onClick={() => void act('start')}
                      className="flex items-center gap-2 rounded border border-emerald-300/40 bg-emerald-400/15 px-4 py-2 text-[12px] uppercase tracking-[0.08em] text-emerald-100 disabled:opacity-40">
                <Play size={13} /> Start
              </button>
              <button type="button" disabled={busy || !status?.running} onClick={() => void act('stop')}
                      className="flex items-center gap-2 rounded border border-red-300/40 bg-red-400/15 px-4 py-2 text-[12px] uppercase tracking-[0.08em] text-red-100 disabled:opacity-40">
                <Square size={13} /> Stop
              </button>
              <button type="button" disabled={busy || !status?.running} onClick={() => void act('restart')}
                      className="flex items-center gap-2 rounded border border-amber-300/40 bg-amber-400/15 px-4 py-2 text-[12px] uppercase tracking-[0.08em] text-amber-100 disabled:opacity-40">
                <RotateCcw size={13} /> Restart
              </button>
              <button type="button" onClick={() => void refresh()}
                      className="flex items-center gap-2 rounded border border-white/15 bg-white/5 px-4 py-2 text-[12px] uppercase tracking-[0.08em] text-text/75">
                <RefreshCw size={13} /> Refresh
              </button>
            </Row>
          </div>
        </section>

        <section className="panel p-5">
          <div className="text-[13px] uppercase tracking-[0.1em] text-white">Data source</div>
          <div className="mt-3">
            <Row label="Dashboard bridge">
              <code className="rounded bg-white/10 px-3 py-2 text-[12px] tracking-[0.08em] text-text/80">{API_URL}</code>
            </Row>
            <Row label="Serving">
              <span className={`rounded px-3 py-2 text-[12px] uppercase tracking-[0.08em] ${
                IS_REPLAY ? 'bg-[#ffd166]/20 text-[#ffd166]'
                  : source === 'live' ? 'bg-emerald-500/20 text-emerald-300' : 'bg-amber-500/20 text-amber-300'}`}>
                {IS_REPLAY ? 'RECORDED REPLAY' : source === 'live' ? 'LIVE SIMULATION' : 'NO SIMULATION'}
              </span>
            </Row>
            <Row label="SITL port 5760">
              <span className="rounded bg-white/10 px-3 py-2 text-[12px] uppercase tracking-[0.08em] text-text/80">
                {IS_REPLAY ? 'n/a in a replay'
                  : status ? (status.port5760Free ? 'free' : 'in use') : 'unknown'}
              </span>
            </Row>
            <Row label="Override">
              <span className="text-[12px] normal-case tracking-normal text-text/72">
                Set VITE_API_URL before starting the dashboard to point it at another bridge.
              </span>
            </Row>
          </div>
        </section>
      </div>
    </div>
  )
}

export default SettingsPage
