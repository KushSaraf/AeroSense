import { AlertTriangle, Boxes, Layers3, MapPinned, Radio, RotateCcw, Thermometer, Video } from 'lucide-react'
import { useEffect, useMemo, useState } from 'react'
import { CircleMarker, MapContainer, Polyline, Popup, TileLayer } from 'react-leaflet'
import StatusPill from '../components/StatusPill'
import { API_URL, simulationControl } from '../services/apiServices'
import { useMission } from '../hooks/useMission'
import type { Victim } from '../types'
import 'leaflet/dist/leaflet.css'

/** The world's origin, from the simulation's spherical coordinates. */
const DEFAULT_CENTRE: [number, number] = [-35.363262, 149.165237]
/** The stages the mission state machine actually goes through, in order. */
const MISSION_STAGES = [
  { state: 'PRE_FLIGHT', label: 'Pre-flight' },
  { state: 'TAKEOFF', label: 'Takeoff' },
  { state: 'SEARCHING', label: 'Search the sector' },
  { state: 'VICTIM_DETECTED', label: 'Inspect casualties' },
  { state: 'RETURNING', label: 'Return to base' },
  { state: 'LANDING', label: 'Land' },
  { state: 'MISSION_COMPLETE', label: 'Complete' },
]
const PRIORITY_TONE: Record<string, string> = {
  P1: '#ef5350', P2: '#ff9f43', P3: '#43d17b', UNTRIAGED: '#8ae0ff',
}

const RESTART_READY_TIMEOUT_MS = 240000
const RESTART_POLL_MS = 4000

/**
 * The simulation's own controls, beside the mission they affect. Gazebo and RViz attach to the
 * simulation already running; restart tears it down and brings up a fresh one, which is the way
 * out of a crashed drone or a wedged autopilot without leaving the dashboard.
 */
function SimulationControls() {
  const [busy, setBusy] = useState<string | null>(null)
  const [note, setNote] = useState<{ text: string; tone: 'warn' | 'ok' } | null>(null)

  const open = async (kind: 'gazebo' | 'rviz') => {
    setBusy(kind)
    setNote(null)
    try {
      const result = await simulationControl.openViewer(kind)
      setNote(result.opened
        ? { text: `${kind} opened`, tone: 'ok' }
        : { text: result.reason ?? `could not open ${kind}`, tone: 'warn' })
    } catch {
      setNote({ text: 'the dashboard bridge is not reachable', tone: 'warn' })
    } finally {
      setBusy(null)
    }
  }

  const restart = async () => {
    if (!window.confirm('Restart the simulation? The mission in progress will be stopped.')) return
    setBusy('restart')
    setNote({ text: 'stopping the simulation…', tone: 'ok' })
    try {
      const result = await simulationControl.restart({ quality: 'low', gui: true, cruiseSpeed: 8 })
      if (!result.started) {
        setNote({ text: result.reason ?? 'the simulation did not start', tone: 'warn' })
        return
      }
      setNote({ text: 'waiting for the drone…', tone: 'ok' })
      const deadline = Date.now() + RESTART_READY_TIMEOUT_MS
      while (Date.now() < deadline) {
        await new Promise((resolve) => setTimeout(resolve, RESTART_POLL_MS))
        const state = await fetch(`${API_URL}/api/state`).then((r) => r.json()).catch(() => null)
        if (state?.connected) {
          setNote({ text: 'simulation ready — start a sector from Mission Command', tone: 'ok' })
          return
        }
      }
      setNote({ text: 'the simulation did not come up in time; see logs/dashboard', tone: 'warn' })
    } catch {
      setNote({ text: 'the dashboard bridge is not reachable', tone: 'warn' })
    } finally {
      setBusy(null)
    }
  }

  const button = 'flex items-center gap-2 rounded border px-3 py-2 text-[10px] uppercase tracking-[0.16em] text-white transition disabled:opacity-50'
  return (
    <div className="flex flex-col items-end gap-1">
      <div className="flex flex-wrap justify-end gap-2">
        {([['gazebo', 'GAZEBO', Boxes], ['rviz', 'RVIZ', Layers3]] as const).map(([kind, label, Icon]) => (
          <button key={kind} type="button" onClick={() => void open(kind)} disabled={busy !== null}
                  title={`Open ${label} on the running simulation`}
                  className={`${button} border-white/25 bg-white/10 hover:bg-white/20`}>
            <Icon size={14} /> {busy === kind ? 'OPENING…' : label}
          </button>
        ))}
        <button type="button" onClick={() => void restart()} disabled={busy !== null}
                title="Stop the simulation and start a fresh one"
                className={`${button} border-amber-300/40 bg-amber-400/15 hover:bg-amber-400/25`}>
          <RotateCcw size={14} /> {busy === 'restart' ? 'RESTARTING…' : 'RESTART SIM'}
        </button>
      </div>
      {note && (
        <span className={`max-w-[420px] text-right text-[9px] uppercase tracking-[0.12em] ${
          note.tone === 'warn' ? 'text-amber-300' : 'text-[#86e2a4]'}`}>{note.text}</span>
      )}
    </div>
  )
}

function Panel({ title, icon, children, action }: {
  title: string; icon?: React.ReactNode; children: React.ReactNode; action?: React.ReactNode
}) {
  return (
    <section className="panel overflow-hidden p-3">
      <div className="mb-3 flex items-center justify-between text-[11px] font-bold uppercase tracking-[0.16em] text-white">
        <span className="flex items-center gap-2">{icon}{title}</span>
        {action}
      </div>
      {children}
    </section>
  )
}

function CameraFeed() {
  const [camera, setCamera] = useState<'rgb' | 'thermal' | 'both'>('rgb')
  const feed = (which: 'rgb' | 'thermal') => (
    <img
      key={which}
      src={`${API_URL}/api/camera/${which}`}
      alt={`${which} camera`}
      className="h-28 w-full rounded border border-white/15 object-cover"
    />
  )
  return (
    <>
      <div className="mt-3 grid grid-cols-3 gap-1">
        {(['rgb', 'thermal', 'both'] as const).map((option) => (
          <button
            key={option}
            type="button"
            onClick={() => setCamera(option)}
            className={`rounded border px-2 py-2 text-[9px] uppercase ${
              camera === option ? 'border-[#8ae0ff]/50 bg-[#8ae0ff]/15 text-white' : 'border-white/15 bg-white/5 text-white/70'
            }`}
          >
            {option === 'rgb' ? <Video size={13} className="mx-auto mb-1" />
              : option === 'thermal' ? <Thermometer size={13} className="mx-auto mb-1" /> : null}
            {option === 'rgb' ? 'Live feed' : option === 'thermal' ? 'Thermal' : 'Both'}
          </button>
        ))}
      </div>
      <div className={`mt-3 grid gap-2 ${camera === 'both' ? 'grid-cols-2' : 'grid-cols-1'}`}>
        {camera === 'both' ? [feed('rgb'), feed('thermal')] : feed(camera)}
      </div>
    </>
  )
}

function VictimRow({ victim }: { victim: Victim }) {
  return (
    <div className="flex items-center justify-between border-b border-white/10 py-2 text-[10px] uppercase tracking-[0.1em] text-white/75">
      <span className="flex items-center gap-2">
        <span className="h-2 w-2 rounded-full" style={{ backgroundColor: PRIORITY_TONE[victim.priority] ?? '#8ae0ff' }} />
        {victim.id}
      </span>
      <span className="text-white/55">{victim.thermalStrength} · {(victim.confidence * 100).toFixed(0)}%</span>
    </div>
  )
}

function DashboardPage() {
  const { mission, drone, victims, source, loading } = useMission()
  const [track, setTrack] = useState<[number, number][]>([])

  // the flown track is built from the positions that arrive, not from a stored route
  useEffect(() => {
    if (drone?.latitude == null || drone?.longitude == null) return
    setTrack((current) => {
      const point: [number, number] = [drone.latitude as number, drone.longitude as number]
      const last = current[current.length - 1]
      if (last && Math.abs(last[0] - point[0]) < 1e-6 && Math.abs(last[1] - point[1]) < 1e-6) return current
      return [...current.slice(-400), point]
    })
  }, [drone?.latitude, drone?.longitude])

  const counts = useMemo(() => {
    const tally: Record<string, number> = { P1: 0, P2: 0, P3: 0, UNTRIAGED: 0 }
    victims.forEach((victim) => { tally[victim.priority] = (tally[victim.priority] ?? 0) + 1 })
    return tally
  }, [victims])

  const centre: [number, number] = drone?.latitude != null && drone?.longitude != null
    ? [drone.latitude, drone.longitude]
    : DEFAULT_CENTRE
  const stageIndex = MISSION_STAGES.findIndex((stage) => stage.state === mission?.state)

  if (loading) {
    return <div className="grid min-h-full place-items-center text-[11px] uppercase tracking-[0.2em] text-white/60">Connecting…</div>
  }

  return (
    <div className="dashboard-page min-h-full bg-[#202635] text-text">
      <div className="mx-auto max-w-[1800px] space-y-3">
        <header className="flex min-h-[62px] items-center justify-between rounded-lg border border-white/15 bg-[#4e586e] px-4 py-3">
          <div>
            <div className="aero-micro text-[9px] text-white/55">Mission / active operation</div>
            <h1 className="aero-heading mt-1 text-[26px] uppercase leading-none text-white md:text-[32px]">
              {mission ? `MISSION: ${mission.name}` : 'NO ACTIVE MISSION'}
            </h1>
            <div className="mt-1 text-[10px] uppercase tracking-[0.18em] text-white/70">
              {mission ? `${mission.state ?? mission.status} — ${mission.reason ?? ''}`
                : 'Start a sector from Mission Command to fly one'}
            </div>
          </div>
          <div className="flex items-center gap-4">
            <SimulationControls />
            <div className="hidden text-right text-[10px] uppercase tracking-[0.16em] text-white/70 lg:block">
            <span className={source === 'live' ? 'text-[#86e2a4]' : 'text-amber-300'}>
              ● {source === 'live' ? 'SYSTEM ONLINE' : 'NO SIMULATION'}
            </span><br />
            {mission?.elapsed ? `ELAPSED ${mission.elapsed}` : 'ONE DRONE | MANY LIVES'}
            </div>
          </div>
        </header>

        <div className="dashboard-layout">
          <Panel title="Drone 1" icon={<Radio size={15} />}
                 action={<StatusPill label={drone?.status ?? 'OFFLINE'} tone={drone?.armed ? 'green' : 'gray'} />}>
            <div className="space-y-2 border-b border-white/10 pb-3 text-[10px] uppercase tracking-[0.13em] text-white/70">
              {[
                ['Model', drone?.model ?? '—'],
                ['Callsign', drone?.id ?? '—'],
                ['Battery', drone?.battery != null ? `${drone.battery.toFixed(0)}%` : '—'],
                ['Flight time', drone?.flightTime ?? '—'],
                ['Altitude', drone ? `${drone.altitude.toFixed(1)} m` : '—'],
                ['Speed', drone ? `${drone.speed.toFixed(1)} m/s` : '—'],
                ['GPS', drone?.gps ?? '—'],
                ['Mode', drone?.mode ?? '—'],
              ].map(([label, value]) => (
                <div key={label} className="flex justify-between gap-3"><span>{label}</span><strong className="text-white">{value}</strong></div>
              ))}
            </div>
            <CameraFeed />
          </Panel>

          <Panel title="Map view" icon={<MapPinned size={15} />}>
            <div className="relative h-[440px] overflow-hidden rounded-md border border-white/15">
              <MapContainer center={centre} zoom={17} style={{ height: '100%', width: '100%' }}>
                <TileLayer attribution="&copy; OpenStreetMap contributors" url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
                {victims.map((victim) => (
                  <CircleMarker
                    key={victim.id}
                    center={[victim.latitude, victim.longitude]}
                    radius={7}
                    pathOptions={{ color: PRIORITY_TONE[victim.priority] ?? '#8ae0ff', fillOpacity: 0.85 }}
                  >
                    <Popup>{victim.id} · {victim.priority} · {(victim.confidence * 100).toFixed(0)}%<br />{victim.rationale}</Popup>
                  </CircleMarker>
                ))}
                {track.length > 1 && <Polyline positions={track} pathOptions={{ color: '#42d7c7', weight: 3 }} />}
                {drone?.latitude != null && drone?.longitude != null && (
                  <CircleMarker center={[drone.latitude, drone.longitude]} radius={8}
                                pathOptions={{ color: '#42d7c7', fillColor: '#42d7c7', fillOpacity: 1 }}>
                    <Popup>{drone.id} · {drone.altitude.toFixed(0)} m</Popup>
                  </CircleMarker>
                )}
              </MapContainer>
            </div>
          </Panel>

          <Panel title="Mission events"
                 action={<span className="rounded border border-white/15 px-2 py-1 text-[9px] text-white/60">{mission?.events?.length ?? 0}</span>}>
            <div className="max-h-[440px] space-y-2 overflow-auto">
              {(mission?.events ?? []).slice().reverse().map((event, index) => (
                <div key={`${event.time}-${index}`} className="rounded-md border border-white/10 bg-[#3a465f]/65 p-2.5">
                  <div className="flex gap-2">
                    <div className="flex h-7 w-7 shrink-0 items-center justify-center rounded bg-[#8ae0ff]/15 text-[#8ae0ff]">
                      <AlertTriangle size={15} />
                    </div>
                    <div>
                      <div className="text-[10px] text-white/55">{event.time}</div>
                      <p className="mt-1 text-[10px] leading-4 text-white/80">{event.text}</p>
                    </div>
                  </div>
                </div>
              ))}
              {!mission?.events?.length && (
                <p className="text-[10px] uppercase tracking-[0.14em] text-white/45">No mission events yet.</p>
              )}
            </div>
          </Panel>
        </div>

        <div className="dashboard-lower">
          <Panel title="Mission status">
            {MISSION_STAGES.map((stage, index) => {
              const done = stageIndex > index || mission?.state === 'MISSION_COMPLETE'
              const current = stageIndex === index
              return (
                <div key={stage.state} className="flex items-center justify-between border-b border-white/10 py-2 text-[9px] uppercase text-white/70">
                  <span className="flex items-center gap-2">
                    <span className={`flex h-4 w-4 items-center justify-center rounded-full border ${
                      done ? 'border-[#43d17b] text-[#43d17b]' : current ? 'border-[#8ae0ff] text-[#8ae0ff]' : 'border-white/25'}`}>
                      {done ? '✓' : '○'}
                    </span>
                    {stage.label}
                  </span>
                  <span>{done ? 'Done' : current ? 'In progress' : 'Pending'}</span>
                </div>
              )
            })}
          </Panel>

          <Panel title="Telemetry">
            <div className="grid grid-cols-2 gap-2">
              {[
                ['Battery', drone?.battery != null ? `${drone.battery.toFixed(0)}%` : '—'],
                ['Altitude', drone ? `${drone.altitude.toFixed(1)} m` : '—'],
                ['Speed', drone ? `${drone.speed.toFixed(1)} m/s` : '—'],
                ['GPS', drone?.gps ?? '—'],
              ].map(([label, value]) => (
                <div key={label} className="rounded border border-white/15 bg-[#3a465f]/75 p-3">
                  <div className="text-[9px] uppercase text-white/55">{label}</div>
                  <div className="mt-3 text-[15px] font-bold text-white">{value}</div>
                </div>
              ))}
            </div>
          </Panel>

          <Panel title="Casualties"
                 action={<span className="text-[9px] text-white/55">{victims.length} found</span>}>
            <div className="mb-2 flex gap-2 text-[9px] uppercase text-white/70">
              {Object.entries(counts).filter(([, value]) => value > 0).map(([priority, value]) => (
                <span key={priority} className="rounded px-2 py-1" style={{ backgroundColor: `${PRIORITY_TONE[priority]}22`, color: PRIORITY_TONE[priority] }}>
                  {priority} {value}
                </span>
              ))}
              {victims.length === 0 && <span className="text-white/45">None detected yet</span>}
            </div>
            <div className="max-h-[150px] overflow-auto">
              {victims.map((victim) => <VictimRow key={victim.id} victim={victim} />)}
            </div>
          </Panel>

          <Panel title="Coverage">
            <div className="text-[34px] font-bold text-white">{mission ? `${mission.coverage.toFixed(0)}%` : '—'}</div>
            <div className="mt-2 h-2 overflow-hidden rounded-full bg-white/10">
              <div className="h-full rounded-full bg-[#8ae0ff]" style={{ width: `${mission?.coverage ?? 0}%` }} />
            </div>
            <p className="mt-3 text-[9px] uppercase leading-4 tracking-[0.12em] text-white/55">
              Measured from where the camera actually looked, not from legs flown.
            </p>
          </Panel>
        </div>
      </div>
    </div>
  )
}

export default DashboardPage
