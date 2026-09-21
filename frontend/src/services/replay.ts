/**
 * The public site's data: a real recorded mission, played back through the same API the bridge
 * serves. The simulation cannot run on a web host, so tools/record_replay.py records what the
 * bridge actually served during a flight, and this answers the dashboard's requests from that
 * recording at the replay clock's time. Every page works unchanged, and the site says plainly
 * that it is a replay.
 */
import type { AlertItem, LiveState, MissionEvent } from '../types'

export const IS_REPLAY = import.meta.env.VITE_REPLAY === '1'
export const REPLAY_REFUSAL =
  'This site replays a real recorded flight. Run tools/dashboard.sh on a machine with the simulation to fly one live.'

const BASE = import.meta.env.BASE_URL
const TELEMETRY_SAMPLES = 120
const EVENTS_SHOWN = 40
/** Seconds the finished mission is held on screen before the replay starts again. */
const HOLD_AT_END_S = 10

interface Frame {
  t: number
  state: LiveState
  eventCount: number
  alertIds: string[]
}

type Json = Record<string, unknown>

interface Recording {
  recordedAt: string
  missionId: string
  scenario: string
  durationS: number
  world: Json
  catalogue: Json[]
  frames: Frame[]
  events: MissionEvent[]
  alerts: Record<string, AlertItem>
  cameras: { rgb: number[]; thermal: number[] }
  final: { perception: Json; report: Json; reports: Json[]; missions: Json[] }
}

let recording: Recording | null = null
let loading: Promise<Recording> | null = null
const clock = { base: 0, startedAt: performance.now(), speed: 1, paused: false }

export const loadRecording = (): Promise<Recording> => {
  loading ??= fetch(`${BASE}replay/mission.json`)
    .then((response) => {
      if (!response.ok) throw new Error(`no recording at ${BASE}replay/mission.json`)
      return response.json() as Promise<Recording>
    })
    .then((loaded) => (recording = loaded))
  return loading
}

/** Seconds into the recorded flight that the replay is showing. */
export const replayTime = (): number => {
  if (!recording) return 0
  const running = clock.paused ? 0 : ((performance.now() - clock.startedAt) / 1000) * clock.speed
  const t = (clock.base + running) % (recording.durationS + HOLD_AT_END_S)
  return Math.min(t, recording.durationS)
}

const rebase = () => {
  clock.base = replayTime()
  clock.startedAt = performance.now()
}

export const replayControls = {
  setSpeed: (speed: number) => { rebase(); clock.speed = speed },
  togglePause: () => { rebase(); clock.paused = !clock.paused },
  restart: () => { clock.base = 0; clock.startedAt = performance.now() },
}

export const replayStatus = () => ({
  loaded: recording !== null,
  t: replayTime(),
  durationS: recording?.durationS ?? 0,
  speed: clock.speed,
  paused: clock.paused,
  missionId: recording?.missionId ?? '',
  recordedAt: recording?.recordedAt ?? '',
})

const frameIndexAt = (frames: Frame[], t: number): number => {
  let low = 0
  let high = frames.length - 1
  while (low < high) {
    const middle = Math.ceil((low + high) / 2)
    if (frames[middle].t <= t) low = middle
    else high = middle - 1
  }
  return low
}

const clockLabel = (seconds: number) =>
  `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(Math.floor(seconds % 60)).padStart(2, '0')}`

const stateAt = (rec: Recording, t: number): LiveState => {
  const index = frameIndexAt(rec.frames, t)
  const frame = rec.frames[index]
  // the live bridge keeps two minutes of 1 Hz samples; rebuilt from the recorded drone frames
  const telemetry = rec.frames.slice(Math.max(0, index - TELEMETRY_SAMPLES + 1), index + 1).map((f) => ({
    time: clockLabel(f.t),
    battery: f.state.drone.battery,
    altitude: f.state.drone.altitude,
    speed: f.state.drone.speed,
    gps: 0,
    temperature: 0,
  }))
  const mission = frame.state.mission
    ? { ...frame.state.mission, events: rec.events.slice(0, frame.eventCount).slice(-EVENTS_SHOWN) }
    : null
  return { ...frame.state, mission, telemetry, alerts: [], hazards: [] }
}

const missionsAt = (rec: Recording, state: LiveState): Json[] =>
  rec.catalogue.map((entry) => {
    const live = state.mission
    if (!live || entry.scenario !== rec.scenario) return entry
    const { status, state: machineState, coverage, progress, victimsFound, elapsed, p1, p2, p3, reason } = live
    return { ...entry, status, state: machineState, coverage, progress, victimsFound, elapsed, p1, p2, p3, reason,
             missionId: live.id }
  })

/** The bridge's answer to one request, taken from the recording at the replay clock. */
export const respond = async (method: string, path: string): Promise<unknown> => {
  const rec = await loadRecording()
  if (method !== 'GET') {
    return { success: false, opened: false, started: false, stopped: false,
             reason: REPLAY_REFUSAL, message: REPLAY_REFUSAL }
  }
  const route = path.split('?')[0]
  const state = stateAt(rec, replayTime())
  const frame = rec.frames[frameIndexAt(rec.frames, replayTime())]
  ended(state)
  switch (route) {
    case '/api/state': return state
    case '/api/drone': return state.drone
    case '/api/victims': return state.victims
    case '/api/telemetry': return state.telemetry
    case '/api/mission': return state.mission
    case '/api/hazards': return []
    case '/api/alerts': return frame.alertIds.map((id) => rec.alerts[id]).filter(Boolean)
    case '/api/world': return rec.world
    case '/api/missions': return missionsAt(rec, state)
    // What the detector had found by this moment, not what it finished with: the recording keeps
    // only the final counts, so the per-frame ones are withheld rather than guessed at.
    case '/api/perception': return {
      ...rec.final.perception,
      confirmedVictims: state.victims.length,
      framesProcessed: null,
      rawDetectionsPerFrame: null,
    }
    // As on the bridge, a report exists once its mission has ended: until the replay clock
    // reaches the landing there is nothing to report on, rather than the finished flight's totals.
    case '/api/reports': return ended(state) ? rec.final.reports.filter((entry) => entry.id === rec.missionId) : []
    // nothing is running during a replay, and saying otherwise put 'RUNNING - 0 processes' on screen
    case '/api/simulation': return { running: false, processes: 0, port5760Free: true, replay: true }
    // This flight was recorded before the 3D view existed, so there are no feature points to
    // replay. The shape still has to be the bridge's, or the page reads points off undefined.
    case '/api/scene': return { structures: [], points: [], obstacles: [] }
    case '/api/routes': return { routes: [], teams: [], unassigned: [] }
    case '/api/teams': return { teams: [], unassigned: [] }
    default:
      if (route.startsWith('/api/reports/')) {
        return ended(state) ? rec.final.report
          : { error: `${rec.missionId} is still flying: its report is written when it ends` }
      }
      return { error: `not in the recording: ${route}` }
  }
}

/** Set the first time the replay reaches the landing. A mission that has ended does not un-end
 *  when the replay loops back to take-off, as on the bridge a report outlives the next flight;
 *  without this the report showed for the ~16 s between landing and the loop, and polled at 5 s a
 *  faster replay never showed it at all. */
let endedOnce = false

/** The bridge's contracts.ENDED_STATUSES: a report exists once a mission has ended, however it ended. */
const ended = (state: ReturnType<typeof stateAt>): boolean => {
  if (['COMPLETED', 'EMERGENCY'].includes(state.mission?.status ?? '')) endedOnce = true
  return endedOnce
}

/** The drone's camera at the replay clock: the latest snapshot taken before that moment. */
export const replayCameraSrc = (camera: 'rgb' | 'thermal'): string => {
  if (!recording) return ''
  const times = recording.cameras[camera]
  if (!times.length) return ''
  const t = replayTime()
  const taken = times.filter((time) => time <= t).pop() ?? times[0]
  return `${BASE}replay/frames/${camera}_${String(taken).padStart(4, '0')}.jpg`
}

/** Answer every request to the bridge from the recording; everything else goes out as normal. */
export const installReplayFetch = (apiBase: string) => {
  const realFetch = window.fetch.bind(window)
  window.fetch = async (input: RequestInfo | URL, init?: RequestInit) => {
    const url = typeof input === 'string' ? input : input instanceof URL ? input.href : input.url
    if (!url.startsWith(apiBase)) return realFetch(input, init)
    const body = await respond((init?.method ?? 'GET').toUpperCase(), url.slice(apiBase.length))
    return new Response(JSON.stringify(body), { status: 200, headers: { 'Content-Type': 'application/json' } })
  }
}
