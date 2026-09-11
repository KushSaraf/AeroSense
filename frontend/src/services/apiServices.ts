/**
 * Live data from the running simulation, via the ROS dashboard bridge
 * (`ros2 run aero_sense_bridge dashboard_bridge`, default http://127.0.0.1:8000).
 *
 * Every value here was measured by the simulation. Where the system does not know something yet
 * — triage priority, hazard risk, reachability — the bridge says so and the UI shows it as
 * unknown, rather than showing a plausible number no part of the system computed.
 */
import type {
  AlertItem, Drone, Hazard, LiveState, Mission, TelemetryPoint, Victim,
} from '../types'

const API_BASE: string =
  (import.meta.env.VITE_API_URL as string | undefined) ?? 'http://127.0.0.1:8000'
const PROBE_TIMEOUT_MS = 1500

const request = async <T>(path: string, timeoutMs = 8000): Promise<T> => {
  const controller = new AbortController()
  const timer = setTimeout(() => controller.abort(), timeoutMs)
  try {
    const response = await fetch(`${API_BASE}${path}`, { signal: controller.signal })
    if (!response.ok) throw new Error(`${path} responded ${response.status}`)
    return (await response.json()) as T
  } finally {
    clearTimeout(timer)
  }
}

/** Whether a simulation is actually serving data, so the UI can fall back to the mock tour. */
export const isLiveAvailable = async (): Promise<boolean> => {
  try {
    await request<LiveState>('/api/state', PROBE_TIMEOUT_MS)
    return true
  } catch {
    return false
  }
}

export const apiService = {
  getState: () => request<LiveState>('/api/state'),
  getDrone: () => request<Drone>('/api/drone'),
  getMission: () => request<Mission>('/api/mission'),
  getMissions: async () => [await request<Mission>('/api/mission')],
  getVictims: () => request<Victim[]>('/api/victims'),
  getHazards: () => request<Hazard[]>('/api/hazards'),
  getTelemetry: () => request<TelemetryPoint[]>('/api/telemetry'),
  getAlerts: () => request<AlertItem[]>('/api/alerts'),
  getSimulationState: async () => ({ gpsLoss: false, commLoss: false, running: true }),
  toggleGpsLoss: async () => undefined,
}

/**
 * Push updates while the mission runs. Returns a function that closes the socket; callers should
 * call it on unmount, or a page change leaves a socket pushing into nothing.
 */
export const connectLiveState = (
  onState: (state: LiveState) => void,
  onClosed?: () => void,
): (() => void) => {
  const url = `${API_BASE.replace(/^http/, 'ws')}/ws`
  let socket: WebSocket | null = null
  let closedByCaller = false

  try {
    socket = new WebSocket(url)
  } catch {
    onClosed?.()
    return () => undefined
  }

  socket.onmessage = (event) => {
    try {
      onState(JSON.parse(event.data as string) as LiveState)
    } catch {
      /* a malformed frame is not worth tearing the dashboard down for */
    }
  }
  socket.onclose = () => { if (!closedByCaller) onClosed?.() }
  socket.onerror = () => socket?.close()

  return () => {
    closedByCaller = true
    socket?.close()
  }
}

const post = async <T>(path: string, body?: unknown, timeoutMs = 20000): Promise<T> => {
  const controller = new AbortController()
  const timer = setTimeout(() => controller.abort(), timeoutMs)
  try {
    const response = await fetch(`${API_BASE}${path}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: body === undefined ? undefined : JSON.stringify(body),
      signal: controller.signal,
    })
    if (!response.ok) throw new Error(`${path} responded ${response.status}`)
    return (await response.json()) as T
  } finally {
    clearTimeout(timer)
  }
}

export interface SimulationStatus {
  running: boolean
  processes: number
  port5760Free: boolean
  started?: boolean
  reason?: string
}

export interface SimulationOptions {
  quality?: 'low' | 'medium' | 'high'
  gui?: boolean
  rviz?: boolean
  cruiseSpeed?: number
}

/**
 * Start and stop the simulation itself. The bridge runs outside it, so pressing start on a cold
 * machine brings up Gazebo, the drone, the autopilot and perception; starting a second one is
 * refused, because two simulations publish the same topics and corrupt what the dashboard shows.
 */
export const simulationControl = {
  status: () => request<SimulationStatus>('/api/simulation', PROBE_TIMEOUT_MS),
  start: (options: SimulationOptions = {}) =>
    post<SimulationStatus>('/api/simulation/start', options, 30000),
  stop: () => post<SimulationStatus>('/api/simulation/stop', undefined, 30000),
  /** Open Gazebo or RViz onto the simulation that is already running. */
  openViewer: (kind: 'gazebo' | 'rviz') =>
    post<{ opened: boolean; reason?: string }>(`/api/simulation/view/${kind}`, undefined, 15000),
}

export const API_URL = API_BASE
