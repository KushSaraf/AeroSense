export type Priority = 'P1' | 'P2' | 'P3'
/** A casualty perception has found but triage has not yet scored. */
export type VictimPriority = Priority | 'UNTRIAGED'
/** The system genuinely does not know this yet: shown as such rather than guessed. */
export type Unknown = 'unknown'
export type Severity = 'CRITICAL' | 'HIGH' | 'MODERATE' | 'CLEARED'
export type MissionStatus = 'ACTIVE' | 'ETA' | 'DONE' | 'COMPLETED' | 'STANDBY'
export type AlertType = 'SURVIVOR' | 'HAZARD' | 'SYSTEM'

export interface Victim {
  id: string
  priority: VictimPriority
  confidence: number
  latitude: number
  longitude: number
  /** Metres in the simulation's map frame, for the 3-D and local views. */
  position?: { x: number; y: number; z: number }
  thermalStrength: 'strong' | 'medium' | 'weak'
  movement: 'moving' | 'waving' | 'static' | Unknown
  hazardRisk: 'low' | 'medium' | 'high' | Unknown
  accessibility: 'accessible' | 'restricted' | 'blocked' | Unknown
  status: 'PENDING' | 'ROUTE' | 'RESCUED'
  rationale: string
  location: string
  timestamp: string
}

export interface Hazard {
  id: string
  type: 'FIRE' | 'FLOOD' | 'STRUCTURAL' | 'SMOKE' | 'ELECTRICAL' | 'CHEMICAL' | 'LANDSLIDE' | 'DEBRIS'
  severity: Severity
  confidence: number
  latitude: number
  longitude: number
  radius: number
  detected: string
  spread: string
  nearbyVictims: number
  coords: Array<[number, number]>
}

export interface Drone {
  id: string
  name: string
  model: string
  status: 'ACTIVE' | 'STANDBY' | 'RETURNING'
  /** Autopilot flight mode, e.g. GUIDED, LAND, RTL. */
  mode?: string
  armed?: boolean
  /** Null until the autopilot has measured it: shown as unknown, never as a number. */
  battery: number | null
  altitude: number
  speed: number
  /** The link state as the ground knows it; '5G STRONG' only in recordings made before the link model. */
  link: LinkState | '5G STRONG'
  gps: '3D FIX' | 'LOST' | 'DEGRADED' | 'UNKNOWN'
  /** Absent until the autopilot has a fix: the map shows nothing rather than a guessed position. */
  latitude?: number | null
  longitude?: number | null
  position?: { x: number; y: number; z: number } | null
  flightTime: string
  autonomy: string
}

export interface Mission {
  id: string
  name: string
  disasterType: string
  status: MissionStatus
  owner: string
  date: string
  type: string
  coverage: number
  progress: number
  priority: 'HIGH' | 'MEDIUM' | 'LOW'
  victimsFound?: number
}

export interface AlertItem {
  id: string
  priority: Priority | 'SYSTEM'
  title: string
  location: string
  timestamp: string
  severity: Severity | 'SYSTEM'
  rationale: string
  confidence: number
  type: AlertType
}

export interface TelemetryPoint {
  time: string
  /** Null for samples taken before the autopilot measured it. */
  battery: number | null
  altitude: number
  speed: number
  gps: number
  temperature: number
}

export interface SimulationState {
  gpsLost: boolean
  commsLoss: boolean
  lowBattery: boolean
  sensorFailure: boolean
  gazeboConnected: boolean
  px4Connected: boolean
  rosConnected: boolean
  mavLinkConnected: boolean
}

export interface RouteInfo {
  id: string
  target: string
  distance: number
  eta: string
  risk: 'LOW' | 'MEDIUM' | 'HIGH'
  alternatives: number
  path: Array<[number, number]>
}

export interface MapLayerState {
  satellite: boolean
  terrain: boolean
  orthomosaic: boolean
  hazards: boolean
  victims: boolean
  dronePath: boolean
  coverage: boolean
  safeRoutes: boolean
  sensor3d: boolean
}

/** One consistent frame of the mission, as the dashboard bridge sends it. */
/** A mission as the state machine reports it: the plan, plus where the flight has got to. */
export type LiveMission = Mission & {
  scenario?: string
  state?: string
  reason?: string
  elapsed?: string
  elapsedSeconds?: number
  p1?: number
  p2?: number
  p3?: number
  events?: MissionEvent[]
}

/** One entry in the mission's event log, as the state machine emitted it. */
export interface MissionEvent {
  time: string
  text: string
  /** Seconds the drone held this on board while it had no network. */
  heldS?: number
}

export type LinkState = 'CONNECTED' | 'DEGRADED' | 'OFFLINE' | 'UNKNOWN'

/** The drone's network as the ground knows it: during an outage it hears nothing, so what is held is unknown. */
export interface LinkStatus {
  state: LinkState
  quality: number | null
  silentSeconds: number | null
  queued: { P1: number; P2: number; P3: number } | null
}

export interface LiveState {
  connected: boolean
  /** Absent in recordings made before the link model. */
  link?: LinkStatus
  drone: Drone
  mission: LiveMission | null
  victims: Victim[]
  hazards: Hazard[]
  alerts: AlertItem[]
  telemetry: TelemetryPoint[]
}
