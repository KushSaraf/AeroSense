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

/** A hazard region the drone mapped: disaster segmentation (SegFormer-B0) banded by HSI (hazard_mapper). */
export interface Hazard {
  id: string
  type: 'STRUCTURAL' | 'FLOOD' | 'DEBRIS'
  severity: 'CRITICAL' | 'HIGH' | 'MODERATE'
  /** the region's mean Hazard Severity Index, 0..1 */
  hsi: number
  areaM2: number
  latitude: number
  longitude: number
  /** outline, [lat, lon] */
  polygon: [number, number][]
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
  /** What the autopilot flies on: OpenVINS once it has taken over from a lost GPS. Absent in older recordings. */
  navigation?: 'GPS' | 'VISION' | 'NONE' | 'UNKNOWN'
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

/** A ground team's road route from the command base to one casualty (ground_routes). */
export interface GroundRoute {
  victimId: string
  reachable: boolean
  /** worst hazard crossed: SAFE | MODERATE | HIGH; CRITICAL when no route is open */
  risk: string
  distanceM: number
  timeS: number
  path: [number, number][]
}

/** Which ground team goes to whom, in order, and who no team reaches within its shift (team_plan). */
export interface TeamTour {
  team: string
  victimIds: string[]
  distanceM: number
  /** travel, time on site with each casualty, and back to base */
  timeS: number
  path: [number, number][]
}

export interface TeamPlan {
  tours: TeamTour[]
  unassigned: string[]
}

/** The 3D view's slow layers (/api/scene), in the map frame (metres, ENU). */
export interface Scene3d {
  structures: { name: string; kind: string; x: number; y: number; radiusM: number; heightM: number }[]
  /** OpenVINS's feature points, [x, y, z] */
  points: [number, number, number][]
  /** Where the avoidance beams hit something, [x, y, z]; absent in recordings made before them. */
  obstacles?: [number, number, number][]
}

export interface LiveState {
  connected: boolean
  /** Absent in recordings made before the link model. */
  link?: LinkStatus
  drone: Drone
  mission: LiveMission | null
  victims: Victim[]
  /** Absent in recordings made before ground routing. */
  routes?: GroundRoute[]
  /** Absent in recordings made before team planning. */
  teams?: TeamPlan
  hazards: Hazard[]
  alerts: AlertItem[]
  telemetry: TelemetryPoint[]
}
