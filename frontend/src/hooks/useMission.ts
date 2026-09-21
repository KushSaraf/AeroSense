import { useEffect, useState } from 'react'
import { apiService, connectLiveState, isLiveAvailable } from '../services/apiServices'
import type { AlertItem, Drone, GasReading, GroundRoute, Hazard, LinkStatus, LiveMission, LiveState, TeamPlan, TelemetryPoint, Victim } from '../types'

/** Where the dashboard's numbers come from. There is no third option: it is live, or it is nothing. */
export type DataSource = 'live' | 'offline'

interface MissionData {
  mission: LiveMission | null
  drone: Drone | null
  victims: Victim[]
  routes: GroundRoute[]
  teams: TeamPlan
  hazards: Hazard[]
  telemetry: TelemetryPoint[]
  alerts: AlertItem[]
  link: LinkStatus | null
  gas: GasReading
  loading: boolean
  source: DataSource
}

const EMPTY = {
  mission: null as LiveMission | null,
  drone: null as Drone | null,
  victims: [] as Victim[],
  routes: [] as GroundRoute[],
  teams: { tours: [], unassigned: [] } as TeamPlan,
  hazards: [] as Hazard[],
  telemetry: [] as TelemetryPoint[],
  alerts: [] as AlertItem[],
  link: null as LinkStatus | null,
  gas: {} as GasReading,
}
const RETRY_MS = 5000

/**
 * One consistent frame of the running mission, pushed over the bridge's WebSocket.
 *
 * When no simulation is serving, this reports nothing and says so. It used to fall back to a
 * demonstration dataset, which meant the dashboard could show casualties that were never
 * detected — the one failure mode a search-and-rescue display must not have.
 */
export const useMission = (): MissionData => {
  const [data, setData] = useState(EMPTY)
  const [source, setSource] = useState<DataSource>('offline')
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    let cancelled = false
    let disconnect: () => void = () => undefined
    let retry: ReturnType<typeof setTimeout> | undefined

    const goOffline = () => {
      if (cancelled) return
      setData(EMPTY)
      setSource('offline')
      setLoading(false)
      retry = setTimeout(() => void connect(), RETRY_MS)
    }

    const connect = async () => {
      if (cancelled) return
      if (!(await isLiveAvailable())) {
        goOffline()
        return
      }
      try {
        const [mission, drone, victims, hazards, telemetry, alerts] = await Promise.all([
          apiService.getMission(), apiService.getDrone(), apiService.getVictims(),
          apiService.getHazards(), apiService.getTelemetry(), apiService.getAlerts(),
        ])
        if (cancelled) return
        setData({ mission: mission as LiveMission, drone, victims, routes: [], teams: EMPTY.teams, hazards, telemetry, alerts, link: null, gas: EMPTY.gas })
        setSource('live')
        setLoading(false)
        disconnect = connectLiveState((state: LiveState) => {
          if (cancelled) return
          setData({
            mission: state.mission, drone: state.drone, victims: state.victims, routes: state.routes ?? [],
            teams: state.teams ?? EMPTY.teams,
            hazards: state.hazards, telemetry: state.telemetry, alerts: state.alerts,
            link: state.link ?? null,
            gas: state.gas ?? EMPTY.gas,
          })
        }, goOffline)
      } catch {
        goOffline()
      }
    }

    void connect()
    return () => {
      cancelled = true
      if (retry) clearTimeout(retry)
      disconnect()
    }
  }, [])

  return { ...data, loading, source }
}
