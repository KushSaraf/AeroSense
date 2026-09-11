import { useEffect, useState } from 'react'
import { connectLiveState } from '../services/apiServices'
import { mockService, resolveService } from '../services/serviceFactory'
import type { AlertItem, Drone, Hazard, LiveMission, LiveState, TelemetryPoint, Victim } from '../types'
import type { DataSource } from '../services/serviceFactory'

interface MissionData {
  mission: LiveMission | null
  drone: Drone | null
  victims: Victim[]
  hazards: Hazard[]
  telemetry: TelemetryPoint[]
  alerts: AlertItem[]
  loading: boolean
  /** 'live' when a simulation is serving this data, 'mock' when it is the demonstration set. */
  source: DataSource
}

const empty = {
  mission: null as LiveMission | null,
  drone: null as Drone | null,
  victims: [] as Victim[],
  hazards: [] as Hazard[],
  telemetry: [] as TelemetryPoint[],
  alerts: [] as AlertItem[],
}

export const useMission = (): MissionData => {
  const [data, setData] = useState(empty)
  const [source, setSource] = useState<DataSource>('mock')
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    let cancelled = false
    let disconnect: () => void = () => undefined

    const applyState = (state: LiveState) => {
      if (cancelled) return
      setData({
        mission: state.mission,
        drone: state.drone,
        victims: state.victims,
        hazards: state.hazards,
        telemetry: state.telemetry,
        alerts: state.alerts,
      })
    }

    const load = async () => {
      const { service, source: resolved } = await resolveService()
      if (cancelled) return
      setSource(resolved)

      const [mission, drone, victims, hazards, telemetry, alerts] = await Promise.all([
        service.getMission(),
        service.getDrone(),
        service.getVictims(),
        service.getHazards(),
        service.getTelemetry(),
        service.getAlerts(),
      ])
      if (cancelled) return
      setData({ mission, drone, victims, hazards, telemetry, alerts })
      setLoading(false)

      if (resolved === 'live') {
        // the socket keeps it current; if the simulation stops, fall back rather than freeze
        disconnect = connectLiveState(applyState, () => {
          if (!cancelled) setSource('mock')
        })
      }
    }

    void load().catch(async () => {
      if (cancelled) return
      const [mission, drone, victims, hazards, telemetry, alerts] = await Promise.all([
        mockService.getMission(), mockService.getDrone(), mockService.getVictims(),
        mockService.getHazards(), mockService.getTelemetry(), mockService.getAlerts(),
      ])
      if (cancelled) return
      setData({ mission, drone, victims, hazards, telemetry, alerts })
      setSource('mock')
      setLoading(false)
    })

    return () => {
      cancelled = true
      disconnect()
    }
  }, [])

  return { ...data, loading, source }
}
