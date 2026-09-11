import { useEffect, useState } from 'react'
import { mockDroneService } from '../services/mockServices'
import type { Mission, AlertItem, Drone, Hazard, TelemetryPoint, Victim } from '../types'

export const useMission = () => {
  const [mission, setMission] = useState<Mission | null>(null)
  const [drone, setDrone] = useState<Drone | null>(null)
  const [victims, setVictims] = useState<Victim[]>([])
  const [hazards, setHazards] = useState<Hazard[]>([])
  const [telemetry, setTelemetry] = useState<TelemetryPoint[]>([])
  const [alerts, setAlerts] = useState<AlertItem[]>([])
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    const load = async () => {
      const [missionData, droneData, victimData, hazardData, telemetryData, alertData] = await Promise.all([
        mockDroneService.getMission(),
        mockDroneService.getDrone(),
        mockDroneService.getVictims(),
        mockDroneService.getHazards(),
        mockDroneService.getTelemetry(),
        mockDroneService.getAlerts(),
      ])

      setMission(missionData)
      setDrone(droneData)
      setVictims(victimData)
      setHazards(hazardData)
      setTelemetry(telemetryData)
      setAlerts(alertData)
      setLoading(false)
    }

    void load()
  }, [])

  return { mission, drone, victims, hazards, telemetry, alerts, loading }
}
