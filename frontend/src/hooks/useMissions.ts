import { useCallback, useEffect, useState } from 'react'
import { API_URL, simulationControl } from '../services/apiServices'

/** A sector that can be flown, and the state of the run if it is being flown now. */
export interface MissionCard {
  id: string
  scenario: string
  name: string
  disasterType: string
  status: 'READY' | 'ACTIVE' | 'COMPLETED' | string
  state?: string
  reason?: string
  areaKm2: number
  bounds: { minX: number; minY: number; maxX: number; maxY: number }
  coverage: number
  progress: number
  victimsFound: number
  missionId?: string
  elapsed?: string
  p1?: number
  p2?: number
  p3?: number
  owner: string
  type: string
}

const POLL_MS = 2500
const SIMULATION_READY_TIMEOUT_MS = 240000

interface MissionsState {
  missions: MissionCard[]
  active: MissionCard | null
  busy: string | null
  error: string | null
  /** Starts a sector: brings the simulation up first if it is not already flying. */
  startMission: (scenario: string) => Promise<void>
  abortMission: () => Promise<void>
}

const sleep = (ms: number) => new Promise((resolve) => setTimeout(resolve, ms))

export const useMissions = (): MissionsState => {
  const [missions, setMissions] = useState<MissionCard[]>([])
  const [busy, setBusy] = useState<string | null>(null)
  const [error, setError] = useState<string | null>(null)

  const refresh = useCallback(async () => {
    try {
      const response = await fetch(`${API_URL}/api/missions`)
      if (!response.ok) throw new Error(String(response.status))
      setMissions((await response.json()) as MissionCard[])
      setError(null)
    } catch {
      setMissions([])
      setError('Dashboard bridge unreachable — run: ros2 run aero_sense_bridge dashboard_bridge')
    }
  }, [])

  useEffect(() => {
    void refresh()
    const timer = setInterval(() => void refresh(), POLL_MS)
    return () => clearInterval(timer)
  }, [refresh])

  const startMission = useCallback(async (scenario: string) => {
    setBusy(scenario)
    setError(null)
    try {
      // a mission needs a simulation; bring one up and wait for the drone before commanding it
      const status = await simulationControl.status()
      if (!status.running) {
        await simulationControl.start({ quality: 'low', gui: true })
        const deadline = Date.now() + SIMULATION_READY_TIMEOUT_MS
        for (;;) {
          await sleep(4000)
          const state = await fetch(`${API_URL}/api/state`).then((r) => r.json())
          if (state.connected) break
          if (Date.now() > deadline) throw new Error('the simulation did not come up in time')
        }
      }
      const started = await fetch(`${API_URL}/api/missions/${scenario}/start`, { method: 'POST' })
        .then((r) => r.json())
      if (!started.success) setError(started.message ?? 'the mission manager refused')
      await refresh()
    } catch (cause: unknown) {
      setError(cause instanceof Error ? cause.message : 'could not start the mission')
    } finally {
      setBusy(null)
    }
  }, [refresh])

  const abortMission = useCallback(async () => {
    setBusy('abort')
    try {
      await fetch(`${API_URL}/api/mission/abort`, { method: 'POST' })
      await refresh()
    } finally {
      setBusy(null)
    }
  }, [refresh])

  const active = missions.find((mission) => mission.status === 'ACTIVE') ?? null
  return { missions, active, busy, error, startMission, abortMission }
}
