import { useCallback, useEffect, useState } from 'react'
import { simulationControl } from '../services/apiServices'
import type { SimulationOptions, SimulationStatus } from '../services/apiServices'

const POLL_MS = 4000

export type SimulationPhase = 'unknown' | 'offline' | 'starting' | 'running' | 'stopping'

interface SimulationControl {
  phase: SimulationPhase
  status: SimulationStatus | null
  /** Why the last action failed, e.g. the bridge is not running. */
  error: string | null
  start: (options?: SimulationOptions) => Promise<void>
  stop: () => Promise<void>
}

/**
 * Drives the simulation from the dashboard. The phase stays 'starting' until the bridge reports
 * live processes, so the button cannot be pressed twice into two simulations.
 */
export const useSimulation = (): SimulationControl => {
  const [status, setStatus] = useState<SimulationStatus | null>(null)
  const [phase, setPhase] = useState<SimulationPhase>('unknown')
  const [error, setError] = useState<string | null>(null)

  const refresh = useCallback(async (): Promise<void> => {
    try {
      const next = await simulationControl.status()
      setStatus(next)
      setError(null)
      setPhase((current) => (next.running ? 'running' : current === 'starting' ? 'starting' : 'offline'))
    } catch {
      setStatus(null)
      setError('Dashboard bridge unreachable — run: ros2 run aero_sense_bridge dashboard_bridge')
      setPhase('unknown')
    }
  }, [])

  useEffect(() => {
    void refresh()
    const timer = setInterval(() => void refresh(), POLL_MS)
    return () => clearInterval(timer)
  }, [refresh])

  const start = useCallback(async (options: SimulationOptions = {}) => {
    setPhase('starting')
    try {
      const result = await simulationControl.start(options)
      if (result.started === false) setError(result.reason ?? 'A simulation is already running')
    } catch {
      setError('Could not start the simulation; check the bridge log')
      setPhase('unknown')
    }
  }, [])

  const stop = useCallback(async () => {
    setPhase('stopping')
    try {
      await simulationControl.stop()
    } catch {
      setError('Could not stop the simulation')
    }
    await refresh()
  }, [refresh])

  return { phase, status, error, start, stop }
}
