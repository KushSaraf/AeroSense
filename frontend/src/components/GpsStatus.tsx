import { Satellite, SatelliteDish } from 'lucide-react'
import { simulationControl } from '../services/apiServices'
import type { Drone } from '../types'
import { SimToggle } from './NetworkStatus'

/**
 * What the drone navigates on when its GPS is gone. It reports this itself (DroneStatus), so without
 * a network the ground learns it only on reconnect.
 */
export function GpsBanner({ drone }: { drone: Drone | null | undefined }) {
  if (!drone?.armed) return null
  const vision = drone.navigation === 'VISION'
  // a jammed receiver lets fake 3D fixes through now and then: what the drone flies on decides
  if (!vision && (drone.gps === '3D FIX' || drone.gps === 'UNKNOWN')) return null
  return (
    <div className={`flex items-center gap-3 rounded-lg border px-4 py-2 text-[11px] uppercase tracking-[0.14em] ${
      vision ? 'border-[#ff9800]/50 bg-[#ff9800]/15 text-[#ffe0b2]' : 'border-red-400/50 bg-red-500/15 text-red-100'}`}>
      {vision ? <SatelliteDish size={16} /> : <Satellite size={16} />}
      {vision
        ? "GPS not trusted: navigating on the stereo cameras (OpenVINS). Positions are the drone's own estimate."
        : `GPS ${drone.gps.toLowerCase()} and vision not ready: the autopilot holds on what it has and lands if that fails.`}
    </div>
  )
}

/** Jam or restore the drone's GPS from the dashboard, anywhere in the world. */
export function GpsSwitch({ drone }: { drone: Drone | null | undefined }) {
  const dot = drone?.navigation === 'VISION' ? '#ff9800' : drone?.gps === '3D FIX' ? '#43d17b' : '#8ae0ff'
  return <SimToggle what="GPS" title="Simulate losing GPS, wherever the drone is" dot={dot} act={simulationControl.setGps} />
}
