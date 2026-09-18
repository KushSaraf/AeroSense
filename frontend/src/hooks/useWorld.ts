import { useApi } from './useApi'

export interface LatLon {
  latitude: number
  longitude: number
}

export interface WorldSector {
  id: string
  name: string
  bounds: { minX: number; minY: number; maxX: number; maxY: number }
  corners: LatLon[]
  centre: LatLon
}

export interface WorldInfo {
  world: string
  origin: LatLon & { elevation: number }
  metresPerDegree: { latitude: number; longitude: number }
  sectors: WorldSector[]
  /** Where the drone has no network; absent in older recordings. */
  noNetworkZones?: WorldSector[]
  /** Where the drone's GPS is jammed (a zone in both lists loses both); absent in older recordings. */
  noGpsZones?: WorldSector[]
}

const WORLD_POLL_MS = 60000

/** Where the simulated sectors sit on Earth. Static for a given world, so it is polled rarely. */
export const useWorld = () => useApi<WorldInfo>('/api/world', WORLD_POLL_MS)
