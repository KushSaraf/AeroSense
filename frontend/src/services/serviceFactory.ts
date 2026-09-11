/**
 * Which data the dashboard shows: the running simulation when one is reachable, otherwise the
 * mock tour. The choice is made once at startup and reported to the UI, so an operator can
 * always tell whether they are looking at a live mission or a demonstration.
 */
import { apiService, isLiveAvailable } from './apiServices'
import { mockDroneService } from './mockServices'
import type {
  AlertItem, Drone, Hazard, Mission, SimulationState, TelemetryPoint, Victim,
} from '../types'

export type DataSource = 'live' | 'mock'

export interface MissionDataService {
  getDrone(): Promise<Drone>
  getMission(): Promise<Mission>
  getMissions(): Promise<Mission[]>
  getVictims(): Promise<Victim[]>
  getHazards(): Promise<Hazard[]>
  getTelemetry(): Promise<TelemetryPoint[]>
  getAlerts(): Promise<AlertItem[]>
  getSimulationState(): Promise<SimulationState>
  toggleGpsLoss(): Promise<void>
}

export const resolveService = async (): Promise<{ service: MissionDataService; source: DataSource }> =>
  (await isLiveAvailable())
    ? { service: apiService as unknown as MissionDataService, source: 'live' }
    : { service: mockDroneService as unknown as MissionDataService, source: 'mock' }

export const mockService = mockDroneService as unknown as MissionDataService
