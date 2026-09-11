import { mockDroneService } from './mockServices'
import type { Drone, Mission, Victim, Hazard, AlertItem, TelemetryPoint, SimulationState } from '../types'

export interface DroneDataService {
  getDrone(): Promise<Drone>
  getMission(): Promise<Mission>
}

export interface MissionService {
  getMissions(): Promise<Mission[]>
  getMission(): Promise<Mission>
}

export interface VictimService {
  getVictims(): Promise<Victim[]>
}

export interface HazardService {
  getHazards(): Promise<Hazard[]>
}

export interface TelemetryService {
  getTelemetry(): Promise<TelemetryPoint[]>
}

export interface AlertService {
  getAlerts(): Promise<AlertItem[]>
}

export interface SimulationService {
  getSimulationState(): Promise<SimulationState>
  toggleGpsLoss(): Promise<void>
}

export const services = {
  drone: mockDroneService,
  mission: mockDroneService,
  victims: mockDroneService,
  hazards: mockDroneService,
  telemetry: mockDroneService,
  alerts: mockDroneService,
  simulation: mockDroneService,
}

export const getService = <T>(serviceName: keyof typeof services): T => services[serviceName] as T
