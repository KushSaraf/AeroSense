import {
  alerts,
  drone,
  hazards,
  mission,
  missions,
  telemetry,
  victims,
} from '../data/mockMissionData'
import type {
  AlertItem,
  Drone,
  Hazard,
  Mission,
  SimulationState,
  TelemetryPoint,
  Victim,
} from '../types'

export const mockDroneService = {
  async getDrone(): Promise<Drone> {
    return drone
  },
  async getMission(): Promise<Mission> {
    return mission
  },
  async getMissions(): Promise<Mission[]> {
    return missions
  },
  async getVictims(): Promise<Victim[]> {
    return victims
  },
  async getHazards(): Promise<Hazard[]> {
    return hazards
  },
  async getTelemetry(): Promise<TelemetryPoint[]> {
    return telemetry
  },
  async getAlerts(): Promise<AlertItem[]> {
    return alerts
  },
  async getSimulationState(): Promise<SimulationState> {
    return {
      gpsLost: false,
      commsLoss: false,
      lowBattery: false,
      sensorFailure: false,
      gazeboConnected: true,
      px4Connected: true,
      rosConnected: true,
      mavLinkConnected: true,
    }
  },
  async toggleGpsLoss(): Promise<void> {
    return
  },
}
