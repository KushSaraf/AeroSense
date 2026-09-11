import { BrowserRouter, Navigate, Route, Routes } from 'react-router-dom'
import AppShell from './layouts/AppShell'
import HomePage from './pages/HomePage'
import DashboardPage from './pages/DashboardPage'
import MissionsPage from './pages/MissionsPage'
import NewMissionPage from './pages/NewMissionPage'
import MapPage from './pages/MapPage'
import HazardsPage from './pages/HazardsPage'
import AlertsPage from './pages/AlertsPage'
import AiPerceptionPage from './pages/AiPerceptionPage'
import NavigationPage from './pages/NavigationPage'
import CommunicationPage from './pages/CommunicationPage'
import TelemetryPage from './pages/TelemetryPage'
import ReportsPage from './pages/ReportsPage'
import ReplayPage from './pages/ReplayPage'
import SimulationPage from './pages/SimulationPage'
import SettingsPage from './pages/SettingsPage'

function App() {
  return (
    <BrowserRouter>
      <AppShell>
        <Routes>
          <Route path="/" element={<HomePage />} />
          <Route path="/dashboard" element={<DashboardPage />} />
          <Route path="/dashboard/missions/:missionId" element={<DashboardPage />} />
          <Route path="/dashboard/map" element={<MapPage />} />
          <Route path="/dashboard/missions" element={<MissionsPage />} />
          <Route path="/dashboard/missions/new" element={<NewMissionPage />} />
          <Route path="/dashboard/live-feed" element={<Navigate to="/dashboard" replace />} />
          <Route path="/dashboard/victims" element={<Navigate to="/dashboard" replace />} />
          <Route path="/dashboard/hazards" element={<HazardsPage />} />
          <Route path="/dashboard/safe-routes" element={<Navigate to="/dashboard" replace />} />
          <Route path="/dashboard/alerts" element={<AlertsPage />} />
          <Route path="/dashboard/ai" element={<AiPerceptionPage />} />
          <Route path="/dashboard/navigation" element={<NavigationPage />} />
          <Route path="/dashboard/communication" element={<CommunicationPage />} />
          <Route path="/dashboard/telemetry" element={<TelemetryPage />} />
          <Route path="/dashboard/reports" element={<ReportsPage />} />
          <Route path="/dashboard/replay" element={<ReplayPage />} />
          <Route path="/dashboard/simulation" element={<SimulationPage />} />
          <Route path="/dashboard/settings" element={<SettingsPage />} />
        </Routes>
      </AppShell>
    </BrowserRouter>
  )
}

export default App
