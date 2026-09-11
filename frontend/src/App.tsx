import { BrowserRouter, Navigate, Route, Routes } from 'react-router-dom'
import AppShell from './layouts/AppShell'
import HomePage from './pages/HomePage'
import DashboardPage from './pages/DashboardPage'
import MissionsPage from './pages/MissionsPage'
import MapPage from './pages/MapPage'
import AlertsPage from './pages/AlertsPage'
import AiPerceptionPage from './pages/AiPerceptionPage'
import TelemetryPage from './pages/TelemetryPage'
import ReportsPage from './pages/ReportsPage'
import SettingsPage from './pages/SettingsPage'

/**
 * Every route here is served by something real. Pages whose data would have to be invented —
 * hazard intelligence, comms, replay, safe routes — are gone rather than mocked, and their old
 * links redirect to the dashboard so a bookmark does not land on nothing.
 */
function App() {
  return (
    <BrowserRouter basename={import.meta.env.BASE_URL.replace(/\/$/, '')}>
      <AppShell>
        <Routes>
          <Route path="/" element={<HomePage />} />
          <Route path="/dashboard" element={<DashboardPage />} />
          <Route path="/dashboard/missions" element={<MissionsPage />} />
          <Route path="/dashboard/missions/:missionId" element={<DashboardPage />} />
          <Route path="/dashboard/map" element={<MapPage />} />
          <Route path="/dashboard/alerts" element={<AlertsPage />} />
          <Route path="/dashboard/ai" element={<AiPerceptionPage />} />
          <Route path="/dashboard/telemetry" element={<TelemetryPage />} />
          <Route path="/dashboard/reports" element={<ReportsPage />} />
          <Route path="/dashboard/settings" element={<SettingsPage />} />
          <Route path="*" element={<Navigate to="/dashboard" replace />} />
        </Routes>
      </AppShell>
    </BrowserRouter>
  )
}

export default App
