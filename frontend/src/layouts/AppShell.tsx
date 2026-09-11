import { useEffect, useState } from 'react'
import { NavLink, useLocation, useNavigate } from 'react-router-dom'
import { useDataSource } from '../hooks/useDataSource'
import {
  Activity,
  Bell,
  Cpu,
  FileText,
  Gauge,
  Home,
  Map,
  Menu,
  Radar,
  Search,
  Settings,
} from 'lucide-react'
import type { ReactNode } from 'react'

/**
 * Two levels, because they answer different questions. Command items are what an operator needs
 * with or without a mission running; mission-session items only mean anything while one is in
 * progress, and reading them as peers of "Reports" made the sidebar a list of thirteen equals.
 */
const commandNav = [
  { label: 'COMMAND HOME', to: '/', icon: Home },
  { label: 'MISSION COMMAND', to: '/dashboard/missions', icon: Radar },
  { label: 'LIVE MAP', to: '/dashboard/map', icon: Map },
  { label: 'ALERT CENTER', to: '/dashboard/alerts', icon: Bell },
  { label: 'REPORTS', to: '/dashboard/reports', icon: FileText },
  { label: 'SYSTEM SETTINGS', to: '/dashboard/settings', icon: Settings },
]

const missionNav = [
  { label: 'LIVE DASHBOARD', to: '/dashboard', icon: Gauge },
  { label: 'AI PERCEPTION', to: '/dashboard/ai', icon: Cpu },
  { label: 'TELEMETRY', to: '/dashboard/telemetry', icon: Activity },
]



const pageTitles: Record<string, string> = {
  '/': 'Command Home',
  '/dashboard': 'Mission Dashboard',
  '/dashboard/map': 'Live Map',
  '/dashboard/missions': 'Mission Command',
  '/dashboard/alerts': 'Alert Center',
  '/dashboard/ai': 'AI Perception',
  '/dashboard/telemetry': 'Telemetry',
  '/dashboard/reports': 'Mission Reports',
  '/dashboard/settings': 'System Settings',
}

function AppShell({ children }: { children: ReactNode }) {
  const [collapsed, setCollapsed] = useState(true)
  const [searchOpen, setSearchOpen] = useState(false)
  const [statusOpen, setStatusOpen] = useState(false)
  const [currentTime, setCurrentTime] = useState(new Date())
  const navigate = useNavigate()
  const dataSource = useDataSource()
  const location = useLocation()

  useEffect(() => {
    const pageTitle = pageTitles[location.pathname] ?? 'Command Center'
    document.title = `Aero Sense | ${pageTitle}`
  }, [location.pathname])

  useEffect(() => {
    const clock = window.setInterval(() => setCurrentTime(new Date()), 1000)
    return () => window.clearInterval(clock)
  }, [])

  const formattedTime = currentTime.toLocaleTimeString([], {
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
    hour12: false,
  })

  return (
    <div className="app-shell h-screen overflow-hidden">
      <aside className="flex w-[88px] flex-col border-r border-white/10 bg-[#202a40]/90 backdrop-blur-sm transition-all duration-300" style={{ width: collapsed ? 88 : 252 }}>
        <div className="flex items-center justify-center border-b border-white/10 px-3 py-4">
          <button type="button" onClick={() => setCollapsed((value) => !value)} className="flex h-10 w-10 items-center justify-center text-text transition hover:bg-white/10" aria-label="Toggle menu">
            <Menu size={28} strokeWidth={2} />
          </button>
        </div>

        <nav className="flex flex-1 flex-col gap-1 overflow-y-auto p-2 pt-3">
          {commandNav.map(({ label, to, icon: Icon }) => (
            <NavLink
              key={label}
              to={to}
              title={label}
              className={({ isActive }) => [
                'flex items-center gap-3 rounded-lg border px-2 py-3 text-[10px] font-medium tracking-[0.16em] text-text/80 transition',
                isActive ? 'border-white/20 bg-[#6f7e9b]/45 text-text' : 'border-transparent bg-transparent hover:border-white/10 hover:bg-white/5',
                collapsed ? 'justify-center px-1' : '',
              ].join(' ')}
            >
              <Icon size={25} strokeWidth={1.8} />
              {!collapsed && <span>{label}</span>}
            </NavLink>
          ))}

          {!collapsed && (
            <div className="mt-4 px-3 pb-1 text-[9px] tracking-[0.22em] text-text/45">
              MISSION SESSION
            </div>
          )}
          {missionNav.map(({ label, to, icon: Icon }) => (
            <NavLink
              key={label}
              to={to}
              title={label}
              end={to === '/dashboard'}
              className={({ isActive }) =>
                `flex items-center gap-3 px-3 py-2 text-[11px] tracking-[0.14em] transition ${collapsed ? 'justify-center' : ''} ${
                  isActive ? 'bg-white/15 text-white' : 'text-text/75 hover:bg-white/10 hover:text-white'
                }`
              }
            >
              <Icon size={16} />
              {!collapsed && <span>{label}</span>}
            </NavLink>
          ))}

        </nav>

        <div className="border-t border-white/10 p-3 text-center text-[10px] tracking-[0.18em] text-text/60">
          {!collapsed && (
            <>
              <div>AERO</div>
              <div>SENSE</div>
              <div className="mt-2 text-[9px]">v1.0</div>
            </>
          )}
        </div>
      </aside>

      <div className="flex min-h-0 min-w-0 flex-1 flex-col">
        <header className="relative flex h-[74px] items-center justify-between border-b border-white/10 bg-[#202a40]/95 px-5 text-text backdrop-blur-sm">
          <div className="flex items-center gap-4">
            <img src="/as-logo.png" alt="AS" className="h-12 w-12 object-contain" />
            <div>
              <div className="text-[20px] font-semibold tracking-[0.2em] uppercase">AERO SENSE</div>
              <div className="hidden items-center gap-2 text-[11px] tracking-[0.2em] text-text/70 md:flex">
                <span>DETECT</span><span>|</span><span>LOCATE</span><span>|</span><span>ASSESS</span><span>|</span><span>GUIDE</span><span>|</span><span>SAVE LIVES</span>
              </div>
            </div>
          </div>

          <div className="flex items-center gap-4 text-[10px] tracking-[0.22em] uppercase text-text/85">
            <button type="button" onClick={() => setSearchOpen((value) => !value)} className="flex items-center gap-2 px-2 py-2 text-[13px] transition hover:text-white" aria-expanded={searchOpen}>
              <Search size={18} /> + SEARCH
            </button>
            <button type="button" onClick={() => setStatusOpen((value) => !value)} className="px-2 py-2 text-text/70 transition hover:text-white" aria-expanded={statusOpen}>STATUS</button>
            <button type="button" onClick={() => navigate('/dashboard/alerts')} className="px-2 py-2 text-text/70 transition hover:text-white">ALERT</button>
            <span
              title={dataSource === 'live'
                ? 'Live data from the running simulation'
                : 'No simulation is serving the dashboard bridge, so nothing is shown'}
              className={`rounded px-2 py-1 text-[10px] tracking-[0.18em] ${
                dataSource === 'live'
                  ? 'bg-emerald-500/20 text-emerald-300'
                  : 'bg-amber-500/20 text-amber-300'
              }`}
            >
              {dataSource === 'live' ? 'LIVE' : 'NO SIMULATION'}
            </span>
            <time className="ml-1 border-l border-white/10 pl-4 font-mono text-[16px] tracking-[0.12em] text-text" dateTime={currentTime.toISOString()}>{formattedTime}</time>
          </div>

          {searchOpen && (
            <div className="absolute right-[190px] top-[66px] z-50 w-72 rounded-lg border border-white/15 bg-[#27334a] p-3 shadow-2xl">
              <label className="flex items-center gap-2 rounded border border-white/15 bg-[#1c2639] px-3 py-2 text-[11px] tracking-[0.12em] text-white/60">
                <Search size={15} />
                <input autoFocus placeholder="SEARCH MISSIONS, DRONES..." className="min-w-0 flex-1 bg-transparent text-[10px] uppercase tracking-[0.12em] text-white outline-none placeholder:text-white/40" />
              </label>
              <button type="button" onClick={() => { navigate('/dashboard/missions'); setSearchOpen(false) }} className="mt-2 w-full px-2 py-2 text-left text-[10px] uppercase tracking-[0.14em] text-white/75 transition hover:bg-white/10">Open mission command</button>
              <button type="button" onClick={() => { navigate('/dashboard/map'); setSearchOpen(false) }} className="w-full px-2 py-2 text-left text-[10px] uppercase tracking-[0.14em] text-white/75 transition hover:bg-white/10">Open live map</button>
            </div>
          )}

          {statusOpen && (
            <div className="absolute right-[115px] top-[66px] z-50 w-52 rounded-lg border border-white/15 bg-[#27334a] p-3 shadow-2xl">
              <div className="mb-3 text-[10px] uppercase tracking-[0.16em] text-white/50">System status</div>
              <div className="flex items-center justify-between text-[10px] uppercase tracking-[0.12em] text-white/80">
                <span>Bridge</span>
                <span className={dataSource === 'live' ? 'text-[#86e2a4]' : 'text-amber-300'}>
                  {dataSource === 'live' ? 'ONLINE' : 'OFFLINE'}
                </span>
              </div>
              <button type="button" onClick={() => { navigate('/dashboard/settings'); setStatusOpen(false) }}
                      className="mt-2 w-full px-2 py-2 text-left text-[10px] uppercase tracking-[0.12em] text-white/70 transition hover:bg-white/10">
                Open system settings
              </button>
            </div>
          )}
        </header>

        <main className="app-main min-h-0 flex-1 overflow-auto bg-[#202635]">{children}</main>
      </div>
    </div>
  )
}

export default AppShell
