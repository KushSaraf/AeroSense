import { useEffect, useState } from 'react'
import { NavLink, useLocation, useNavigate } from 'react-router-dom'
import {
  Bell,
  Binary,
  Compass,
  Cpu,
  Crosshair,
  Gauge,
  Grid2x2,
  Home,
  Map,
  Menu,
  Radio,
  Radar,
  Search,
  Settings,
  ShieldAlert,
  Undo2,
  UserRound,
  Video,
} from 'lucide-react'
import type { ReactNode } from 'react'

const navItems = [
  { label: 'COMMAND HOME', to: '/', icon: Home },
  { label: 'LIVE MAP', to: '/dashboard/map', icon: Map },
  { label: 'MISSION COMMAND', to: '/dashboard/missions', icon: Radar },
  { label: 'DRONE FEED', to: '/dashboard/live-feed', icon: Video },
  { label: 'SURVIVORS', to: '/dashboard/victims', icon: UserRound },
  { label: 'HAZARD INTEL', to: '/dashboard/hazards', icon: ShieldAlert },
  { label: 'SAFE ROUTES', to: '/dashboard/safe-routes', icon: Compass },
  { label: 'ALERT CENTER', to: '/dashboard/alerts', icon: Bell },
  { label: 'AI PERCEPTION', to: '/dashboard/ai', icon: Cpu },
  { label: 'NAVIGATION', to: '/dashboard/navigation', icon: Crosshair },
  { label: 'COMMS', to: '/dashboard/communication', icon: Radio },
  { label: 'TELEMETRY', to: '/dashboard/telemetry', icon: Gauge },
  { label: 'REPORTS', to: '/dashboard/reports', icon: Grid2x2 },
  { label: 'MISSION REPLAY', to: '/dashboard/replay', icon: Undo2 },
  { label: 'SIMULATION LAB', to: '/dashboard/simulation', icon: Binary },
  { label: 'SYSTEM SETTINGS', to: '/dashboard/settings', icon: Settings },
]

const pageTitles: Record<string, string> = {
  '/': 'Command Home',
  '/dashboard': 'Mission Dashboard',
  '/dashboard/map': 'Live Map',
  '/dashboard/missions': 'Mission Command',
  '/dashboard/missions/new': 'Create Mission',
  '/dashboard/live-feed': 'Live Video',
  '/dashboard/victims': 'Survivor Intelligence',
  '/dashboard/hazards': 'Hazard Intelligence',
  '/dashboard/safe-routes': 'Safe Routes',
  '/dashboard/alerts': 'Alert Center',
  '/dashboard/ai': 'AI Perception',
  '/dashboard/navigation': 'Navigation',
  '/dashboard/communication': 'Communications',
  '/dashboard/telemetry': 'Telemetry',
  '/dashboard/reports': 'Mission Reports',
  '/dashboard/replay': 'Mission Replay',
  '/dashboard/simulation': 'Simulation Lab',
  '/dashboard/settings': 'System Settings',
}

function AppShell({ children }: { children: ReactNode }) {
  const [collapsed, setCollapsed] = useState(true)
  const [searchOpen, setSearchOpen] = useState(false)
  const [statusOpen, setStatusOpen] = useState(false)
  const [currentTime, setCurrentTime] = useState(new Date())
  const navigate = useNavigate()
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

        <nav className="flex flex-1 flex-col gap-2 p-2 pt-3">
          {navItems.map(({ label, to, icon: Icon }) => (
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
              <div className="flex items-center justify-between text-[10px] uppercase tracking-[0.12em] text-white/80"><span>Mission link</span><span className="text-[#8ae0ff]">ONLINE</span></div>
              <div className="mt-2 flex items-center justify-between text-[10px] uppercase tracking-[0.12em] text-white/80"><span>Drone AS-01</span><span className="text-[#86e2a4]">ACTIVE</span></div>
            </div>
          )}
        </header>

        <main className="min-h-0 flex-1 overflow-hidden bg-[#202635]">{children}</main>
      </div>
    </div>
  )
}

export default AppShell
