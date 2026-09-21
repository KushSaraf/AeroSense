import { useEffect, useState } from 'react'
import { NavLink, useLocation, useNavigate } from 'react-router-dom'
import { useDataSource } from '../hooks/useDataSource'
import { useMission } from '../hooks/useMission'
import ReplayBar from '../components/ReplayBar'
import { IS_REPLAY } from '../services/replay'
import {
  Activity,
  Bell,
  Box,
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
 * Grouped by what an operator is doing: running the flight, making sense of it, then the record
 * of it. The old split ("command" against "mission session") put the live map and the live
 * dashboard in different groups though an operator uses them side by side.
 */
type NavItem = { label: string; to: string; icon: typeof Home; end?: boolean; signal?: 'live' | 'alerts' }

const navSections: Array<{ title: string | null; items: NavItem[] }> = [
  { title: null, items: [{ label: 'Command home', to: '/', icon: Home, end: true }] },
  {
    title: 'Operate',
    items: [
      { label: 'Mission command', to: '/dashboard/missions', icon: Radar },
      { label: 'Live dashboard', to: '/dashboard', icon: Gauge, end: true, signal: 'live' },
      { label: 'Live map', to: '/dashboard/map', icon: Map },
      { label: '3D view', to: '/dashboard/scene', icon: Box },
    ],
  },
  {
    title: 'Analyse',
    items: [
      { label: 'Alert center', to: '/dashboard/alerts', icon: Bell, signal: 'alerts' },
      { label: 'AI perception', to: '/dashboard/ai', icon: Cpu },
      { label: 'Telemetry', to: '/dashboard/telemetry', icon: Activity },
    ],
  },
  { title: 'Record', items: [{ label: 'Reports', to: '/dashboard/reports', icon: FileText }] },
]
const settingsItem: NavItem = { label: 'System settings', to: '/dashboard/settings', icon: Settings }

/** One style for every item: active carries an accent bar, hover only a tint, so they differ. */
function SideLink({ item, collapsed, live, alerts }: {
  item: NavItem; collapsed: boolean; live: boolean; alerts: number
}) {
  const { label, to, icon: Icon, end, signal } = item
  const showLive = signal === 'live' && live
  const showAlerts = signal === 'alerts' && alerts > 0
  const count = alerts > 99 ? '99+' : String(alerts)
  return (
    <NavLink
      to={to}
      end={end}
      // only when collapsed: expanded, the label is already there and the tooltip covered the next item
      title={collapsed ? label + (showAlerts ? ` (${count} alerts)` : showLive ? ' (mission flying)' : '') : undefined}
      className={({ isActive }) => [
        'relative flex items-center gap-3 rounded-lg py-2.5 text-[12px] font-semibold uppercase tracking-[0.08em] transition',
        collapsed ? 'justify-center px-0' : 'px-3',
        isActive
          ? 'bg-[#8ae0ff]/12 text-white before:absolute before:bottom-2 before:left-0 before:top-2 before:w-[3px] before:rounded-full before:bg-[#8ae0ff]'
          : 'text-text/75 hover:bg-white/[0.06] hover:text-white',
      ].join(' ')}
    >
      <span className="relative flex">
        <Icon size={20} strokeWidth={1.8} />
        {collapsed && showLive && <span className="absolute -right-1 -top-1 h-2.5 w-2.5 animate-pulse rounded-full bg-[#43d17b] ring-2 ring-[#202a40]" />}
        {collapsed && showAlerts && (
          <span className="absolute -right-2.5 -top-2 min-w-[18px] rounded-full bg-[#e2707a] px-1 text-center text-[10px] font-bold leading-[18px] text-white ring-2 ring-[#202a40]">{count}</span>
        )}
      </span>
      {!collapsed && <span className="flex-1 truncate">{label}</span>}
      {!collapsed && showLive && (
        <span className="flex items-center gap-1.5 text-[10px] tracking-[0.08em] text-[#86e2a4]">
          <span className="h-2 w-2 animate-pulse rounded-full bg-[#43d17b]" />LIVE
        </span>
      )}
      {!collapsed && showAlerts && (
        <span className="min-w-[22px] rounded-full bg-[#e2707a]/25 px-2 text-center text-[11px] leading-5 text-[#ffc7cb]">{count}</span>
      )}
    </NavLink>
  )
}

const pageTitles: Record<string, string> = {
  '/': 'Command Home',
  '/dashboard': 'Mission Dashboard',
  '/dashboard/map': 'Live Map',
  '/dashboard/missions': 'Mission Command',
  '/dashboard/alerts': 'Alert Center',
  '/dashboard/ai': 'AI Perception',
  '/dashboard/telemetry': 'Telemetry',
  '/dashboard/scene': 'Local 3D View',
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
  const { mission, alerts } = useMission()
  const missionLive = mission?.status === 'ACTIVE'
  const alertCount = alerts.length

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

        <nav className="no-scrollbar flex flex-1 flex-col gap-0.5 overflow-y-auto p-2 pt-3">
          {navSections.map((section, index) => (
            <div key={section.title ?? 'home'} className={index ? 'mt-3' : ''}>
              {section.title && (collapsed
                ? <div className="mx-3 mb-2 border-t border-white/10" aria-hidden />
                : <div className="mb-1 px-3 text-[11px] font-semibold uppercase tracking-[0.16em] text-text/50">{section.title}</div>)}
              <div className="flex flex-col gap-0.5">
                {section.items.map((item) => (
                  <SideLink key={item.to} item={item} collapsed={collapsed} live={missionLive} alerts={alertCount} />
                ))}
              </div>
            </div>
          ))}
          {/* settings is configuration, not operation: kept out of the way at the foot */}
          <div className="mt-auto border-t border-white/10 pt-2">
            <SideLink item={settingsItem} collapsed={collapsed} live={false} alerts={0} />
          </div>
        </nav>

        <div className="border-t border-white/10 p-3 text-center text-[12px] tracking-[0.1em] text-text/75">
          {!collapsed && (
            <>
              <div>AERO</div>
              <div>SENSE</div>
              <div className="mt-2 text-[11px]">v1.0</div>
            </>
          )}
        </div>
      </aside>

      <div className="flex min-h-0 min-w-0 flex-1 flex-col">
        <header className="relative flex h-[74px] items-center justify-between border-b border-white/10 bg-[#202a40]/95 px-5 text-text backdrop-blur-sm">
          <div className="flex items-center gap-4">
            <img src={`${import.meta.env.BASE_URL}as-logo.png`} alt="AS" className="h-12 w-12 object-contain" />
            <div>
              <div className="text-[20px] font-semibold tracking-[0.2em] uppercase">AERO SENSE</div>
              <div className="hidden items-center gap-2 text-[13px] tracking-[0.2em] text-text/70 md:flex">
                <span>DETECT</span><span>|</span><span>LOCATE</span><span>|</span><span>ASSESS</span><span>|</span><span>GUIDE</span><span>|</span><span>SAVE LIVES</span>
              </div>
            </div>
          </div>

          <div className="flex items-center gap-4 text-[12px] tracking-[0.22em] uppercase text-text/85">
            <button type="button" onClick={() => setSearchOpen((value) => !value)} className="flex items-center gap-2 px-2 py-2 text-[14px] transition hover:text-white" aria-expanded={searchOpen}>
              <Search size={18} /> + SEARCH
            </button>
            <button type="button" onClick={() => setStatusOpen((value) => !value)} className="px-2 py-2 text-text/70 transition hover:text-white" aria-expanded={statusOpen}>STATUS</button>
            <button type="button" onClick={() => navigate('/dashboard/alerts')} className="px-2 py-2 text-text/70 transition hover:text-white">ALERT</button>
            <span
              title={dataSource === 'live'
                ? 'Live data from the running simulation'
                : 'No simulation is serving the dashboard bridge, so nothing is shown'}
              className={`rounded px-2 py-1 text-[12px] tracking-[0.1em] ${
                dataSource === 'live'
                  ? 'bg-emerald-500/20 text-emerald-300'
                  : 'bg-amber-500/20 text-amber-300'
              }`}
            >
              {IS_REPLAY ? 'REPLAY' : dataSource === 'live' ? 'LIVE' : 'NO SIMULATION'}
            </span>
            <time className="ml-1 border-l border-white/10 pl-4 font-mono text-[16px] tracking-[0.07em] text-text" dateTime={currentTime.toISOString()}>{formattedTime}</time>
          </div>

          {searchOpen && (
            <div className="absolute right-[190px] top-[66px] z-50 w-72 rounded-lg border border-white/15 bg-[#27334a] p-3 shadow-2xl">
              <label className="flex items-center gap-2 rounded border border-white/15 bg-[#1c2639] px-3 py-2 text-[13px] tracking-[0.07em] text-white/75">
                <Search size={15} />
                <input autoFocus placeholder="SEARCH MISSIONS, DRONES..." className="min-w-0 flex-1 bg-transparent text-[12px] uppercase tracking-[0.07em] text-white outline-none placeholder:text-white/40" />
              </label>
              <button type="button" onClick={() => { navigate('/dashboard/missions'); setSearchOpen(false) }} className="mt-2 w-full px-2 py-2 text-left text-[12px] uppercase tracking-[0.08em] text-white/75 transition hover:bg-white/10">Open mission command</button>
              <button type="button" onClick={() => { navigate('/dashboard/map'); setSearchOpen(false) }} className="w-full px-2 py-2 text-left text-[12px] uppercase tracking-[0.08em] text-white/75 transition hover:bg-white/10">Open live map</button>
            </div>
          )}

          {statusOpen && (
            <div className="absolute right-[115px] top-[66px] z-50 w-52 rounded-lg border border-white/15 bg-[#27334a] p-3 shadow-2xl">
              <div className="mb-3 text-[12px] uppercase tracking-[0.09em] text-white/70">System status</div>
              <div className="flex items-center justify-between text-[12px] uppercase tracking-[0.07em] text-white/80">
                <span>Bridge</span>
                <span className={dataSource === 'live' ? 'text-[#86e2a4]' : 'text-amber-300'}>
                  {dataSource === 'live' ? 'ONLINE' : 'OFFLINE'}
                </span>
              </div>
              <button type="button" onClick={() => { navigate('/dashboard/settings'); setStatusOpen(false) }}
                      className="mt-2 w-full px-2 py-2 text-left text-[12px] uppercase tracking-[0.07em] text-white/70 transition hover:bg-white/10">
                Open system settings
              </button>
            </div>
          )}
        </header>

        {IS_REPLAY && <ReplayBar />}
        <main className="app-main min-h-0 flex-1 overflow-auto bg-[#202635]">{children}</main>
      </div>
    </div>
  )
}

export default AppShell
