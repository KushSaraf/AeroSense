import { AlertTriangle, BellRing, UserRound } from 'lucide-react'
import { useState } from 'react'
import { useApi } from '../hooks/useApi'
import type { AlertItem } from '../types'

const SEVERITY_TONE: Record<string, string> = {
  CRITICAL: 'border-[#ef5350]/40 bg-[#ef5350]/10 text-[#ffb4b4]',
  HIGH: 'border-[#ff9f43]/40 bg-[#ff9f43]/10 text-[#ffd7a8]',
  MODERATE: 'border-[#8ae0ff]/40 bg-[#8ae0ff]/10 text-[#bfeaff]',
}

function AlertsPage() {
  const { data, error } = useApi<AlertItem[]>('/api/alerts', 2000)
  const [filter, setFilter] = useState<'ALL' | 'SURVIVOR' | 'SYSTEM'>('ALL')
  const alerts = (data ?? []).filter((alert) => filter === 'ALL' || alert.type === filter)

  return (
    <div className="min-h-full space-y-4 p-4 md:p-6">
      <header className="flex items-end justify-between">
        <div>
          <div className="aero-micro text-[12px] tracking-[0.09em] text-white/70">Alert center</div>
          <h1 className="aero-heading mt-1 text-[38px] uppercase leading-none text-white">ALERTS</h1>
          <p className="mt-2 max-w-[640px] text-[12px] uppercase tracking-[0.07em] text-white/78">
            Raised by the mission itself: a casualty alert exists because perception confirmed one.
          </p>
        </div>
        <div className="flex gap-2">
          {(['ALL', 'SURVIVOR', 'SYSTEM'] as const).map((option) => (
            <button key={option} type="button" onClick={() => setFilter(option)}
                    className={`rounded border px-3 py-2 text-[12px] uppercase tracking-[0.09em] transition ${
                      filter === option ? 'border-[#8ae0ff]/50 bg-[#8ae0ff]/15 text-white' : 'border-white/15 text-white/75 hover:bg-white/10'}`}>
              {option}
            </button>
          ))}
        </div>
      </header>

      {error && <div className="rounded border border-amber-300/30 bg-amber-300/10 px-4 py-3 text-[13px] tracking-[0.1em] text-amber-200">{error}</div>}

      <div className="space-y-3">
        {alerts.map((alert) => (
          <article key={alert.id} className={`rounded-xl border p-4 ${SEVERITY_TONE[alert.severity] ?? 'border-white/15 bg-white/5 text-white/80'}`}>
            <div className="flex flex-wrap items-center gap-3">
              <span className="flex h-8 w-8 items-center justify-center rounded bg-black/25">
                {alert.type === 'SURVIVOR' ? <UserRound size={17} /> : <AlertTriangle size={17} />}
              </span>
              <strong className="text-[14px] uppercase tracking-[0.07em] text-white">{alert.title}</strong>
              <span className="rounded bg-black/25 px-2 py-1 text-[12px] tracking-[0.08em]">{alert.priority}</span>
              <span className="ml-auto text-[12px] tracking-[0.08em] text-white/72">{alert.timestamp}</span>
            </div>
            <div className="mt-3 grid gap-3 text-[12px] uppercase tracking-[0.07em] text-white/75 md:grid-cols-3">
              <div><div className="text-white/40">Location</div><div className="mt-1 text-white/85">{alert.location || '—'}</div></div>
              <div><div className="text-white/40">Confidence</div><div className="mt-1 text-white/85">{(alert.confidence * 100).toFixed(0)}%</div></div>
              <div><div className="text-white/40">Evidence</div><div className="mt-1 normal-case tracking-normal text-white/85">{alert.rationale}</div></div>
            </div>
          </article>
        ))}
        {alerts.length === 0 && !error && (
          <div className="flex items-center gap-3 rounded-xl border border-white/10 bg-white/5 px-4 py-8 text-[13px] uppercase tracking-[0.08em] text-white/78">
            <BellRing size={18} /> No alerts: nothing has been detected in this mission yet.
          </div>
        )}
      </div>
    </div>
  )
}

export default AlertsPage
