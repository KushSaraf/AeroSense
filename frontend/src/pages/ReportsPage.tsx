import { Download, FileText, Printer } from 'lucide-react'
import { useCallback, useEffect, useState } from 'react'
import MissionReportDocument from '../components/MissionReportDocument'
import type { MissionReport } from '../components/MissionReportDocument'
import { useApi } from '../hooks/useApi'
import { useMission } from '../hooks/useMission'
import { API_URL } from '../services/apiServices'
import '../styles/report.css'

interface ReportSummary {
  id: string
  name: string
  status: string
  coverage: number
  victims: number
}

const REPORT_POLL_MS = 5000

const download = (filename: string, body: string, type: string) => {
  const url = URL.createObjectURL(new Blob([body], { type }))
  const link = document.createElement('a')
  link.href = url
  link.download = filename
  link.click()
  URL.revokeObjectURL(url)
}

const toCsv = (report: MissionReport): string => {
  const header = 'id,priority,confidence,latitude,longitude,thermal,first_seen'
  const rows = report.victims.map((victim) => [
    victim.id, victim.priority, victim.confidence.toFixed(3),
    victim.latitude.toFixed(6), victim.longitude.toFixed(6),
    victim.thermalStrength, victim.timestamp,
  ].join(','))
  return [header, ...rows].join('\n')
}

function ReportsPage() {
  const { data: summaries, error } = useApi<ReportSummary[]>('/api/reports', REPORT_POLL_MS)
  const [selectedId, setSelectedId] = useState<string | null>(null)
  const [report, setReport] = useState<MissionReport | null>(null)
  const [loadError, setLoadError] = useState<string | null>(null)

  const { mission: live } = useMission()
  const missions = summaries ?? []
  const activeId = selectedId && missions.some((entry) => entry.id === selectedId)
    ? selectedId : missions[0]?.id ?? null
  // A report is written when a mission ends, so while one flies the page says so rather than
  // showing a report of a search half done.
  const flying = live && live.status === 'ACTIVE' ? live : null
  const waiting = flying
    ? `${flying.name} is still flying (${flying.state}, ${flying.elapsed}). Its report is written when it ends.`
    : 'No mission has ended yet. A report is written when a mission ends.'

  const loadReport = useCallback(async (missionId: string) => {
    try {
      const response = await fetch(`${API_URL}/api/reports/${missionId}`)
      const body = await response.json()
      if (body.error) throw new Error(body.error)
      setReport(body as MissionReport)
      setLoadError(null)
    } catch (cause: unknown) {
      setReport(null)
      setLoadError(cause instanceof Error ? cause.message : 'could not load that report')
    }
  }, [])

  useEffect(() => {
    // no ended mission (a replay looping back to take-off, say): drop the report on screen
    if (!activeId) {
      setReport(null)
      return undefined
    }
    void loadReport(activeId)
    const timer = setInterval(() => void loadReport(activeId), REPORT_POLL_MS)
    return () => clearInterval(timer)
  }, [activeId, loadReport])

  return (
    <div className="space-y-5 p-4 md:p-6">
      <div className="flex flex-wrap items-end justify-between gap-4">
        <div>
          <div className="text-[12px] uppercase tracking-[0.09em] text-text/75">Mission reporting</div>
          <h1 className="aero-heading mt-1 text-[36px] uppercase leading-none text-text">REPORTS</h1>
          <p className="mt-2 max-w-[620px] text-[12px] uppercase tracking-[0.07em] text-text/78">
            Generated from recorded mission data. Nothing here is written by hand.
          </p>
        </div>
        {report && (
          <div className="flex flex-wrap gap-2 text-[12px] uppercase tracking-[0.09em]">
            <button type="button" onClick={() => window.print()}
                    className="flex items-center gap-2 rounded border border-[#8ae0ff]/50 bg-[#8ae0ff]/15 px-4 py-3 text-text hover:bg-[#8ae0ff]/25">
              <Printer size={14} /> Print / PDF
            </button>
            <button type="button" onClick={() => download(`${report.reportId}.json`, JSON.stringify(report, null, 2), 'application/json')}
                    className="flex items-center gap-2 rounded border border-white/15 bg-white/5 px-4 py-3 text-text/80 hover:bg-white/10">
              <Download size={14} /> JSON
            </button>
            <button type="button" onClick={() => download(`${report.reportId}-casualties.csv`, toCsv(report), 'text/csv')}
                    className="flex items-center gap-2 rounded border border-white/15 bg-white/5 px-4 py-3 text-text/80 hover:bg-white/10">
              <Download size={14} /> CSV
            </button>
          </div>
        )}
      </div>

      {(error || loadError) && (
        <div className="rounded border border-amber-300/30 bg-amber-300/10 px-4 py-3 text-[13px] tracking-[0.1em] text-amber-200">
          {error ?? loadError}
        </div>
      )}

      <div className="grid gap-5 xl:grid-cols-[minmax(240px,0.6fr)_minmax(0,2.4fr)]">
        <section className="panel h-fit p-4">
          <div className="mb-4 flex items-center justify-between">
            <div className="text-[12px] uppercase tracking-[0.2em] text-text/75">Ended missions</div>
            <FileText size={16} className="text-[#8ae0ff]" />
          </div>
          <div className="space-y-2">
            {missions.map((mission) => (
              <button key={mission.id} type="button" onClick={() => setSelectedId(mission.id)}
                      className={`w-full rounded-lg border p-3 text-left transition ${activeId === mission.id ? 'border-[#8ae0ff]/70 bg-[#8ae0ff]/12' : 'border-white/10 bg-white/5 hover:bg-white/10'}`}>
                <div className="text-[13px] font-semibold uppercase tracking-[0.07em] text-text">{mission.name}</div>
                <div className="mt-2 flex justify-between text-[11px] uppercase tracking-[0.08em] text-text/72">
                  <span>{mission.status}</span>
                  <span>{mission.coverage.toFixed(0)}% · {mission.victims} found</span>
                </div>
              </button>
            ))}
            {missions.length === 0 && !error && (
              <div className="rounded border border-white/10 bg-white/5 px-3 py-6 text-center text-[12px] uppercase tracking-[0.08em] text-text/78">
                {waiting}
              </div>
            )}
          </div>
        </section>

        <section className="overflow-auto rounded-xl border border-white/10 bg-[#1a2030] p-4">
          {report
            ? <MissionReportDocument report={report} />
            : <div className="mx-auto max-w-[520px] py-24 text-center text-[14px] leading-6 text-text/78">
                {missions.length ? loadError ?? 'Loading the report…' : waiting}
              </div>}
        </section>
      </div>
    </div>
  )
}

export default ReportsPage
