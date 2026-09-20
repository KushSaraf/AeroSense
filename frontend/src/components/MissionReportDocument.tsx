import type { MissionEvent, Victim } from '../types'

export interface MissionReport {
  reportId: string
  generated: string
  mission: {
    id: string
    name: string
    disasterType: string
    status: string
    elapsed?: string
    coverage: number
    scenario?: string
  }
  summary: {
    coveragePercent: number
    victimsDetected: number
    byPriority: Record<string, number>
    duration: string
    durationSeconds: number
    hazardsIdentified: number
    dronesUsed: number
    status: string
  }
  victims: Victim[]
  events: MissionEvent[]
  drone: {
    id: string
    name: string
    model: string
    flightTime: string
    /** Null while the autopilot had not measured it. */
    battery: number | null
    gps: string
    link: string
  }
  recommendations: string[]
}

const PRIORITY_LABEL: Record<string, string> = {
  P1: 'P1 — immediate',
  P2: 'P2 — urgent',
  P3: 'P3 — delayed',
  UNTRIAGED: 'Untriaged',
}

function Section({ index, title, children }: { index: number; title: string; children: React.ReactNode }) {
  return (
    <section className="report-section">
      <h2>{index}. {title}</h2>
      {children}
    </section>
  )
}

function Fields({ rows }: { rows: Array<[string, string]> }) {
  return (
    <table className="report-fields">
      <tbody>
        {rows.map(([label, value]) => (
          <tr key={label}><th>{label}</th><td>{value}</td></tr>
        ))}
      </tbody>
    </table>
  )
}

/**
 * The printable mission report: A4, ten sections, every number taken from the mission that was
 * flown. Where the system has no data — no hazard regions mapped — the report says so instead
 * of printing a zero that reads like a finding.
 */
function MissionReportDocument({ report }: { report: MissionReport }) {
  const { summary, mission, drone } = report
  const counted = summary.byPriority

  return (
    <article className="report-page">
      <header className="report-header">
        <div>
          <div className="report-brand">AERO SENSE</div>
          <div className="report-subtitle">Autonomous search &amp; rescue — mission report</div>
        </div>
        <div className="report-meta">
          <div><strong>{report.reportId}</strong></div>
          <div>Generated {report.generated}</div>
        </div>
      </header>

      <Section index={1} title="Mission identification">
        <Fields rows={[
          ['Mission ID', mission.id],
          ['Mission name', mission.name],
          ['Disaster type', mission.disasterType],
          ['Sector', mission.scenario ?? mission.disasterType],
          ['Status at report', summary.status],
          ['Aircraft', `${drone.name} (${drone.model})`],
          ['Aircraft deployed', String(summary.dronesUsed)],
        ]} />
      </Section>

      <Section index={2} title="Executive summary">
        <p>
          {summary.victimsDetected === 0
            ? `No casualties were confirmed over ${summary.coveragePercent.toFixed(0)}% of the sector searched in ${summary.duration}.`
            : `${summary.victimsDetected} casualt${summary.victimsDetected === 1 ? 'y was' : 'ies were'} confirmed over ${summary.coveragePercent.toFixed(0)}% of the sector, searched in ${summary.duration}. Of these, ${counted.P1 ?? 0} require immediate extraction.`}
          {' '}Each confirmation required repeated thermal looks from independent viewpoints; single-frame hits were not reported.
        </p>
      </Section>

      <Section index={3} title="Search coverage">
        <Fields rows={[
          ['Coverage of assigned sector', `${summary.coveragePercent.toFixed(1)}%`],
          ['Flight duration', summary.duration],
          ['Duration (seconds)', String(Math.round(summary.durationSeconds))],
          ['Coverage method', 'Measured from the camera footprint swept over the sector grid'],
        ]} />
      </Section>

      <Section index={4} title="Casualty register">
        <table className="report-table">
          <thead>
            <tr><th>ID</th><th>Priority</th><th>Confidence</th><th>Position (lat, lon)</th><th>Thermal</th><th>First seen</th></tr>
          </thead>
          <tbody>
            {report.victims.map((victim) => (
              <tr key={victim.id}>
                <td>{victim.id}</td>
                <td>{victim.priority}</td>
                <td>{(victim.confidence * 100).toFixed(0)}%</td>
                <td>{victim.latitude.toFixed(6)}, {victim.longitude.toFixed(6)}</td>
                <td>{victim.thermalStrength}</td>
                <td>{victim.timestamp}</td>
              </tr>
            ))}
            {report.victims.length === 0 && <tr><td colSpan={6}>No casualties confirmed.</td></tr>}
          </tbody>
        </table>
      </Section>

      <Section index={5} title="Triage breakdown">
        <table className="report-table">
          <thead><tr><th>Priority</th><th>Count</th><th>Meaning</th></tr></thead>
          <tbody>
            <tr><td>{PRIORITY_LABEL.P1}</td><td>{counted.P1 ?? 0}</td><td>Extract first: trapped, or in water</td></tr>
            <tr><td>{PRIORITY_LABEL.P2}</td><td>{counted.P2 ?? 0}</td><td>Beside or under damaged structure</td></tr>
            <tr><td>{PRIORITY_LABEL.P3}</td><td>{counted.P3 ?? 0}</td><td>Open ground, reachable on foot</td></tr>
            <tr><td>{PRIORITY_LABEL.UNTRIAGED}</td><td>{counted.UNTRIAGED ?? 0}</td><td>Confirmed but not yet scored</td></tr>
          </tbody>
        </table>
      </Section>

      <Section index={6} title="Hazards">
        {summary.hazardsIdentified === 0
          ? <p className="report-note">No hazard regions were mapped on this flight (disaster segmentation found none, or did not run).</p>
          : <p>{summary.hazardsIdentified} hazard regions mapped by disaster segmentation (SegFormer-B0), banded by HSI.</p>}
      </Section>

      <Section index={7} title="Platform and link">
        <Fields rows={[
          ['Airframe', `${drone.name} (${drone.model})`],
          ['Flight time', drone.flightTime],
          ['Battery at report', drone.battery != null ? `${drone.battery.toFixed(0)}%` : 'not measured'],
          ['GNSS', drone.gps],
          ['Downlink', drone.link],
        ]} />
      </Section>

      <Section index={8} title="Mission event timeline">
        <table className="report-table">
          <thead><tr><th>Time</th><th>Event</th></tr></thead>
          <tbody>
            {report.events.map((event, index) => (
              <tr key={`${event.time}-${index}`}><td>{event.time}</td><td>{event.text}</td></tr>
            ))}
            {report.events.length === 0 && <tr><td colSpan={2}>No events recorded.</td></tr>}
          </tbody>
        </table>
      </Section>

      <Section index={9} title="Recommendations">
        <ol className="report-list">
          {report.recommendations.map((line) => <li key={line}>{line}</li>)}
        </ol>
      </Section>

      <Section index={10} title="Basis and limitations">
        <ul className="report-list">
          <li>Every figure above is derived from recorded mission data: autopilot telemetry, thermal detections and the mission state machine's own event log.</li>
          <li>Positions are geolocated by projecting the thermal detection through the camera onto the ground plane; accuracy degrades over sloped or elevated ground.</li>
          <li>Casualties fully inside intact structures are not visible to a downward thermal camera and are not claimed here.</li>
          <li>This report is generated from a simulation and is not a record of a real incident.</li>
        </ul>
      </Section>

      <footer className="report-footer">
        {report.reportId} · {mission.name} · generated {report.generated} · Aero Sense
      </footer>
    </article>
  )
}

export default MissionReportDocument
