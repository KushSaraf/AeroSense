import { Line, LineChart, ResponsiveContainer, Tooltip, XAxis, YAxis } from 'recharts'
import { useMission } from '../hooks/useMission'

const CHARTS = [
  { key: 'battery', label: 'Battery %', colour: '#86e2a4' },
  { key: 'altitude', label: 'Altitude (m)', colour: '#8ae0ff' },
  { key: 'speed', label: 'Ground speed (m/s)', colour: '#ff9f43' },
] as const

function Reading({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded border border-white/12 bg-white/5 p-3">
      <div className="text-[12px] uppercase tracking-[0.1em] text-text/72">{label}</div>
      <div className="mt-2 text-[20px] font-medium tracking-[0.06em] text-text">{value}</div>
    </div>
  )
}

/** The autopilot's own numbers. Anything the autopilot does not report is shown as unknown. */
function TelemetryPage() {
  const { drone, telemetry, mission } = useMission()

  const readings: Array<[string, string]> = [
    ['Battery', drone?.battery != null ? `${drone.battery.toFixed(0)}%` : 'unknown'],
    ['Altitude', drone ? `${drone.altitude.toFixed(1)} m` : 'unknown'],
    ['Ground speed', drone ? `${drone.speed.toFixed(1)} m/s` : 'unknown'],
    ['Flight mode', drone?.mode ?? 'unknown'],
    ['Armed', drone ? (drone.armed ? 'yes' : 'no') : 'unknown'],
    ['GNSS', drone?.gps ?? 'unknown'],
    ['Downlink', drone?.link ?? 'unknown'],
    ['Flight time', drone?.flightTime ?? 'unknown'],
    ['Latitude', drone?.latitude != null ? drone.latitude.toFixed(6) : 'no fix'],
    ['Longitude', drone?.longitude != null ? drone.longitude.toFixed(6) : 'no fix'],
    ['Mission state', mission?.state ?? 'no mission'],
    ['Coverage', mission ? `${mission.coverage.toFixed(0)}%` : '—'],
  ]

  return (
    <div className="space-y-5 p-4 md:p-6">
      <div>
        <div className="text-[12px] uppercase tracking-[0.09em] text-text/75">Real-time metrics</div>
        <h1 className="aero-heading mt-1 text-[36px] uppercase leading-none text-text">TELEMETRY</h1>
      </div>

      <div className="grid gap-3 md:grid-cols-3 xl:grid-cols-4">
        {readings.map(([label, value]) => <Reading key={label} label={label} value={value} />)}
      </div>

      <div className="grid gap-4 xl:grid-cols-3">
        {CHARTS.map(({ key, label, colour }) => (
          <section key={key} className="panel p-4">
            <div className="text-[12px] uppercase tracking-[0.1em] text-text/75">{label}</div>
            <div className="mt-3 h-[180px]">
              {telemetry.length > 1 ? (
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart data={telemetry}>
                    <XAxis dataKey="time" tick={{ fill: '#8f9cb5', fontSize: 9 }} minTickGap={40} />
                    <YAxis tick={{ fill: '#8f9cb5', fontSize: 9 }} width={34} />
                    <Tooltip contentStyle={{ background: '#27334a', border: '1px solid rgba(255,255,255,0.15)', fontSize: 11 }} />
                    <Line type="monotone" dataKey={key} stroke={colour} dot={false} strokeWidth={2} isAnimationActive={false} />
                  </LineChart>
                </ResponsiveContainer>
              ) : (
                <div className="flex h-full items-center justify-center text-[12px] uppercase tracking-[0.08em] text-text/40">
                  Waiting for telemetry
                </div>
              )}
            </div>
          </section>
        ))}
      </div>
    </div>
  )
}

export default TelemetryPage
