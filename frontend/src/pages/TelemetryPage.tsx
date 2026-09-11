function TelemetryPage() {
  return (
    <div className="space-y-5">
      <div>
        <div className="text-[12px] uppercase tracking-[0.28em] text-text/60">Real-time metrics</div>
        <h1 className="mt-2 text-[32px] uppercase tracking-[0.14em] text-text">TELEMETRY</h1>
      </div>

      <div className="grid gap-4 md:grid-cols-2 xl:grid-cols-4">
        {[
          ['Battery', '78%'],
          ['Voltage', '26.8V'],
          ['Current', '14.2A'],
          ['Altitude', '50 m'],
          ['Speed', '5.2 m/s'],
          ['Heading', '118°'],
          ['GPS satellites', '12'],
          ['GPS accuracy', '1.2m'],
          ['CPU', '48%'],
          ['NPU', '68%'],
          ['Temperature', '52°C'],
          ['Flight time', '14:32'],
        ].map(([label, value]) => (
          <div key={label} className="panel p-4">
            <div className="text-[10px] uppercase tracking-[0.18em] text-text/60">{label}</div>
            <div className="mt-3 text-[22px] font-semibold tracking-[0.08em] text-text">{value}</div>
          </div>
        ))}
      </div>
    </div>
  )
}

export default TelemetryPage
