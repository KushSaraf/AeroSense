function CommunicationPage() {
  return (
    <div className="space-y-5">
      <div>
        <div className="text-[12px] uppercase tracking-[0.28em] text-text/60">Network resilience</div>
        <h1 className="mt-2 text-[32px] uppercase tracking-[0.14em] text-text">COMMUNICATION</h1>
      </div>

      <div className="grid gap-4 md:grid-cols-2 xl:grid-cols-4">
        {[
          ['5G / Wi-Fi', 'CONNECTED'],
          ['MAVLink', 'CONNECTED'],
          ['GPS', 'UNAVAILABLE'],
          ['VIO', 'ACTIVE'],
          ['Store-and-Forward', 'ACTIVE'],
          ['Mesh', 'STANDBY'],
        ].map(([label, value]) => (
          <div key={label} className="panel p-4">
            <div className="text-[10px] uppercase tracking-[0.18em] text-text/60">{label}</div>
            <div className="mt-3 text-[14px] font-medium uppercase tracking-[0.14em] text-text">{value}</div>
          </div>
        ))}
      </div>
    </div>
  )
}

export default CommunicationPage
