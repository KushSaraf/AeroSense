function SettingsPage() {
  return (
    <div className="space-y-5">
      <div>
        <div className="text-[12px] uppercase tracking-[0.28em] text-text/60">System configuration</div>
        <h1 className="mt-2 text-[32px] uppercase tracking-[0.14em] text-text">SETTINGS</h1>
      </div>

      <div className="grid gap-4 md:grid-cols-2 xl:grid-cols-4">
        {['Drone', 'Sensors', 'AI Models', 'Navigation', 'Communication', 'Alerts', 'Triage', 'Map', 'Users', 'System'].map((section) => (
          <div key={section} className="panel p-4">
            <div className="text-[10px] uppercase tracking-[0.2em] text-text/60">{section}</div>
            <div className="mt-3 text-[12px] uppercase tracking-[0.14em] text-text/75">Ready</div>
          </div>
        ))}
      </div>
    </div>
  )
}

export default SettingsPage
