function NavigationPage() {
  return (
    <div className="space-y-5">
      <div>
        <div className="text-[12px] uppercase tracking-[0.28em] text-text/60">Localization</div>
        <h1 className="mt-2 text-[32px] uppercase tracking-[0.14em] text-text">NAVIGATION</h1>
      </div>

      <div className="panel p-5">
        <div className="grid gap-6 md:grid-cols-2">
          <div className="space-y-4">
            <div className="text-[10px] uppercase tracking-[0.22em] text-text/60">State indicator</div>
            <div className="space-y-2 text-[11px] uppercase tracking-[0.18em] text-text/75">
              <div>GPS AVAILABLE</div>
              <div>GPS LOST</div>
              <div>VIO / SLAM ACTIVE</div>
              <div>AUTONOMOUS CONTINUATION</div>
            </div>
          </div>
          <div className="space-y-3 text-[11px] uppercase tracking-[0.14em] text-text/70">
            <div className="flex justify-between"><span>Position</span><span className="text-text">28.6140, 77.2221</span></div>
            <div className="flex justify-between"><span>Velocity</span><span className="text-text">5.2 m/s</span></div>
            <div className="flex justify-between"><span>Altitude</span><span className="text-text">50 m</span></div>
            <div className="flex justify-between"><span>Heading</span><span className="text-text">118°</span></div>
            <div className="flex justify-between"><span>Satellites</span><span className="text-text">12 GPS</span></div>
            <div className="flex justify-between"><span>SLAM status</span><span className="text-text">ACTIVE</span></div>
          </div>
        </div>
      </div>
    </div>
  )
}

export default NavigationPage
