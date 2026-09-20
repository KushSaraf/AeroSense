import { BarChart3, Crosshair, MapPin, Route, ShieldCheck } from 'lucide-react'

const capabilityItems = [
  { icon: Crosshair, label: 'DETECT', sub: 'SURVIVORS' },
  { icon: MapPin, label: 'LOCATE', sub: 'HAZARDS' },
  { icon: BarChart3, label: 'ASSESS', sub: 'RISKS' },
  { icon: Route, label: 'GUIDE', sub: 'RESPONDERS' },
  { icon: ShieldCheck, label: 'SAVE LIVES', sub: 'TOGETHER' },
]

function HomePage() {
  return (
    <section className="relative h-full overflow-hidden bg-[#25344a] text-[#172235]">
      <div className="absolute inset-0 bg-cover bg-center opacity-80"
           style={{ backgroundImage: `url(${import.meta.env.BASE_URL}disaster-landscape.png)` }} />
      <div className="absolute inset-0 bg-[linear-gradient(180deg,rgba(132,151,169,0.38)_0%,rgba(62,82,105,0.58)_53%,rgba(18,32,51,0.96)_100%)]" />
      <div className="absolute inset-0 bg-[radial-gradient(ellipse_at_24%_26%,rgba(210,222,232,0.28),transparent_32%),linear-gradient(90deg,rgba(210,218,225,0.14)_1px,transparent_1px),linear-gradient(rgba(210,218,225,0.12)_1px,transparent_1px)] bg-[size:auto,80px_80px,80px_80px] opacity-75" />
      <div className="absolute inset-x-0 bottom-0 h-1/2 bg-[linear-gradient(180deg,transparent,rgba(16,29,47,0.72))]" />

      <div className="relative h-full min-w-[820px]">
        <div className="hero-lockup absolute left-[5.5%] top-[8%] w-[52%] max-w-[720px] text-[#162033]">
          <div className="hero-corner hero-corner-tl" />
          <div className="hero-corner hero-corner-tr" />
          <div className="hero-corner hero-corner-bl" />
          <div className="hero-corner hero-corner-br" />
          <h1 className="hero-wordmark aero-heading uppercase">AERO<br />SENSE</h1>
          <p className="aero-micro hero-subtitle uppercase">Autonomous Search and Rescue Drone</p>
        </div>

        <p className="aero-micro absolute right-[5%] top-[10%] max-w-[250px] text-[clamp(11px,0.9vw,15px)] font-bold uppercase leading-[1.8] tracking-[0.2em] text-white/85">
          Intelligence
          <br />
          in every mission
          <br />
          a safer tomorrow
        </p>

        <div className="aero-micro absolute left-[7%] top-[57%] border-l-2 border-white/70 pl-5 text-[clamp(11px,1vw,16px)] font-bold uppercase leading-[1.75] tracking-[0.1em] text-white">
          From disaster
          <br />
          to hope
          <br />
          with intelligent
          <br />
          drones
        </div>

        <div className="absolute left-[28%] top-[26%] h-[48%] w-[66%] overflow-hidden">
          <img src={`${import.meta.env.BASE_URL}drone-render.png`} alt="Aero Sense rescue drone" className="absolute left-[-6%] top-[-24%] w-[112%] max-w-none mix-blend-multiply drop-shadow-[0_20px_22px_rgba(12,20,32,0.35)]" />
        </div>

        <div className="absolute inset-x-0 bottom-[10%] flex justify-center px-8">
          <div className="grid w-full max-w-[1180px] grid-cols-5 border-y border-white/20 py-4">
            {capabilityItems.map(({ icon: Icon, label, sub }, index) => (
              <div key={label} className={`flex items-center justify-center gap-4 px-4 ${index > 0 ? 'border-l border-white/35' : ''}`}>
                <Icon size={42} strokeWidth={1.6} className="shrink-0 text-white" />
                <div className="aero-micro text-left text-[clamp(10px,0.85vw,14px)] font-bold uppercase tracking-[0.07em] text-white">
                  <div>{label}</div>
                  <div className="mt-1 text-[clamp(9px,0.65vw,11px)] text-white/70">{sub}</div>
                </div>
              </div>
            ))}
          </div>
        </div>

        <footer className="absolute inset-x-0 bottom-0 flex h-[7%] items-center justify-between border-t border-white/15 bg-[#1b2a40]/75 px-7 text-[12px] uppercase tracking-[0.09em] text-white/78">
          <span>Built for a safer tomorrow</span>
          <span>AERO SENSE&nbsp;&nbsp; | &nbsp;&nbsp;SIH 2026</span>
        </footer>
      </div>
    </section>
  )
}

export default HomePage
