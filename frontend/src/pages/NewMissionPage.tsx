import { ArrowLeft, Check, ChevronDown, MapPinned, Radio, Save, ShieldCheck } from 'lucide-react'
import { useState } from 'react'
import { useNavigate } from 'react-router-dom'

const initialForm = {
  name: 'Earthquake SAR — Sector A',
  type: 'Earthquake',
  area: 'Riverdale West',
  priority: 'HIGH',
  owner: 'Command 1',
  drone: 'AS-01',
  pattern: 'Adaptive',
  altitude: '50 m',
  entry: 'North Ridge',
  returnPoint: 'Base Camp 1',
  threshold: '30%',
  communication: '5G / Wi-Fi',
  notes: 'Prioritize thermal signatures around collapsed structures and flooded access roads.',
}

function Field({ label, children }: { label: string; children: React.ReactNode }) {
  return <label className="flex flex-col gap-2 text-[10px] uppercase tracking-[0.16em] text-text/60">{label}{children}</label>
}

function Input({ value, onChange, placeholder }: { value: string; onChange: (value: string) => void; placeholder?: string }) {
  return <input value={value} onChange={(event) => onChange(event.target.value)} placeholder={placeholder} className="w-full rounded-md border border-white/12 bg-[#29344b] px-3 py-3 text-[12px] tracking-[0.06em] text-text outline-none transition focus:border-[#8ae0ff]/60 focus:bg-[#303e58]" />
}

function Select({ value, onChange, options }: { value: string; onChange: (value: string) => void; options: string[] }) {
  return <div className="relative"><select value={value} onChange={(event) => onChange(event.target.value)} className="w-full appearance-none rounded-md border border-white/12 bg-[#29344b] px-3 py-3 text-[12px] tracking-[0.06em] text-text outline-none focus:border-[#8ae0ff]/60">{options.map((option) => <option key={option}>{option}</option>)}</select><ChevronDown size={15} className="pointer-events-none absolute right-3 top-3.5 text-text/60" /></div>
}

function NewMissionPage() {
  const navigate = useNavigate()
  const [form, setForm] = useState(initialForm)
  const [isSaved, setIsSaved] = useState(false)
  const update = (key: keyof typeof form, value: string) => setForm((current) => ({ ...current, [key]: value }))

  return (
    <div className="h-full overflow-auto p-4 md:p-5">
      <div className="mx-auto max-w-[1500px] space-y-4">
        <header className="flex flex-wrap items-center justify-between gap-4 rounded-xl border border-white/10 bg-[#596278] px-5 py-4">
          <div className="flex items-center gap-4">
            <button type="button" onClick={() => navigate('/dashboard/missions')} className="flex h-9 w-9 items-center justify-center rounded-md border border-white/15 bg-white/10 text-white transition hover:bg-white/20" aria-label="Back to missions"><ArrowLeft size={17} /></button>
            <div><div className="aero-micro text-[10px] text-white/55">Mission command / planning</div><h1 className="aero-heading mt-1 text-[36px] uppercase leading-none text-white">NEW MISSION</h1></div>
          </div>
          <div className="flex items-center gap-3"><span className="rounded-full border border-yellow-300/30 bg-yellow-300/10 px-3 py-2 text-[10px] uppercase tracking-[0.16em] text-yellow-100">Draft configuration</span><button type="button" onClick={() => setIsSaved(true)} className="flex items-center gap-2 rounded-md bg-[#8ae0ff]/20 px-4 py-2 text-[10px] uppercase tracking-[0.16em] text-white transition hover:bg-[#8ae0ff]/30"><Save size={15} /> Save draft</button></div>
        </header>

        <div className="grid gap-4 xl:grid-cols-[minmax(0,1fr)_320px]">
          <main className="space-y-4">
            <section className="panel p-5"><div className="mb-5 flex items-center gap-3"><span className="flex h-8 w-8 items-center justify-center rounded-full bg-[#8ae0ff]/15 text-[#8ae0ff]">01</span><div><h2 className="aero-heading text-[18px] uppercase text-text">Mission identity</h2><p className="text-[10px] uppercase tracking-[0.12em] text-text/50">Define the incident and command ownership</p></div></div><div className="grid gap-4 md:grid-cols-2"><Field label="Mission name"><Input value={form.name} onChange={(value) => update('name', value)} /></Field><Field label="Incident type"><Select value={form.type} onChange={(value) => update('type', value)} options={['Earthquake', 'Flood', 'Landslide', 'Industrial Fire', 'Urban Collapse']} /></Field><Field label="Search area"><Input value={form.area} onChange={(value) => update('area', value)} /></Field><Field label="Mission owner"><Select value={form.owner} onChange={(value) => update('owner', value)} options={['Command 1', 'Command 2', 'Field Team Alpha']} /></Field><Field label="Priority"><Select value={form.priority} onChange={(value) => update('priority', value)} options={['HIGH', 'MEDIUM', 'LOW']} /></Field><Field label="Operating mode"><Select value="AUTONOMOUS SAR" onChange={() => undefined} options={['AUTONOMOUS SAR', 'ASSISTED', 'MANUAL']} /></Field></div></section>

            <section className="panel p-5"><div className="mb-5 flex items-center gap-3"><span className="flex h-8 w-8 items-center justify-center rounded-full bg-[#86e2a4]/15 text-[#86e2a4]">02</span><div><h2 className="aero-heading text-[18px] uppercase text-text">Deployment plan</h2><p className="text-[10px] uppercase tracking-[0.12em] text-text/50">Assign the aircraft and define the search pattern</p></div></div><div className="grid gap-4 md:grid-cols-2"><Field label="Assigned drone"><Select value={form.drone} onChange={(value) => update('drone', value)} options={['AS-01', 'AS-02', 'AS-03']} /></Field><Field label="Search pattern"><Select value={form.pattern} onChange={(value) => update('pattern', value)} options={['Adaptive', 'Grid', 'Lawn-Mower', 'Point-to-Point', 'Perimeter']} /></Field><Field label="Search altitude"><Select value={form.altitude} onChange={(value) => update('altitude', value)} options={['30 m', '50 m', '80 m', '120 m']} /></Field><Field label="Entry point"><Input value={form.entry} onChange={(value) => update('entry', value)} /></Field><Field label="Return point"><Input value={form.returnPoint} onChange={(value) => update('returnPoint', value)} /></Field><Field label="Battery return threshold"><Select value={form.threshold} onChange={(value) => update('threshold', value)} options={['20%', '30%', '40%', '50%']} /></Field></div><button type="button" onClick={() => navigate('/dashboard/map')} className="mt-5 flex items-center gap-2 rounded-md border border-[#8ae0ff]/30 bg-[#8ae0ff]/10 px-4 py-3 text-[10px] uppercase tracking-[0.16em] text-[#b8efff] transition hover:bg-[#8ae0ff]/20"><MapPinned size={15} /> Define search area on map</button></section>

            <section className="panel p-5"><div className="mb-5 flex items-center gap-3"><span className="flex h-8 w-8 items-center justify-center rounded-full bg-[#f0b35b]/15 text-[#f0b35b]">03</span><div><h2 className="aero-heading text-[18px] uppercase text-text">Safety and communications</h2><p className="text-[10px] uppercase tracking-[0.12em] text-text/50">Set the mission fail-safes before launch</p></div></div><div className="grid gap-4 md:grid-cols-2"><Field label="Communication mode"><Select value={form.communication} onChange={(value) => update('communication', value)} options={['5G / Wi-Fi', '5G only', 'Mesh relay', 'MAVLink relay']} /></Field><Field label="Lost-link behavior"><Select value="Return to base" onChange={() => undefined} options={['Return to base', 'Hover and wait', 'Land immediately']} /></Field></div><Field label="Operator notes"><textarea value={form.notes} onChange={(event) => update('notes', event.target.value)} rows={3} className="w-full resize-none rounded-md border border-white/12 bg-[#29344b] px-3 py-3 text-[12px] tracking-[0.06em] text-text outline-none transition focus:border-[#8ae0ff]/60" /></Field></section>
          </main>

          <aside className="space-y-4"><section className="panel overflow-hidden"><div className="border-b border-white/10 bg-[#596278] p-5"><div className="aero-micro text-[10px] text-white/55">Mission preview</div><h2 className="aero-heading mt-2 text-[24px] uppercase text-white">{form.name || 'UNNAMED MISSION'}</h2><div className="mt-2 text-[10px] uppercase tracking-[0.16em] text-white/60">{form.type} · {form.area}</div></div><div className="space-y-4 p-5"><div className="flex items-center justify-between"><span className="text-[10px] uppercase tracking-[0.16em] text-text/55">Readiness</span><span className="text-[14px] font-bold text-[#86e2a4]">READY 86%</span></div><div className="h-2 overflow-hidden rounded-full bg-white/10"><div className="h-full w-[86%] rounded-full bg-[#86e2a4]" /></div>{[['Drone', form.drone], ['Pattern', form.pattern], ['Altitude', form.altitude], ['Priority', form.priority], ['Link', form.communication]].map(([label, value]) => <div key={label} className="flex items-center justify-between border-b border-white/10 pb-3 text-[10px] uppercase tracking-[0.14em]"><span className="text-text/55">{label}</span><span className="text-text">{value}</span></div>)}<div className="rounded-md border border-[#86e2a4]/25 bg-[#86e2a4]/10 p-3 text-[10px] uppercase leading-5 tracking-[0.12em] text-[#c5f4d0]"><ShieldCheck size={16} className="mb-2" />All launch safety gates are configured.</div></div></section><section className="panel p-5"><div className="mb-4 flex items-center gap-2 text-[10px] uppercase tracking-[0.16em] text-text/60"><Radio size={15} /> Launch checklist</div>{['Mission details complete', 'Drone assigned', 'Search area defined', 'Return threshold set', 'Communication link ready'].map((item) => <div key={item} className="flex items-center gap-3 py-2 text-[10px] uppercase tracking-[0.1em] text-text/75"><span className="flex h-5 w-5 items-center justify-center rounded-full border border-[#86e2a4]/50 text-[#86e2a4]"><Check size={12} /></span>{item}</div>)}</section><button type="button" onClick={() => setIsSaved(true)} className="flex w-full items-center justify-center gap-2 rounded-md bg-[#86e2a4] px-4 py-4 text-[11px] font-bold uppercase tracking-[0.18em] text-[#13251b] transition hover:bg-[#a1efb5]"><ShieldCheck size={17} />{isSaved ? 'MISSION CONFIGURATION SAVED' : 'SAVE AND PREPARE MISSION'}</button></aside>
        </div>
      </div>
    </div>
  )
}

export default NewMissionPage
