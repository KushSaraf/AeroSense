interface StatusPillProps {
  label: string
  tone?: 'green' | 'yellow' | 'orange' | 'red' | 'gray'
}

const toneMap = {
  green: 'border-emerald-400/40 bg-emerald-500/10 text-emerald-100',
  yellow: 'border-yellow-400/40 bg-yellow-500/10 text-yellow-100',
  orange: 'border-orange-400/40 bg-orange-500/10 text-orange-100',
  red: 'border-red-400/40 bg-red-500/10 text-red-100',
  gray: 'border-white/10 bg-white/5 text-text/80',
}

export default function StatusPill({ label, tone = 'gray' }: StatusPillProps) {
  return (
    <span className={`inline-flex items-center gap-2 rounded-full border px-2.5 py-1 text-[10px] font-medium uppercase tracking-[0.14em] ${toneMap[tone]}`}>
      <span className="status-dot" style={{ backgroundColor: tone === 'green' ? '#7EB17A' : tone === 'yellow' ? '#C7B16A' : tone === 'orange' ? '#D68E58' : tone === 'red' ? '#C96B68' : '#C7CBD5' }} />
      {label}
    </span>
  )
}
