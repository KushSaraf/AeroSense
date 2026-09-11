import type { ReactNode } from 'react'

interface MetricCardProps {
  label: string
  value: string
  accent?: 'green' | 'yellow' | 'orange' | 'red' | 'default'
  children?: ReactNode
}

const accentMap = {
  green: 'border-emerald-400/30 bg-emerald-500/10 text-emerald-200',
  yellow: 'border-yellow-400/30 bg-yellow-500/10 text-yellow-100',
  orange: 'border-orange-400/30 bg-orange-500/10 text-orange-100',
  red: 'border-red-400/30 bg-red-500/10 text-red-100',
  default: 'border-white/10 bg-white/5 text-text',
}

export default function MetricCard({ label, value, accent = 'default', children }: MetricCardProps) {
  return (
    <div className={`panel flex min-h-[116px] flex-1 flex-col justify-between border p-4 ${accentMap[accent]}`}>
      <div className="text-[10px] uppercase tracking-[0.24em] text-text/60">{label}</div>
      <div className="mt-4 text-2xl font-semibold tracking-[0.06em] text-text">{value}</div>
      {children}
    </div>
  )
}
