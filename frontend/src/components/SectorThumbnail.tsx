import type { Victim } from '../types'

interface Bounds {
  minX: number
  minY: number
  maxX: number
  maxY: number
}

interface SectorThumbnailProps {
  bounds: Bounds
  victims?: Victim[]
  drone?: { x: number; y: number } | null
  /** Marks the sector as the one being flown, so a glance separates it from the idle ones. */
  active?: boolean
}

const PRIORITY_TONE: Record<string, string> = {
  P1: '#ef5350', P2: '#ff9f43', P3: '#43d17b', UNTRIAGED: '#8ae0ff',
}
const WIDTH = 120
const HEIGHT = 70
const PADDING = 6

/**
 * A plan view of the sector itself, drawn from its real bounds with whatever has been found in
 * it. The previous tiles were stock satellite photographs of another city, which told an
 * operator nothing about the ground they are searching.
 */
function SectorThumbnail({ bounds, victims = [], drone, active = false }: SectorThumbnailProps) {
  const spanX = Math.max(1, bounds.maxX - bounds.minX)
  const spanY = Math.max(1, bounds.maxY - bounds.minY)
  const scale = Math.min((WIDTH - 2 * PADDING) / spanX, (HEIGHT - 2 * PADDING) / spanY)
  const offsetX = (WIDTH - spanX * scale) / 2
  const offsetY = (HEIGHT - spanY * scale) / 2

  // north is up: the map frame's +y rises, SVG's y falls
  const place = (x: number, y: number): [number, number] => [
    offsetX + (x - bounds.minX) * scale,
    HEIGHT - offsetY - (y - bounds.minY) * scale,
  ]
  const [sectorX, sectorY] = place(bounds.minX, bounds.maxY)

  return (
    <svg width={WIDTH} height={HEIGHT} viewBox={`0 0 ${WIDTH} ${HEIGHT}`} role="img"
         aria-label="Sector plan view" className="rounded border border-white/15 bg-[#26324a]">
      <rect x={sectorX} y={sectorY} width={spanX * scale} height={spanY * scale}
            fill={active ? 'rgba(134,226,164,0.10)' : 'rgba(138,224,255,0.07)'}
            stroke={active ? '#86e2a4' : '#8ae0ff'} strokeWidth={1} strokeDasharray="3 2" />
      {victims.map((victim) => {
        if (!victim.position) return null
        const [cx, cy] = place(victim.position.x, victim.position.y)
        return <circle key={victim.id} cx={cx} cy={cy} r={2.4}
                       fill={PRIORITY_TONE[victim.priority] ?? '#8ae0ff'} />
      })}
      {drone && (() => {
        const [cx, cy] = place(drone.x, drone.y)
        return <circle cx={cx} cy={cy} r={3.2} fill="#42d7c7" stroke="#0d1b26" strokeWidth={1} />
      })()}
    </svg>
  )
}

export default SectorThumbnail
