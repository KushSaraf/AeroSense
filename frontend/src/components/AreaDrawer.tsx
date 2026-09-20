import { useCallback, useEffect, useRef, useState } from 'react'
import type { LeafletMouseEvent } from 'leaflet'
import { Rectangle, useMapEvents } from 'react-leaflet'
import { startCustomArea } from '../services/apiServices'
import type { LatLon, WorldInfo } from '../hooks/useWorld'

const AREA_COLOUR = '#ffd166'

/** Width and height in metres of the rectangle two corners span, with the bridge's own conversion. */
const sizeOf = (a: LatLon, b: LatLon, world: WorldInfo): [number, number] => [
  Math.abs(a.longitude - b.longitude) * world.metresPerDegree.longitude,
  Math.abs(a.latitude - b.latitude) * world.metresPerDegree.latitude,
]

/** A press closer than this to the release is a click (corner by corner), not a drag. */
const DRAG_MIN_PX = 6

/**
 * Drawing inside the MapContainer: press, drag and release, or click one corner then the other.
 * The rectangle follows the mouse either way, and the map holds still while drawing.
 */
function DrawInteraction({ drawing, anchor, onAnchor, onHover, onDone, onCancel }: {
  drawing: boolean
  anchor: LatLon | null
  onAnchor: (corner: LatLon) => void
  onHover: (corner: LatLon) => void
  onDone: (corner: LatLon) => void
  onCancel: () => void
}) {
  const pressedAt = useRef<{ x: number; y: number } | null>(null)
  const isFirstPress = useRef(true)
  const toLatLon = (event: LeafletMouseEvent): LatLon => ({ latitude: event.latlng.lat, longitude: event.latlng.lng })
  const map = useMapEvents({
    mousedown: (event) => {
      if (!drawing) return
      pressedAt.current = { x: event.containerPoint.x, y: event.containerPoint.y }
      if (!anchor) onAnchor(toLatLon(event))
    },
    mousemove: (event) => { if (drawing && anchor) onHover(toLatLon(event)) },
    mouseup: (event) => {
      if (!drawing || !anchor || !pressedAt.current) return
      const moved = Math.hypot(event.containerPoint.x - pressedAt.current.x, event.containerPoint.y - pressedAt.current.y)
      pressedAt.current = null
      // a drag ends here; a click on the first corner waits for the second click
      if (moved >= DRAG_MIN_PX || !isFirstPress.current) onDone(toLatLon(event))
      isFirstPress.current = false
    },
  })
  useEffect(() => {
    isFirstPress.current = true
    const container = map.getContainer()
    if (!drawing) return undefined
    map.dragging.disable()
    map.doubleClickZoom.disable()
    container.style.cursor = 'crosshair'
    const escape = (event: KeyboardEvent) => { if (event.key === 'Escape') onCancel() }
    window.addEventListener('keydown', escape)
    return () => {
      map.dragging.enable()
      map.doubleClickZoom.enable()
      container.style.cursor = ''
      window.removeEventListener('keydown', escape)
    }
  }, [drawing, map, onCancel])
  return null
}

/**
 * Draw a search area on the map and fly it. The corners go to the bridge as latitude and
 * longitude; it converts them to metres on the world origin and the mission manager searches that
 * rectangle, GPS or not.
 */
export function useAreaDrawer(world: WorldInfo | null) {
  const [drawing, setDrawing] = useState(false)
  const [corners, setCorners] = useState<LatLon[]>([])
  const [sending, setSending] = useState(false)
  const [message, setMessage] = useState<{ ok: boolean; text: string } | null>(null)

  const [anchor, setAnchor] = useState<LatLon | null>(null)
  const [hover, setHover] = useState<LatLon | null>(null)

  const cancel = useCallback(() => { setDrawing(false); setAnchor(null); setHover(null) }, [])
  const done = useCallback((corner: LatLon) => {
    setCorners(anchor ? [anchor, corner] : [])
    setDrawing(false)
    setAnchor(null)
    setHover(null)
  }, [anchor])
  const clear = () => { setCorners([]); setMessage(null) }
  const start = () => { setCorners([]); setMessage(null); setAnchor(null); setHover(null); setDrawing(true) }

  const fly = async () => {
    setSending(true)
    setMessage(null)
    try {
      const answer = await startCustomArea(corners)
      setMessage({ ok: answer.success, text: answer.message })
      if (answer.success) setCorners([])
    } catch (cause: unknown) {
      setMessage({ ok: false, text: cause instanceof Error ? cause.message : 'the bridge did not answer' })
    } finally {
      setSending(false)
    }
  }

  // while drawing, the rectangle runs from the first corner to the mouse
  const shown = corners.length === 2 ? corners : anchor && hover ? [anchor, hover] : []
  const layer = (
    <>
      <DrawInteraction drawing={drawing} anchor={anchor} onAnchor={setAnchor} onHover={setHover}
                       onDone={done} onCancel={cancel} />
      {shown.length === 2 && (
        <Rectangle bounds={shown.map((c) => [c.latitude, c.longitude] as [number, number])}
                   pathOptions={{ color: AREA_COLOUR, fillColor: AREA_COLOUR, fillOpacity: 0.12, weight: 2, dashArray: '8 6' }} />
      )}
    </>
  )

  const size = shown.length === 2 && world ? sizeOf(shown[0], shown[1], world) : null
  const panel = (drawing || corners.length > 0 || message) && (
    <div className="absolute right-4 top-4 z-[1000] w-[250px] rounded border border-white/15 bg-[#202635]/95 p-3 text-[12px] uppercase tracking-[0.08em] text-white/80">
      <div className="font-semibold text-white">Search area</div>
      {drawing && <div className="mt-2 normal-case tracking-normal text-white/70">
        {anchor ? 'Release, or click, on the opposite corner' : 'Press on one corner and drag across the area'} · Esc cancels
      </div>}
      {size && <div className="mt-2 normal-case tracking-normal">{size[0].toFixed(0)} × {size[1].toFixed(0)} m</div>}
      {message && <div role="status" className={`mt-2 normal-case tracking-normal ${message.ok ? 'text-[#86e2a4]' : 'text-[#ffb9bf]'}`}>{message.text}</div>}
      <div className="mt-3 flex gap-2">
        {corners.length === 2 && (
          <button type="button" disabled={sending} onClick={() => void fly()}
                  className="flex-1 rounded bg-[#86e2a4]/25 px-2 py-2 text-[#c8f5d6] transition hover:bg-[#86e2a4]/40 disabled:opacity-40">
            {sending ? 'Sending…' : 'Fly this area'}
          </button>
        )}
        <button type="button" onClick={() => { clear(); cancel() }}
                className="flex-1 rounded bg-white/10 px-2 py-2 transition hover:bg-white/20">
          {corners.length || drawing ? 'Cancel' : 'Close'}
        </button>
      </div>
    </div>
  )

  return { drawing, start, layer, panel }
}
