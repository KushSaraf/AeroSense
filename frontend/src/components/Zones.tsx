import { Polygon, Popup } from 'react-leaflet'
import type { WorldInfo } from '../hooks/useWorld'

/** Red: no network. Orange: no GPS. Purple: neither. As the outlines in Gazebo. */
const ZONE_STYLE = {
  network: { colour: '#ef5350', label: 'No network' },
  gps: { colour: '#ff9800', label: 'No GPS' },
  both: { colour: '#b05cff', label: 'No GPS, no network' },
}

/** Every zone where the drone loses its network, its GPS or both, drawn once in its kind's colour. */
export function DeniedZones({ world }: { world: WorldInfo | undefined | null }) {
  const gps = new Set((world?.noGpsZones ?? []).map((zone) => zone.id))
  const network = new Set((world?.noNetworkZones ?? []).map((zone) => zone.id))
  const zones = [...(world?.noNetworkZones ?? []), ...(world?.noGpsZones ?? []).filter((zone) => !network.has(zone.id))]
  return (
    <>
      {zones.map((zone) => {
        const style = ZONE_STYLE[network.has(zone.id) && gps.has(zone.id) ? 'both' : gps.has(zone.id) ? 'gps' : 'network']
        return (
          <Polygon key={zone.id} positions={zone.corners.map((c): [number, number] => [c.latitude, c.longitude])}
                   pathOptions={{ color: style.colour, fillColor: style.colour, fillOpacity: 0.15, weight: 2, dashArray: '4 6' }}>
            <Popup>{style.label}: {zone.name}</Popup>
          </Polygon>
        )
      })}
    </>
  )
}
