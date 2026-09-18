import { useEffect, useRef, useState } from 'react'
import { CircleMarker, MapContainer, Polygon, Polyline, Popup, TileLayer, useMap } from 'react-leaflet'
import { LINK_COLOUR } from '../components/NetworkStatus'
import { DeniedZones } from '../components/Zones'
import { useFlightTrack } from '../hooks/useFlightTrack'
import { useMission } from '../hooks/useMission'
import { useWorld } from '../hooks/useWorld'
import type { LatLon } from '../hooks/useWorld'
import 'leaflet/dist/leaflet.css'

const PRIORITY_COLOUR: Record<string, string> = {
  P1: '#ef5350', P2: '#ff9f43', P3: '#43d17b', UNTRIAGED: '#8ae0ff',
}
const TRACK_LIMIT = 600
const BASE_MAPS = {
  Satellite: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  Street: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
  Terrain: 'https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png',
} as const

const toLatLng = (point: LatLon): [number, number] => [point.latitude, point.longitude]

/** Recentres once the world is known, rather than leaving the map wherever it opened. */
function Recentre({ centre }: { centre: [number, number] | null }) {
  const map = useMap()
  const done = useRef(false)
  useEffect(() => {
    if (!centre || done.current) return
    map.setView(centre, 17)
    done.current = true
  }, [centre, map])
  return null
}

function MapPage() {
  const { victims, drone, mission, link } = useMission()
  const { data: world, error } = useWorld()
  const [baseLayer, setBaseLayer] = useState<keyof typeof BASE_MAPS>('Satellite')
  const [layers, setLayers] = useState({ sectors: true, network: true, victims: true, track: true })
  const latitude = drone?.latitude
  const longitude = drone?.longitude
  const track = useFlightTrack(latitude, longitude, mission?.id, mission?.elapsedSeconds, TRACK_LIMIT)

  const flownSector = world?.sectors.find((sector) => sector.id === mission?.scenario)
  const centre: [number, number] | null = flownSector
    ? toLatLng(flownSector.centre)
    : world
      ? toLatLng(world.sectors[0].centre)
      : null

  const toggles = [
    { label: 'Sectors', on: layers.sectors, act: () => setLayers((l) => ({ ...l, sectors: !l.sectors })) },
    { label: 'No network', on: layers.network, act: () => setLayers((l) => ({ ...l, network: !l.network })) },
    { label: 'Casualties', on: layers.victims, act: () => setLayers((l) => ({ ...l, victims: !l.victims })) },
    { label: 'Flight track', on: layers.track, act: () => setLayers((l) => ({ ...l, track: !l.track })) },
  ]

  return (
    <div className="h-full overflow-hidden bg-[#202635]">
      <div className="flex min-h-[48px] flex-wrap items-center gap-2 border-b border-white/10 bg-[#252e42] px-3 py-2 text-[10px] uppercase tracking-[0.18em] text-text/70">
        {(Object.keys(BASE_MAPS) as Array<keyof typeof BASE_MAPS>).map((name) => (
          <button key={name} type="button" onClick={() => setBaseLayer(name)}
                  className={`rounded border px-3 py-2 transition ${baseLayer === name ? 'border-[#8ae0ff]/60 bg-[#8ae0ff]/20 text-text' : 'border-white/10 bg-white/5 hover:bg-white/10'}`}>{name}</button>
        ))}
        <span className="mx-1 h-5 w-px bg-white/15" />
        {toggles.map(({ label, on, act }) => (
          <button key={label} type="button" onClick={act}
                  className={`rounded border px-3 py-2 transition ${on ? 'border-[#8ae0ff]/60 bg-[#8ae0ff]/20 text-text' : 'border-white/10 bg-white/5 hover:bg-white/10'}`}>{label}</button>
        ))}
        <span className="ml-auto normal-case tracking-[0.1em] text-text/50">
          {world ? `${world.world} · origin ${world.origin.latitude.toFixed(5)}, ${world.origin.longitude.toFixed(5)}` : error ?? 'loading world…'}
        </span>
      </div>
      <div className="relative h-[calc(100%-48px)] overflow-hidden border-t border-white/10">
        <MapContainer center={centre ?? [0, 0]} zoom={centre ? 17 : 3} scrollWheelZoom style={{ height: '100%', width: '100%' }}>
          <TileLayer attribution="&copy; OpenStreetMap contributors" url={BASE_MAPS[baseLayer]} />
          <Recentre centre={centre} />

          {layers.sectors && world?.sectors.map((sector) => {
            const active = sector.id === mission?.scenario
            return (
              <Polygon key={sector.id} positions={sector.corners.map(toLatLng)}
                       pathOptions={{ color: active ? '#86e2a4' : '#8ae0ff', fillColor: active ? '#86e2a4' : '#8ae0ff',
                                      fillOpacity: active ? 0.12 : 0.05, weight: 2, dashArray: active ? undefined : '6 6' }}>
                <Popup>{sector.name}{active ? ' · being searched' : ''}</Popup>
              </Polygon>
            )
          })}

          {layers.network && <DeniedZones world={world} />}

          {layers.victims && victims.map((victim) => (
            <CircleMarker key={victim.id} center={[victim.latitude, victim.longitude]} radius={7}
                          pathOptions={{ color: PRIORITY_COLOUR[victim.priority] ?? '#8ae0ff',
                                         fillColor: PRIORITY_COLOUR[victim.priority] ?? '#8ae0ff', fillOpacity: 0.85, weight: 2 }}>
              <Popup>
                <strong>{victim.id}</strong> · {victim.priority}<br />
                {(victim.confidence * 100).toFixed(0)}% confident, {victim.thermalStrength} thermal<br />
                {victim.rationale}<br />
                found {victim.timestamp}
              </Popup>
            </CircleMarker>
          ))}

          {layers.track && track.length > 1 && (
            <Polyline positions={track} pathOptions={{ color: '#A6D9E7', weight: 3 }} />
          )}

          {latitude != null && longitude != null && (
            <CircleMarker center={[latitude, longitude]} radius={9}
                          pathOptions={{ color: '#ffffff', fillColor: LINK_COLOUR[link?.state ?? 'UNKNOWN'], fillOpacity: 1, weight: 3 }}>
              <Popup>{drone?.name} · {drone?.mode ?? 'unknown mode'} · {drone?.altitude.toFixed(1)} m · network {link?.state ?? 'unknown'}</Popup>
            </CircleMarker>
          )}
        </MapContainer>

        {!error && victims.length === 0 && (
          <div className="pointer-events-none absolute bottom-4 left-4 rounded border border-white/15 bg-[#202635]/90 px-3 py-2 text-[10px] uppercase tracking-[0.14em] text-white/60">
            No casualties detected yet
          </div>
        )}
      </div>
    </div>
  )
}

export default MapPage
