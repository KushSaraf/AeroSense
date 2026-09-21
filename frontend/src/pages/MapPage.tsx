import { useEffect, useRef, useState } from 'react'
import { CircleMarker, MapContainer, Polygon, Polyline, Popup, TileLayer, Tooltip, useMap } from 'react-leaflet'
import { useSearchParams } from 'react-router-dom'
import { useAreaDrawer } from '../components/AreaDrawer'
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
/** Ground routes by the worst hazard they cross. */
const ROUTE_COLOUR: Record<string, string> = { SAFE: '#6ec6ff', MODERATE: '#ffb74d', HIGH: '#ef5350' }
/** Hazard regions by severity. */
const HAZARD_COLOUR: Record<string, string> = { MODERATE: '#ffd166', HIGH: '#ff8c42', CRITICAL: '#e63946' }

/** One colour per ground team's tour. */
const TEAM_COLOUR = ['#ffd166', '#c792ea', '#4dd0e1', '#f78c6c', '#a5d6a7', '#ff80ab']

const minutes = (seconds: number) => `${Math.floor(seconds / 60)} min ${Math.round(seconds % 60)} s`
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
  const { victims, routes, teams, hazards, drone, mission, link } = useMission()
  const { data: world, error } = useWorld()
  const [baseLayer, setBaseLayer] = useState<keyof typeof BASE_MAPS>('Satellite')
  const [layers, setLayers] = useState({ sectors: true, network: true, hazards: true, victims: true, track: true, teams: true, routes: false })
  const area = useAreaDrawer(world)
  // Mission command sends the operator here with ?draw=1 to start drawing an area straight away.
  // The ref makes it one shot: arming allocates fresh drawer state and every dependency here
  // changes identity each render, so without it the effect re-armed forever and span the CPU.
  const [params, setParams] = useSearchParams()
  const armed = useRef(false)
  const startDrawing = area.start
  useEffect(() => {
    if (armed.current || params.get('draw') !== '1') return
    armed.current = true
    startDrawing()
    setParams({}, { replace: true })
  }, [params, setParams, startDrawing])
  const latitude = drone?.latitude
  const longitude = drone?.longitude
  const track = useFlightTrack(latitude, longitude, mission?.id, mission?.elapsedSeconds, TRACK_LIMIT)

  const flownSector = world?.sectors.find((sector) => sector.id === mission?.scenario)
  const centre: [number, number] | null = flownSector
    ? toLatLng(flownSector.centre)
    : world
      ? toLatLng(world.sectors[0].centre)
      : null

  const teamOf = (id: string) => {
    const tour = teams.tours.find((t) => t.victimIds.includes(id))
    if (tour) return `ground team ${tour.team}, stop ${tour.victimIds.indexOf(id) + 1}`
    return teams.unassigned.includes(id) ? 'no ground team reaches them within a shift' : 'not yet planned'
  }

  const toggles = [
    { label: 'Sectors', on: layers.sectors, act: () => setLayers((l) => ({ ...l, sectors: !l.sectors })) },
    { label: 'No network', on: layers.network, act: () => setLayers((l) => ({ ...l, network: !l.network })) },
    { label: 'Hazards', on: layers.hazards, act: () => setLayers((l) => ({ ...l, hazards: !l.hazards })) },
    { label: 'Casualties', on: layers.victims, act: () => setLayers((l) => ({ ...l, victims: !l.victims })) },
    { label: 'Flight track', on: layers.track, act: () => setLayers((l) => ({ ...l, track: !l.track })) },
    { label: 'Team tours', on: layers.teams, act: () => setLayers((l) => ({ ...l, teams: !l.teams })) },
    { label: 'Ground routes', on: layers.routes, act: () => setLayers((l) => ({ ...l, routes: !l.routes })) },
  ]

  return (
    <div className="h-full overflow-hidden bg-[#202635]">
      <div className="flex min-h-[48px] flex-wrap items-center gap-2 border-b border-white/10 bg-[#252e42] px-3 py-2 text-[12px] uppercase tracking-[0.1em] text-text/70">
        {(Object.keys(BASE_MAPS) as Array<keyof typeof BASE_MAPS>).map((name) => (
          <button key={name} type="button" onClick={() => setBaseLayer(name)}
                  className={`rounded border px-3 py-2 transition ${baseLayer === name ? 'border-[#8ae0ff]/60 bg-[#8ae0ff]/20 text-text' : 'border-white/10 bg-white/5 hover:bg-white/10'}`}>{name}</button>
        ))}
        <span className="mx-1 h-5 w-px bg-white/15" />
        {toggles.map(({ label, on, act }) => (
          <button key={label} type="button" onClick={act}
                  className={`rounded border px-3 py-2 transition ${on ? 'border-[#8ae0ff]/60 bg-[#8ae0ff]/20 text-text' : 'border-white/10 bg-white/5 hover:bg-white/10'}`}>{label}</button>
        ))}
        <span className="mx-1 h-5 w-px bg-white/15" />
        <button type="button" onClick={area.start}
                className={`rounded border px-3 py-2 transition ${area.drawing ? 'border-[#ffd166]/60 bg-[#ffd166]/20 text-text' : 'border-white/10 bg-white/5 hover:bg-white/10'}`}>Draw search area</button>
        <span className="ml-auto normal-case tracking-[0.1em] text-text/70">
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
                <Tooltip permanent direction="center" className="sector-label">{sector.name}</Tooltip>
                <Popup>{sector.name}{active ? ' · being searched' : ''}</Popup>
              </Polygon>
            )
          })}

          {layers.network && <DeniedZones world={world} />}
          {area.layer}

          {layers.hazards && hazards.filter((hazard) => hazard.polygon.length > 2).map((hazard) => (
            <Polygon key={hazard.id} positions={hazard.polygon}
                     pathOptions={{ color: HAZARD_COLOUR[hazard.severity], fillColor: HAZARD_COLOUR[hazard.severity],
                                    fillOpacity: 0.3, weight: 1 }}>
              <Popup>
                <strong>{hazard.severity}</strong> {hazard.type.toLowerCase()} hazard
                {hazard.hsi !== null ? ` · HSI ${hazard.hsi.toFixed(2)}` : ''}
                {hazard.detail ? ` · ${hazard.detail}` : ''}<br />
                {hazard.areaM2} m², {hazard.type === 'CHEMICAL'
                  ? 'air the drone flew through over its NIOSH exposure limit'
                  : 'mapped by disaster segmentation'}
                {hazard.severity === 'CRITICAL' ? ' · roads through it are closed to ground teams' : ''}
              </Popup>
            </Polygon>
          ))}

          {layers.routes && routes.filter((route) => route.reachable && route.path.length > 1).map((route) => (
            <Polyline key={`route-${route.victimId}`} positions={route.path}
                      pathOptions={{ color: ROUTE_COLOUR[route.risk] ?? '#6ec6ff', weight: 3, opacity: 0.8, dashArray: '10 6' }}>
              <Popup>
                Ground route to <strong>{route.victimId}</strong> from the command base<br />
                {route.distanceM.toFixed(0)} m by road, about {minutes(route.timeS)} · risk {route.risk.toLowerCase()}
              </Popup>
            </Polyline>
          ))}

          {layers.teams && teams.tours.filter((tour) => tour.path.length > 1).map((tour, i) => (
            <Polyline key={`team-${tour.team}`} positions={tour.path}
                      pathOptions={{ color: TEAM_COLOUR[i % TEAM_COLOUR.length], weight: 4, opacity: 0.85 }}>
              <Popup>
                Team <strong>{tour.team}</strong>: {tour.victimIds.join(' → ')}, back to base<br />
                {tour.distanceM.toFixed(0)} m by road, about {minutes(tour.timeS)} with time on site
              </Popup>
            </Polyline>
          ))}

          {layers.victims && victims.map((victim) => (
            <CircleMarker key={victim.id} center={[victim.latitude, victim.longitude]} radius={7}
                          pathOptions={{ color: PRIORITY_COLOUR[victim.priority] ?? '#8ae0ff',
                                         fillColor: PRIORITY_COLOUR[victim.priority] ?? '#8ae0ff', fillOpacity: 0.85, weight: 2 }}>
              <Popup>
                <strong>{victim.id}</strong> · {victim.priority}<br />
                {(victim.confidence * 100).toFixed(0)}% confident, {victim.thermalStrength} thermal<br />
                {victim.rationale}<br />
                {teamOf(victim.id)}<br />
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

        {area.panel}
        {!error && victims.length === 0 && (
          <div className="pointer-events-none absolute bottom-4 left-4 rounded border border-white/15 bg-[#202635]/90 px-3 py-2 text-[12px] uppercase tracking-[0.08em] text-white/75">
            No casualties detected yet
          </div>
        )}
      </div>
    </div>
  )
}

export default MapPage
