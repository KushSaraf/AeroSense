import { useState } from 'react'
import { Circle, CircleMarker, MapContainer, Marker, Polygon, Polyline, Popup, TileLayer } from 'react-leaflet'
import { mapCenter, hazards, victims } from '../data/mockMissionData'
import 'leaflet/dist/leaflet.css'

const path = [
  [28.612, 77.216],
  [28.614, 77.219],
  [28.616, 77.223],
  [28.613, 77.227],
] as const

function MapPage() {
  const [baseLayer, setBaseLayer] = useState('Street')
  const [activeLayers, setActiveLayers] = useState({
    hazards: true,
    victims: true,
    dronePath: true,
    coverage: false,
    safeRoutes: false,
    sensor3d: false,
  })
  const mapPosition: [number, number] = [mapCenter.lat, mapCenter.lng]
  const toggleLayer = (layer: keyof typeof activeLayers) => {
    setActiveLayers((current) => ({ ...current, [layer]: !current[layer] }))
  }

  const buttons = [
    { label: 'Satellite', action: () => setBaseLayer('Satellite'), active: baseLayer === 'Satellite' },
    { label: 'Terrain', action: () => setBaseLayer('Terrain'), active: baseLayer === 'Terrain' },
    { label: 'Orthomosaic', action: () => setBaseLayer('Orthomosaic'), active: baseLayer === 'Orthomosaic' },
    { label: 'Hazards', action: () => toggleLayer('hazards'), active: activeLayers.hazards },
    { label: 'Victims', action: () => toggleLayer('victims'), active: activeLayers.victims },
    { label: 'Drone Path', action: () => toggleLayer('dronePath'), active: activeLayers.dronePath },
    { label: 'Coverage', action: () => toggleLayer('coverage'), active: activeLayers.coverage },
    { label: 'Safe Routes', action: () => toggleLayer('safeRoutes'), active: activeLayers.safeRoutes },
    { label: '3D', action: () => toggleLayer('sensor3d'), active: activeLayers.sensor3d },
  ]

  const baseMaps = {
    Street: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
    Satellite: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    Terrain: 'https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png',
    Orthomosaic: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
  }

  return (
    <div className="h-full overflow-hidden bg-[#202635]">
      <div className="flex min-h-[48px] flex-wrap items-center gap-2 border-b border-white/10 bg-[#252e42] px-3 py-2 text-[10px] uppercase tracking-[0.18em] text-text/70">
        {buttons.map(({ label, action, active }) => (
          <button key={label} type="button" onClick={action} className={`rounded border px-3 py-2 transition ${active ? 'border-[#8ae0ff]/60 bg-[#8ae0ff]/20 text-text' : 'border-white/10 bg-white/5 hover:bg-white/10'}`}>{label}</button>
        ))}
      </div>
      <div className="relative h-[calc(100%-48px)] overflow-hidden border-t border-white/10">
        <MapContainer center={mapPosition} zoom={14} scrollWheelZoom style={{ height: '100%', width: '100%' }}>
          <TileLayer
            attribution='&copy; OpenStreetMap contributors'
            url={baseMaps[baseLayer as keyof typeof baseMaps]}
          />
          {activeLayers.victims && victims.map((victim) => (
            <Marker key={victim.id} position={[victim.latitude, victim.longitude]}>
              <Popup>{victim.id} · {victim.priority}</Popup>
            </Marker>
          ))}
          {activeLayers.hazards && hazards.map((hazard) => (
            <Polygon
              key={hazard.id}
              positions={hazard.coords as unknown as [number, number][]}
              pathOptions={{
                color: hazard.severity === 'CRITICAL' ? '#C96B68' : hazard.severity === 'HIGH' ? '#D68E58' : '#C7B16A',
                fillColor: hazard.severity === 'CRITICAL' ? '#C96B68' : hazard.severity === 'HIGH' ? '#D68E58' : '#C7B16A',
                fillOpacity: 0.2,
                weight: 2,
              }}
            />
          ))}
          {activeLayers.dronePath && <Polyline positions={path as unknown as [number, number][]} pathOptions={{ color: '#A6D9E7', weight: 4, dashArray: '8 8' }} />}
          {activeLayers.coverage && <Circle center={[28.6146, 77.222]} radius={360} pathOptions={{ color: '#8AE0FF', fillColor: '#8AE0FF', fillOpacity: 0.12, weight: 2, dashArray: '6 8' }} />}
          {activeLayers.safeRoutes && <Polyline positions={[[28.6135, 77.2186], [28.6142, 77.2202], [28.6148, 77.2213], [28.6139, 77.2208]]} pathOptions={{ color: '#86e2a4', weight: 5 }} />}
          <CircleMarker center={[28.6139, 77.2208]} radius={activeLayers.sensor3d ? 16 : 8} pathOptions={{ color: '#8AE0FF', fillColor: '#8AE0FF', fillOpacity: 1, weight: activeLayers.sensor3d ? 3 : 1 }} />
        </MapContainer>
      </div>
    </div>
  )
}

export default MapPage
