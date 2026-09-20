import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { useMission } from '../hooks/useMission'
import { apiService } from '../services/apiServices'
import type { Scene3d } from '../types'

const SCENE_POLL_MS = 1000
const TRAIL_LIMIT = 600
const PRIORITY_COLOUR: Record<string, number> = { P1: 0xef5350, P2: 0xff9f43, P3: 0x43d17b }
const UNTRIAGED_COLOUR = 0x8ae0ff
/** Feature points coloured by height: ground blue through roof-top yellow. */
const POINT_LOW_M = 0
const POINT_HIGH_M = 20

/** Map frame (ENU: x east, y north, z up) to three.js (y up). */
const toThree = (x: number, y: number, z: number) => new THREE.Vector3(x, z, -y)

/** Replaces a group's children, disposing what they held. */
function refill(group: THREE.Group, children: THREE.Object3D[]) {
  group.children.forEach((child) => {
    if (child instanceof THREE.Mesh || child instanceof THREE.Points || child instanceof THREE.Line) {
      child.geometry.dispose()
      ;(child.material as THREE.Material).dispose()
    }
  })
  group.clear()
  children.forEach((child) => group.add(child))
}

function pointCloud(points: [number, number, number][]): THREE.Points {
  const positions = new Float32Array(points.length * 3)
  const colours = new Float32Array(points.length * 3)
  const colour = new THREE.Color()
  points.forEach(([x, y, z], i) => {
    const v = toThree(x, y, z)
    positions.set([v.x, v.y, v.z], i * 3)
    const t = Math.min(1, Math.max(0, (z - POINT_LOW_M) / (POINT_HIGH_M - POINT_LOW_M)))
    colour.setHSL(0.6 - 0.45 * t, 0.9, 0.55)
    colours.set([colour.r, colour.g, colour.b], i * 3)
  })
  const geometry = new THREE.BufferGeometry()
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
  geometry.setAttribute('color', new THREE.BufferAttribute(colours, 3))
  return new THREE.Points(geometry, new THREE.PointsMaterial({ size: 0.6, vertexColors: true }))
}

/** A beam return: a small red marker where the drone found something its map did not have. */
function obstacleMarkers(points: [number, number, number][]): THREE.Points {
  const positions = new Float32Array(points.length * 3)
  points.forEach(([x, y, z], i) => {
    const v = toThree(x, y, z)
    positions.set([v.x, v.y, v.z], i * 3)
  })
  const geometry = new THREE.BufferGeometry()
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
  return new THREE.Points(geometry, new THREE.PointsMaterial({ size: 2.0, color: 0xef5350 }))
}

function structureMeshes(scene: Scene3d): THREE.Mesh[] {
  return scene.structures.filter((s) => s.heightM > 0).map((s) => {
    const mesh = new THREE.Mesh(
      new THREE.CylinderGeometry(s.radiusM, s.radiusM, s.heightM, 24, 1, true),
      new THREE.MeshBasicMaterial({ color: 0x8ea3c7, transparent: true, opacity: 0.14, side: THREE.DoubleSide, depthWrite: false }),
    )
    mesh.position.copy(toThree(s.x, s.y, s.heightM / 2))
    return mesh
  })
}

/**
 * The drone's local 3D view: OpenVINS's feature points (what its stereo cameras have triangulated,
 * placed in the map by the same fit it navigates on), the structures it plans round, the
 * casualties it has confirmed and its own track. Everything comes from the bridge.
 */
function ScenePage() {
  const { drone, victims } = useMission()
  const mount = useRef<HTMLDivElement>(null)
  const layers = useRef<{ points: THREE.Group; obstacles: THREE.Group; structures: THREE.Group; victims: THREE.Group; trail: THREE.Line; drone: THREE.Mesh } | null>(null)
  const controls = useRef<OrbitControls | null>(null)
  const trail = useRef<THREE.Vector3[]>([])
  const [scene, setScene] = useState<Scene3d | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [follow, setFollow] = useState(true)

  // the renderer, once
  useEffect(() => {
    const host = mount.current
    if (!host) return
    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.setPixelRatio(window.devicePixelRatio)
    host.appendChild(renderer.domElement)
    const world = new THREE.Scene()
    world.background = new THREE.Color(0x1b2130)
    const camera = new THREE.PerspectiveCamera(55, 1, 0.5, 3000)
    camera.position.set(-60, 80, 40)
    const orbit = new OrbitControls(camera, renderer.domElement)
    orbit.target.copy(toThree(0, -110, 0))
    controls.current = orbit
    world.add(new THREE.GridHelper(600, 60, 0x3b4a66, 0x283247))
    world.add(new THREE.AxesHelper(10))

    const groups = { points: new THREE.Group(), obstacles: new THREE.Group(), structures: new THREE.Group(), victims: new THREE.Group() }
    Object.values(groups).forEach((group) => world.add(group))
    const trailLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xa6d9e7 }))
    const droneMesh = new THREE.Mesh(new THREE.SphereGeometry(1.2, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffffff }))
    droneMesh.visible = false
    world.add(trailLine, droneMesh)
    layers.current = { ...groups, trail: trailLine, drone: droneMesh }

    const resize = () => {
      const { clientWidth, clientHeight } = host
      renderer.setSize(clientWidth, clientHeight)
      camera.aspect = clientWidth / Math.max(1, clientHeight)
      camera.updateProjectionMatrix()
    }
    resize()
    const observer = new ResizeObserver(resize)
    observer.observe(host)
    renderer.setAnimationLoop(() => {
      orbit.update()
      renderer.render(world, camera)
    })
    return () => {
      renderer.setAnimationLoop(null)
      observer.disconnect()
      orbit.dispose()
      Object.values(groups).forEach((group) => refill(group, []))
      renderer.dispose()
      host.removeChild(renderer.domElement)
      layers.current = null
    }
  }, [])

  // structures and feature points from the bridge
  useEffect(() => {
    let cancelled = false
    const poll = async () => {
      try {
        const next = await apiService.getScene()
        if (!cancelled) { setScene(next); setError(null) }
      } catch (reason) {
        if (!cancelled) setError(`no 3D scene from the bridge (${String(reason)})`)
      }
    }
    void poll()
    const timer = setInterval(() => void poll(), SCENE_POLL_MS)
    return () => { cancelled = true; clearInterval(timer) }
  }, [])

  useEffect(() => {
    const l = layers.current
    if (!l || !scene) return
    if (l.structures.children.length === 0) refill(l.structures, structureMeshes(scene))
    refill(l.points, scene.points.length ? [pointCloud(scene.points)] : [])
    const obstacles = scene.obstacles ?? []
    refill(l.obstacles, obstacles.length ? [obstacleMarkers(obstacles)] : [])
  }, [scene])

  useEffect(() => {
    const l = layers.current
    if (!l) return
    refill(l.victims, victims.filter((v) => v.position).map((v) => {
      const p = v.position as { x: number; y: number; z: number }
      const mesh = new THREE.Mesh(new THREE.SphereGeometry(1.0, 12, 8),
        new THREE.MeshBasicMaterial({ color: PRIORITY_COLOUR[v.priority] ?? UNTRIAGED_COLOUR }))
      mesh.position.copy(toThree(p.x, p.y, p.z + 1))
      return mesh
    }))
  }, [victims])

  const position = drone?.position
  useEffect(() => {
    const l = layers.current
    if (!l || !position) return
    const here = toThree(position.x, position.y, position.z)
    l.drone.visible = true
    l.drone.position.copy(here)
    const last = trail.current[trail.current.length - 1]
    if (!last || last.distanceTo(here) > 0.5) {
      trail.current = [...trail.current, here].slice(-TRAIL_LIMIT)
      l.trail.geometry.dispose()
      l.trail.geometry = new THREE.BufferGeometry().setFromPoints(trail.current)
    }
    if (follow && controls.current) {
      const orbit = controls.current
      const shift = here.clone().sub(orbit.target)
      orbit.target.copy(here)
      orbit.object.position.add(shift)
    }
  }, [position, follow])

  const points = scene?.points.length ?? 0
  const obstacles = scene?.obstacles?.length ?? 0
  return (
    <div className="relative h-full overflow-hidden bg-[#1b2130]">
      <div ref={mount} className="absolute inset-0" />
      <div className="pointer-events-none absolute left-4 top-4 max-w-sm rounded border border-white/15 bg-[#202635]/90 px-3 py-2 text-[11px] leading-5 text-white/75">
        <div className="mb-1 text-[10px] font-bold uppercase tracking-[0.16em] text-white">Local 3D view</div>
        {error ?? (points
          ? `${points} feature points OpenVINS has triangulated from the stereo cameras, coloured by height`
          : 'No feature points yet: OpenVINS starts in the hover after take-off, and its points are placed once its track fits GPS')}
        <br />Cylinders: structures the drone plans round · spheres: casualties by priority · white: the drone and its track
        {obstacles > 0 && <><br />{`Red: ${obstacles} obstacle${obstacles === 1 ? '' : 's'} the rangefinder beams found, which the mission now routes round`}</>}
      </div>
      <button type="button" onClick={() => setFollow((f) => !f)}
              className={`absolute right-4 top-4 rounded border px-3 py-2 text-[10px] uppercase tracking-[0.16em] ${follow ? 'border-[#8ae0ff]/60 bg-[#8ae0ff]/20 text-white' : 'border-white/15 bg-white/5 text-white/70'}`}>
        Follow drone
      </button>
    </div>
  )
}

export default ScenePage
