import { useEffect, useRef, useState } from 'react'

type Point = [number, number]

/**
 * The path the drone has flown, built from the positions that arrive rather than from a stored
 * route. It starts again when a new mission begins or the mission clock runs backwards (a restart,
 * or the replay looping): without that, the previous flight's whole track stayed drawn while the
 * drone sat on the pad.
 */
export const useFlightTrack = (
  latitude: number | null | undefined,
  longitude: number | null | undefined,
  missionId: string | undefined,
  elapsedSeconds: number | undefined,
  limit: number,
): Point[] => {
  const [track, setTrack] = useState<Point[]>([])
  const seen = useRef({ missionId, elapsed: elapsedSeconds ?? 0 })

  useEffect(() => {
    const elapsed = elapsedSeconds ?? 0
    const restarted = missionId !== seen.current.missionId || elapsed < seen.current.elapsed
    seen.current = { missionId, elapsed }
    if (restarted) setTrack([])
  }, [missionId, elapsedSeconds])

  useEffect(() => {
    if (latitude == null || longitude == null) return
    setTrack((flown) => {
      const last = flown[flown.length - 1]
      if (last && Math.abs(last[0] - latitude) < 1e-6 && Math.abs(last[1] - longitude) < 1e-6) return flown
      return [...flown, [latitude, longitude] as Point].slice(-limit)
    })
  }, [latitude, longitude, limit])

  return track
}
