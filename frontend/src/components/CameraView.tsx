import { useEffect, useRef, useState } from 'react'
import { cameraSrc, webrtcAnswer } from '../services/apiServices'
import { IS_REPLAY } from '../services/replay'

type Camera = 'rgb' | 'thermal'
/** Give up on WebRTC and show the MJPEG stream if no video has started by then. */
const CONNECT_TIMEOUT_MS = 8000
/** After the stream fails, try it again this often: the bridge, the link or the simulation may
 *  come back, and the operator should not have to reload to see video again. */
const RETRY_MS = 5000

/** Resolves once the peer has gathered its (host) candidates, so the offer carries them all. */
const gathered = (peer: RTCPeerConnection) =>
  new Promise<void>((resolve) => {
    if (peer.iceGatheringState === 'complete') return resolve()
    peer.addEventListener('icegatheringstatechange', () => {
      if (peer.iceGatheringState === 'complete') resolve()
    })
  })

/**
 * The drone's camera over WebRTC from the bridge. If WebRTC fails (no browser support, the bridge
 * refuses, nothing arrives in time) it falls back to the bridge's MJPEG stream; a replay shows its
 * recorded snapshot.
 */
function CameraView({ camera, className }: { camera: Camera; className: string }) {
  const video = useRef<HTMLVideoElement>(null)
  const [fallback, setFallback] = useState(IS_REPLAY || typeof RTCPeerConnection === 'undefined')

  useEffect(() => {
    if (fallback) return
    const peer = new RTCPeerConnection()
    let playing = false
    const timer = setTimeout(() => { if (!playing) setFallback(true) }, CONNECT_TIMEOUT_MS)
    peer.addTransceiver('video', { direction: 'recvonly' })
    peer.ontrack = (event) => {
      if (video.current) video.current.srcObject = event.streams[0] ?? new MediaStream([event.track])
    }
    peer.onconnectionstatechange = () => {
      if (peer.connectionState === 'failed') setFallback(true)
    }
    const element = video.current
    const onPlaying = () => { playing = true }
    element?.addEventListener('playing', onPlaying)

    const negotiate = async () => {
      await peer.setLocalDescription(await peer.createOffer())
      await gathered(peer)
      const local = peer.localDescription
      if (!local) throw new Error('no local description')
      await peer.setRemoteDescription(await webrtcAnswer(camera, { sdp: local.sdp, type: local.type }))
    }
    negotiate().catch((error: unknown) => {
      console.warn(`WebRTC ${camera} camera: ${String(error)}; showing MJPEG instead`)
      setFallback(true)
    })

    return () => {
      clearTimeout(timer)
      element?.removeEventListener('playing', onPlaying)
      peer.close()
    }
  }, [camera, fallback])

  if (fallback) return <StillFeed camera={camera} className={className} />
  return <video ref={video} autoPlay muted playsInline aria-label={`${camera} camera`} className={className} />
}

/**
 * The MJPEG stream (or a replay's recorded snapshot). When it cannot be fetched - the bridge is
 * stopped, the drone's link is down, no simulation is running - the browser drew its own broken-
 * image icon with the alt text, which read as a fault in the dashboard. This says what is missing
 * instead, and tries again.
 */
function StillFeed({ camera, className }: { camera: Camera; className: string }) {
  const [failed, setFailed] = useState(false)
  const src = cameraSrc(camera)

  useEffect(() => {
    if (!failed) return undefined
    const timer = setTimeout(() => setFailed(false), RETRY_MS)
    return () => clearTimeout(timer)
  }, [failed])

  if (failed || !src) {
    return (
      <div role="img" aria-label={`${camera} camera: no video`}
           className={`${className} flex flex-col items-center justify-center gap-1 bg-[#1b2333] px-3 text-center`}>
        <span className="text-[12px] font-bold uppercase tracking-[0.08em] text-white/75">No {camera === 'rgb' ? 'video' : 'thermal'}</span>
        <span className="text-[11px] leading-4 text-white/55">
          {IS_REPLAY ? 'no recorded frame at this moment' : "the drone's camera is not reaching the dashboard"}
        </span>
      </div>
    )
  }
  return <img src={src} alt={`${camera} camera`} className={className} onError={() => setFailed(true)} />
}

export default CameraView
