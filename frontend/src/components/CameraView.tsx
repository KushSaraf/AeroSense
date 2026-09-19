import { useEffect, useRef, useState } from 'react'
import { cameraSrc, webrtcAnswer } from '../services/apiServices'
import { IS_REPLAY } from '../services/replay'

type Camera = 'rgb' | 'thermal'
/** Give up on WebRTC and show the MJPEG stream if no video has started by then. */
const CONNECT_TIMEOUT_MS = 8000

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

  if (fallback) return <img src={cameraSrc(camera)} alt={`${camera} camera`} className={className} />
  return <video ref={video} autoPlay muted playsInline aria-label={`${camera} camera`} className={className} />
}

export default CameraView
