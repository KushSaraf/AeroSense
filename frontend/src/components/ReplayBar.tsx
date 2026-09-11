import { Pause, Play, RotateCcw } from 'lucide-react'
import { useEffect, useState } from 'react'
import { replayControls, replayStatus } from '../services/replay'

const SPEEDS = [1, 2, 4, 8] as const
const TICK_MS = 500

const clockLabel = (seconds: number) =>
  `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(Math.floor(seconds % 60)).padStart(2, '0')}`

/**
 * Says, on every page, that this is a recorded flight and not a live one, and lets a viewer
 * pause it or speed it up. A replay presented without this would be the fabricated dashboard
 * the project set out not to be.
 */
function ReplayBar() {
  const [status, setStatus] = useState(replayStatus())

  useEffect(() => {
    const timer = setInterval(() => setStatus(replayStatus()), TICK_MS)
    return () => clearInterval(timer)
  }, [])

  const button = 'flex items-center gap-1 rounded border border-white/20 px-2 py-1 transition hover:bg-white/10'
  const recorded = status.recordedAt ? new Date(status.recordedAt).toLocaleString() : ''
  return (
    <div className="flex flex-wrap items-center gap-3 border-b border-amber-300/30 bg-amber-400/10 px-5 py-2 text-[10px] uppercase tracking-[0.14em] text-amber-100">
      <span className="rounded bg-amber-400/25 px-2 py-1 font-bold">Recorded flight replay</span>
      <span className="normal-case tracking-normal text-amber-100/80">
        {status.loaded
          ? `Mission ${status.missionId}, flown in the simulation ${recorded}. Every number below is what the drone reported during that flight.`
          : 'Loading the recorded flight…'}
      </span>
      <span className="ml-auto font-mono text-[12px]">{clockLabel(status.t)} / {clockLabel(status.durationS)}</span>
      <button type="button" className={button} onClick={() => { replayControls.togglePause(); setStatus(replayStatus()) }}
              aria-label={status.paused ? 'Play' : 'Pause'}>
        {status.paused ? <Play size={12} /> : <Pause size={12} />}
      </button>
      <button type="button" className={button} onClick={() => { replayControls.restart(); setStatus(replayStatus()) }}
              aria-label="Restart replay">
        <RotateCcw size={12} />
      </button>
      {SPEEDS.map((speed) => (
        <button key={speed} type="button" onClick={() => { replayControls.setSpeed(speed); setStatus(replayStatus()) }}
                className={`${button} ${status.speed === speed ? 'bg-amber-400/30 text-white' : ''}`}>
          {speed}×
        </button>
      ))}
    </div>
  )
}

export default ReplayBar
