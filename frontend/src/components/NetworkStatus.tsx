import { Wifi, WifiOff } from 'lucide-react'
import { useState } from 'react'
import { simulationControl } from '../services/apiServices'
import type { LinkStatus } from '../types'

export const LINK_COLOUR: Record<string, string> = {
  CONNECTED: '#43d17b', DEGRADED: '#ffb347', OFFLINE: '#ef5350', UNKNOWN: '#8ae0ff',
}

/**
 * What the ground knows about the drone's network. While it is down the ground hears nothing, so
 * this says how long it has been silent, not what the drone is doing: that arrives on reconnect.
 */
export function NetworkBanner({ link }: { link: LinkStatus | null }) {
  if (!link || link.state === 'CONNECTED' || link.state === 'UNKNOWN') return null
  const offline = link.state === 'OFFLINE'
  return (
    <div className={`flex items-center gap-3 rounded-lg border px-4 py-2 text-[13px] uppercase tracking-[0.08em] ${
      offline ? 'border-red-400/50 bg-red-500/15 text-red-100' : 'border-amber-300/50 bg-amber-400/10 text-amber-100'}`}>
      {offline ? <WifiOff size={16} /> : <Wifi size={16} />}
      {offline
        ? `Network lost: no reports from the drone for ${link.silentSeconds ?? 0} s. It keeps searching on its own and sends what it found when it reconnects. Commands cannot reach it.`
        : 'Network weak: telemetry and casualty reports still arrive, video is paused.'}
    </div>
  )
}

/**
 * A simulator switch (network, GPS): press to take it away anywhere, press again to give it back.
 * ponytail: remembers its state in this page only; after a reload press it twice. Ask the bridge if that bites.
 */
export function SimToggle({ what, title, dot, act }: {
  what: string; title: string; dot: string; act: (up: boolean) => Promise<{ success: boolean; message: string }>
}) {
  const [busy, setBusy] = useState(false)
  const [cut, setCut] = useState(false)
  const [note, setNote] = useState<string | null>(null)

  const toggle = async () => {
    setBusy(true)
    try {
      const result = await act(cut)
      if (result.success) setCut(!cut)
      setNote(result.success ? null : result.message)
    } catch {
      setNote('the dashboard bridge is not reachable')
    } finally {
      setBusy(false)
    }
  }

  return (
    <div className="flex flex-col items-end">
      <button type="button" onClick={() => void toggle()} disabled={busy} title={title}
              className="flex items-center gap-2 rounded border border-white/25 bg-white/10 px-3 py-2 text-[12px] uppercase tracking-[0.09em] text-white transition hover:bg-white/20 disabled:opacity-50">
        <span className="h-2 w-2 rounded-full" style={{ backgroundColor: dot }} />
        {cut ? `RESTORE ${what}` : `${what === 'GPS' ? 'JAM' : 'CUT'} ${what}`}
      </button>
      {note && <span className="mt-1 max-w-[260px] text-right text-[11px] uppercase text-amber-300">{note}</span>}
    </div>
  )
}

/** Cut or restore the drone's network from the dashboard, anywhere in the world. */
export function NetworkSwitch({ link }: { link: LinkStatus | null }) {
  return <SimToggle what="NETWORK" title="Simulate losing the drone's network, wherever it is"
                    dot={LINK_COLOUR[link?.state ?? 'UNKNOWN']} act={simulationControl.setNetwork} />
}
