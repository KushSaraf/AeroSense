import { useEffect, useState } from 'react'
import { isLiveAvailable } from '../services/apiServices'
import type { DataSource } from './useMission'

const POLL_MS = 10000

/**
 * Whether a simulation is serving data right now. Polled, so starting or stopping a simulation
 * is reflected without a page reload — an operator should never have to wonder whether the
 * numbers in front of them are live.
 */
export const useDataSource = (): DataSource => {
  const [source, setSource] = useState<DataSource>('offline')

  useEffect(() => {
    let cancelled = false
    const check = async () => {
      const live = await isLiveAvailable()
      if (!cancelled) setSource(live ? 'live' : 'offline')
    }
    void check()
    const timer = setInterval(() => void check(), POLL_MS)
    return () => {
      cancelled = true
      clearInterval(timer)
    }
  }, [])

  return source
}
