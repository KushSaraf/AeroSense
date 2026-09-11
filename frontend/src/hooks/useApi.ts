import { useEffect, useState } from 'react'
import { API_URL } from '../services/apiServices'

interface ApiState<T> {
  data: T | null
  error: string | null
  loading: boolean
}

/**
 * Polls one endpoint on the dashboard bridge. There is no cached fallback: if the bridge is not
 * reachable the page says so, because showing the last good numbers as if they were current is
 * how an operator ends up trusting a frozen screen.
 */
export const useApi = <T>(path: string, pollMs = 3000): ApiState<T> => {
  const [state, setState] = useState<ApiState<T>>({ data: null, error: null, loading: true })

  useEffect(() => {
    let cancelled = false
    const load = async () => {
      try {
        const response = await fetch(`${API_URL}${path}`)
        if (!response.ok) throw new Error(String(response.status))
        const data = (await response.json()) as T
        if (!cancelled) setState({ data, error: null, loading: false })
      } catch {
        if (!cancelled) {
          setState({
            data: null,
            error: 'Dashboard bridge unreachable — run: ros2 run aero_sense_bridge dashboard_bridge',
            loading: false,
          })
        }
      }
    }
    void load()
    const timer = setInterval(() => void load(), pollMs)
    return () => {
      cancelled = true
      clearInterval(timer)
    }
  }, [path, pollMs])

  return state
}
