import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import './index.css'
import App from './App.tsx'
import { API_URL } from './services/apiServices'
import { IS_REPLAY, installReplayFetch, loadRecording } from './services/replay'

// the public site has no simulation behind it: answer the dashboard from a recorded flight
if (IS_REPLAY) {
  installReplayFetch(API_URL)
  void loadRecording()
}

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>,
)
