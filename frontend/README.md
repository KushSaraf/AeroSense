# Aero Sense dashboard

React + TypeScript + Vite command-centre UI. Live, it talks to `aero_sense_bridge` on :8000;
with `VITE_REPLAY=1` it plays back the recorded flight in `public/replay/` instead, needing no
simulation.

```bash
npm install
npm run dev        # http://localhost:5173 (tools/dashboard.sh starts this for you)
npm run build      # type-check + production build into dist/
```

| Path | Role |
|---|---|
| `src/pages/` | one file per screen: Home, Dashboard, Map, Missions, Alerts, Reports, Telemetry, AI Perception, Settings |
| `src/layouts/AppShell.tsx` | sidebar + header around every page |
| `src/components/` | shared UI pieces (metric cards, status pills, replay bar, printable report) |
| `src/hooks/` | data hooks: live bridge API or replay, chosen by `useDataSource` |
| `src/services/` | `apiServices.ts` (bridge HTTP calls), `replay.ts` (recorded-flight playback) |
| `src/types/` | shared TypeScript types |
| `public/replay/` | recorded flight: `mission.json` + camera frames (re-record with `tools/record_replay.py`) |
| `public/mission-tiles/` | satellite imagery for the map |
