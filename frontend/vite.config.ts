import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  // GitHub Pages serves the replay site under /<repo>/; locally it is served from the root
  base: process.env.PAGES_BASE || '/',
})
