/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{js,ts,jsx,tsx}'],
  theme: {
    extend: {
      colors: {
        bg: '#202635',
        panel: '#555D72',
        panelAlt: '#626A80',
        panelSoft: '#2A3345',
        border: '#8A91A4',
        text: '#F2F2EA',
        textMuted: '#C7CBD5',
        dark: '#20232C',
        success: '#7EB17A',
        warning: '#C7B16A',
        orange: '#D68E58',
        danger: '#C96B68',
      },
      fontFamily: {
        display: ['"Orbitron"', 'sans-serif'],
        sans: ['"Segoe UI"', 'sans-serif'],
      },
      boxShadow: {
        panel: '0 0 0 1px rgba(138,145,164,0.35)',
      },
    },
  },
  plugins: [],
}

