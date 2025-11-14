/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        bg1: '#1E1E1E',
        bg2: '#333333',
        bg3: '#1f1f1f',
        primary: '#FF3B3B',
        active: '#007a1f',
        fall: '#cc0000',
      }
    },
  },
  plugins: [],
}