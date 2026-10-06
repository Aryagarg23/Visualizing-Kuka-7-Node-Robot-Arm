import { defineConfig } from 'vite';

// Relative base so the build runs from any folder (the site serves it under /embeds/kuka-arm/).
export default defineConfig({ base: './', build: { target: 'es2022' } });
