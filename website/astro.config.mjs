import path from "path";
import sitemap from "@astrojs/sitemap";
import solid from "@astrojs/solid-js";
import { defineConfig } from 'astro/config';

import vercel from "@astrojs/vercel/serverless";

// https://astro.build/config
export default defineConfig({
  site: "https://taolib.vercel.app/",
  integrations: [solid(), sitemap()],
  output: "hybrid",
  vite: {
    build: {
      sourcemap: true
    },
    resolve: {
      alias: {
        "~": path.resolve("./src")
      }
    }
  },
  adapter: vercel()
});