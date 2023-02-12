import path from "path";

import sitemap from "@astrojs/sitemap";
import solid from "@astrojs/solid-js";

import { defineConfig } from 'astro/config';

// https://astro.build/config
export default defineConfig({
	integrations: [solid(), sitemap()],
	vite: {
		build: { sourcemap: true },
		resolve: {
			alias: {
				"~": path.resolve("./src")
			}
		},
	}
});