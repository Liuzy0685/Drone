// Package the already-built app as an offline, double-clickable HTML preview.
import { readFile, writeFile } from 'node:fs/promises';
import { resolve } from 'node:path';

const output = resolve('dist');
let html = await readFile(resolve(output, 'index.html'), 'utf8');
const script = html.match(/<script[^>]*src="([^"]+)"[^>]*><\/script>/);
const stylesheet = html.match(/<link[^>]*rel="stylesheet"[^>]*href="([^"]+)"[^>]*>/);
if (!script || !stylesheet) throw new Error('Built script or stylesheet was not found');
const javascript = await readFile(resolve(output, script[1].replace(/^\//, '')), 'utf8');
const css = await readFile(resolve(output, stylesheet[1].replace(/^\//, '')), 'utf8');
html = html.replace(script[0], () => `<script type="module">${javascript.replace(/<\/script/gi, '<\\/script')}</script>`);
html = html.replace(stylesheet[0], () => `<style>${css}</style>`);
await writeFile(resolve(output, 'standalone.html'), html);
console.log('Offline preview: dist/standalone.html');
