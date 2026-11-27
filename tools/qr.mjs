// Terminal panel for phone.ps1: a compact QR code plus the URLs.
// Run as `npx -p qrcode-terminal@0.12.0 node tools/qr.mjs <url> <local> <log>`.
import { createRequire } from 'node:module'
import { delimiter, join } from 'node:path'

const [url, local, log] = process.argv.slice(2)
// npx puts the package's .bin on PATH; find the package next to it.
const bin = process.env.PATH.split(delimiter).find(p => p.includes('_npx') && p.endsWith('.bin'))
const qr = createRequire(join(bin ?? process.cwd(), '..', '_.js'))('qrcode-terminal')

const c = (n, s) => `\x1b[${n}m${s}\x1b[0m`
const row = (k, v) => `  ${c(90, k.padEnd(8))}${v}`

qr.generate(url, { small: true }, (code) => {
  console.log([
    '',
    `  ${c('1;36', '◆ Opsis')} ${c(90, '· phone testing')}`,
    '',
    code.split('\n').map(l => '  ' + l).join('\n'),
    '',
    row('Public', c('1;32', url)),
    row('Local', c(36, local)),
    row('Log', c(90, log)),
    '',
    `  ${c(33, '▸')} Scan with the phone camera  ${c(90, '·')}  ${c(1, 'Ctrl+C')} to stop`,
    '',
  ].join('\n'))
})
