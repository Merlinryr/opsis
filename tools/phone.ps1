# Phone testing: serve examples/ + pkg/ locally and expose them on a temporary public HTTPS URL
# (Cloudflare quick tunnel; HTTPS is required for WebGPU on phones). Prints a QR code. Ctrl+C stops both.
# -Record: accept camera.html?record=1 uploads into recordings/ (for the replay test).
param([switch]$Record)
if ($Record) { $env:OPSIS_RECORD = "1" }
$port = if ($env:PORT) { $env:PORT } else { 8085 }
Set-Location (Join-Path $PSScriptRoot "..")

# Shells opened before the winget install don't have cloudflared on PATH yet.
$cf = (Get-Command cloudflared -ErrorAction SilentlyContinue).Source
if (-not $cf) { $cf = "${env:ProgramFiles(x86)}\cloudflared\cloudflared.exe" }
if (-not (Test-Path $cf)) { throw "cloudflared not found: winget install Cloudflare.cloudflared" }

$env:PORT = $port
$server = Start-Process node -ArgumentList "tools/serve.mjs" -NoNewWindow -PassThru
Start-Sleep -Milliseconds 500
if ($server.HasExited) { throw "server failed to start on port $port (in use? set `$env:PORT to another port)" }

$log = Join-Path $env:TEMP "opsis-tunnel-$port.log"
$tunnel = Start-Process $cf -ArgumentList "tunnel --no-autoupdate --url http://127.0.0.1:$port" -NoNewWindow -PassThru -RedirectStandardError $log
try {
    $url = $null
    for ($i = 0; $i -lt 60 -and -not $url -and -not $tunnel.HasExited; $i++) {
        Start-Sleep 1
        $m = Select-String -Path $log -Pattern "https://[a-z0-9-]+\.trycloudflare\.com" -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($m) { $url = $m.Matches[0].Value + "/examples/" }
    }
    if (-not $url) { throw "no tunnel URL after 60 s, see $log" }
    npx --yes -p qrcode-terminal@0.12.0 node tools/qr.mjs $url "http://localhost:$port/examples/" $log
    Wait-Process -Id $tunnel.Id
} finally {
    Stop-Process -Id $tunnel.Id, $server.Id -ErrorAction SilentlyContinue
}
