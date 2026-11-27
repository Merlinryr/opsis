# Builds the wasm and the SDK into packages/opsis. -Parity also builds the GPU-vs-CPU check (examples/index.html).
param([switch]$Parity)
$env:Path += ";$env:USERPROFILE\.cargo\bin"
Set-Location (Join-Path $PSScriptRoot "..")
function Step { & $args[0] $args[1..($args.Count - 1)]; if ($LASTEXITCODE) { throw "$($args[0]) failed ($LASTEXITCODE)" } }
$features = if ($Parity) { @("--features", "parity") } else { @() }
Step cargo build --release --target wasm32-unknown-unknown -p opsis @features
Step wasm-bindgen --target web --out-dir packages/opsis/wasm target\wasm32-unknown-unknown\release\opsis.wasm
# Not --all-features: it emits imports browsers reject.
Step wasm-opt -Oz --strip-debug --strip-target-features -o packages/opsis/wasm/opsis_bg.wasm packages/opsis/wasm/opsis_bg.wasm
$b = [IO.File]::ReadAllBytes((Resolve-Path packages/opsis/wasm/opsis_bg.wasm))
$ms = New-Object IO.MemoryStream
$gz = New-Object IO.Compression.GZipStream($ms, [IO.Compression.CompressionLevel]::Optimal)
$gz.Write($b, 0, $b.Length); $gz.Close()
"opsis_bg.wasm: {0:N0} B raw, {1:N0} B gzip" -f $b.Length, $ms.ToArray().Length
Push-Location packages/opsis; try { pnpm run build; if ($LASTEXITCODE) { throw "tsc failed" } } finally { Pop-Location }
