$env:Path += ";$env:USERPROFILE\.cargo\bin"
Set-Location (Join-Path $PSScriptRoot "..")
function Step { & $args[0] $args[1..($args.Count - 1)]; if ($LASTEXITCODE) { throw "$($args[0]) failed ($LASTEXITCODE)" } }
Step cargo build --release --target wasm32-unknown-unknown
Step wasm-bindgen --target web --out-dir pkg target\wasm32-unknown-unknown\release\opsis.wasm
# Not --all-features: it emits compact imports that browsers reject.
Step wasm-opt -Oz --strip-debug --strip-target-features -o pkg\opsis_bg.wasm pkg\opsis_bg.wasm
$b = [IO.File]::ReadAllBytes((Resolve-Path pkg\opsis_bg.wasm))
$ms = New-Object IO.MemoryStream
$gz = New-Object IO.Compression.GZipStream($ms, [IO.Compression.CompressionLevel]::Optimal)
$gz.Write($b, 0, $b.Length); $gz.Close()
"opsis_bg.wasm: {0:N0} B raw, {1:N0} B gzip" -f $b.Length, $ms.ToArray().Length
