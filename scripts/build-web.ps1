# Release wasm build into pkg/, then print the gzipped size (M00 R9 budget: 1 MB).
$env:Path += ";$env:USERPROFILE\.cargo\bin"
Set-Location (Join-Path $PSScriptRoot "..")
function Step { & $args[0] $args[1..($args.Count - 1)]; if ($LASTEXITCODE) { throw "$($args[0]) failed ($LASTEXITCODE)" } }
Step cargo build --release --target wasm32-unknown-unknown
Step wasm-bindgen --target web --out-dir pkg target\wasm32-unknown-unknown\release\webar.wasm
# Not --all-features: it emits compact imports that browsers reject.
Step wasm-opt -Oz --strip-debug --strip-target-features -o pkg\webar_bg.wasm pkg\webar_bg.wasm
$b = [IO.File]::ReadAllBytes((Resolve-Path pkg\webar_bg.wasm))
$ms = New-Object IO.MemoryStream
$gz = New-Object IO.Compression.GZipStream($ms, [IO.Compression.CompressionLevel]::Optimal)
$gz.Write($b, 0, $b.Length); $gz.Close()
"webar_bg.wasm: {0:N0} B raw, {1:N0} B gzip" -f $b.Length, $ms.ToArray().Length
