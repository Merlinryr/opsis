# Replays every recording in recordings/ over several RANSAC seeds and prints the median [min..max] of each
# metric per recording: a change counts only if it moves a median past the other run's range.
#   .\scripts\replay-seeds.ps1 [-Seeds 5]
param([int]$Seeds = 5)

cargo test --release --test replay --no-run 2>$null | Out-Null
$exe = Get-ChildItem target\release\build -Recurse -Filter "replay-*.exe" | Sort-Object LastWriteTime | Select-Object -Last 1
$keys = "lost", "limited", "closure", "zspread", "gapmed", "gap95", "gapmax", "ms", "msmax"
foreach ($rec in Get-ChildItem recordings -Directory | Sort-Object Name) {
    $runs = 1..$Seeds | ForEach-Object {
        $env:OPSIS_REPLAY = $rec.FullName; $env:OPSIS_REPLAY_SEED = $_
        $line = & $exe.FullName --ignored --nocapture 2>$null | Select-String "^SUMMARY"
        $v = @{}; ($line -replace "^SUMMARY ", "") -split " " | ForEach-Object -Begin { $k = $null } -Process { if ($k) { $v[$k] = [double]$_; $k = $null } else { $k = $_ } }
        $v
    }
    $cols = foreach ($k in $keys) {
        $x = $runs | ForEach-Object { $_[$k] } | Sort-Object
        "{0} {1:g4} [{2:g4}..{3:g4}]" -f $k, $x[[int][math]::Floor($x.Count / 2)], $x[0], $x[-1]
    }
    "{0}  {1}" -f $rec.Name.Substring(11, 8), ($cols -join "  ")
}
Remove-Item Env:OPSIS_REPLAY, Env:OPSIS_REPLAY_SEED
