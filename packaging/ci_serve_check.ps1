# CI helper (release.yml on GitHub and GitLab): start the built exe, wait for its "ready" line, load the page in headless Edge,
# check that the Viser client rendered, then stop the exe. Exits 1 with a message on any failure.
#   powershell -File packaging\ci_serve_check.ps1 -Exe dist\RobotCellOptimizer\RobotCellOptimizer.exe [-Port 8765]
# The caller blocks the network (firewall rule) and sets RSO_OFFLINE_GUARD / RSO_DATA; this script only checks.
param(
    [Parameter(Mandatory = $true)][string]$Exe,
    [int]$Port = 8765,
    [int]$ReadyTimeoutSec = 120,
    [int]$VirtualTimeMs = 30000
)
$ErrorActionPreference = "Stop"

function Find-Edge {
    $c = @(@(
        "${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe",
        "$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe"
    ) | Where-Object { $_ -and (Test-Path $_) })
    if ($c) { return $c[0] }
    $cmd = Get-Command msedge -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    return $null
}

$tmp = Join-Path ([IO.Path]::GetTempPath()) ("rso-serve-" + [guid]::NewGuid().ToString("N").Substring(0, 8))
New-Item -ItemType Directory -Path $tmp | Out-Null
$out = Join-Path $tmp "out.txt"
$err = Join-Path $tmp "err.txt"
$proc = $null
try {
    $edge = Find-Edge
    if (-not $edge) { throw "Microsoft Edge (msedge.exe) not found on this machine" }
    $proc = Start-Process -FilePath $Exe -ArgumentList @("--no-browser", "--port", "$Port") `
        -RedirectStandardOutput $out -RedirectStandardError $err -PassThru -WindowStyle Hidden
    $deadline = (Get-Date).AddSeconds($ReadyTimeoutSec)
    $ready = $false
    while ((Get-Date) -lt $deadline) {
        if ($proc.HasExited) { throw "the exe exited early with code $($proc.ExitCode)" }
        if ((Test-Path $out) -and (Select-String -Path $out -Pattern "Robot Cell Optimizer ready:" -Quiet)) {
            $ready = $true; break
        }
        Start-Sleep -Milliseconds 500
    }
    if (-not $ready) { throw "no 'Robot Cell Optimizer ready:' line within $ReadyTimeoutSec s" }
    Write-Host "exe ready: $(Select-String -Path $out -Pattern 'ready:' | Select-Object -First 1 -ExpandProperty Line)"

    $edgeProfile = Join-Path $tmp "edge-profile"
    $domFile = Join-Path $tmp "dom.html"
    # --timeout: a wall-clock cap after which Edge dumps the page anyway (virtual time alone hung on the
    # GitHub runner, 2026-11-19). An Edge that wrote the page but does not exit is ended, not a failure.
    $edgeArgs = @("--headless=new", "--disable-gpu", "--no-first-run", "--no-default-browser-check",
        "--user-data-dir=`"$edgeProfile`"", "--use-angle=swiftshader", "--enable-unsafe-swiftshader",
        "--virtual-time-budget=$VirtualTimeMs", "--timeout=$(2 * $VirtualTimeMs)", "--dump-dom",
        "http://localhost:$Port/")
    $t0 = Get-Date
    $edgeProc = Start-Process -FilePath $edge -ArgumentList $edgeArgs -RedirectStandardOutput $domFile `
        -RedirectStandardError (Join-Path $tmp "edge-err.txt") -PassThru -WindowStyle Hidden
    $exited = $edgeProc.WaitForExit(180000)
    Write-Host "headless Edge: exited $exited after $([int]((Get-Date) - $t0).TotalSeconds) s"
    if (-not $exited) { & taskkill /PID $edgeProc.Id /T /F | Out-Null; Start-Sleep -Seconds 2 }
    $dom = if (Test-Path $domFile) { Get-Content $domFile -Raw } else { "" }
    if (-not $exited -and -not $dom) { throw "headless Edge timed out without writing the page" }
    if (-not $dom) { throw "headless Edge returned no DOM" }
    # the Viser client mounts React into #root; once connected it adds a canvas and the control panel
    $hasRoot = $dom -match '<div id="root"[^>]*>\s*<'
    $hasCanvas = $dom -match '<canvas'
    Write-Host "DOM: $($dom.Length) chars, root populated: $hasRoot, canvas: $hasCanvas"
    if (-not ($hasRoot -and $hasCanvas)) {
        $dom.Substring(0, [Math]::Min(2000, $dom.Length)) | Write-Host
        throw "the Viser page did not render (root populated: $hasRoot, canvas: $hasCanvas)"
    }
    if (Select-String -Path $out, $err -Pattern "offline-guard\] blocked" -Quiet) {
        throw "the app tried to reach the network (see [offline-guard] lines in its output)"
    }
    Write-Host "serve check: PASS"
}
catch {
    Write-Host "serve check: FAIL: $($_.Exception.Message)"
    foreach ($f in @($out, $err, (Join-Path $tmp "edge-err.txt"))) {
        if (Test-Path $f) { Write-Host "--- $f"; Get-Content $f -Tail 40 }
    }
    $script:failed = $true
}
finally {
    if ($proc -and -not $proc.HasExited) { & taskkill /PID $proc.Id /T /F | Out-Null }
    Start-Sleep -Milliseconds 500
    Remove-Item -Recurse -Force $tmp -ErrorAction SilentlyContinue
}
if ($script:failed) { exit 1 }
