param(
    [int]$Port = 8765,
    [string]$PythonPath = '',
    [string]$Sam3Url = 'http://192.168.3.107:25541/api/v1/segment'
)
$ErrorActionPreference = 'Stop'
$repoDir = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$scriptPath = Join-Path $PSScriptRoot 'sam3_debug_server.py'
$runtimeDir = Join-Path $repoDir 'logs/sam3_debug/runtime'
New-Item -ItemType Directory -Force -Path $runtimeDir | Out-Null
$localUrl = "http://127.0.0.1:$Port"
function Read-DebugHealth {
    $request = [System.Net.WebRequest]::Create("$localUrl/health")
    $request.Proxy = $null
    $request.Timeout = 2000
    $response = $request.GetResponse()
    $reader = New-Object System.IO.StreamReader($response.GetResponseStream())
    try { return ($reader.ReadToEnd() | ConvertFrom-Json) }
    finally { $reader.Dispose(); $response.Dispose() }
}
try {
    $existing = Read-DebugHealth
} catch { $existing = $null }
if ($existing -and $existing.app -eq 'cdf-sam3-debug') {
    Write-Output "Already running: $localUrl (SAM3: $($existing.upstream))"
    exit 0
}
if (-not $PythonPath) {
    $candidates = @()
    if ($env:CONDA_PREFIX) { $candidates += (Join-Path $env:CONDA_PREFIX 'python.exe') }
    $candidates += (Join-Path $env:USERPROFILE 'miniconda3/envs/py11/python.exe')
    $pythonCommand = Get-Command python -ErrorAction SilentlyContinue
    if ($pythonCommand) { $candidates += $pythonCommand.Source }
    $PythonPath = $candidates | Where-Object { Test-Path -LiteralPath $_ -PathType Leaf } | Select-Object -First 1
}
if (-not $PythonPath -or -not (Test-Path -LiteralPath $PythonPath -PathType Leaf)) {
    throw 'Provide -PythonPath pointing to Python with requests, numpy and opencv-python installed.'
}
# Quote each Start-Process argument to preserve paths containing spaces or Chinese characters.
if ($scriptPath.Contains('"') -or $Sam3Url.Contains('"')) { throw 'Invalid quote in path or SAM3 URL.' }
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$stdoutPath = Join-Path $runtimeDir "$stamp.stdout.log"
$stderrPath = Join-Path $runtimeDir "$stamp.stderr.log"
$process = Start-Process -FilePath $PythonPath -ArgumentList @('-B', '-X', 'utf8', ('"' + $scriptPath + '"'), '--port', "$Port", '--sam3-url', ('"' + $Sam3Url + '"')) -WorkingDirectory $repoDir -WindowStyle Hidden -PassThru -RedirectStandardOutput $stdoutPath -RedirectStandardError $stderrPath
$ready = $false
for ($attempt = 0; $attempt -lt 30; $attempt++) {
    Start-Sleep -Milliseconds 200
    $process.Refresh()
    if ($process.HasExited) { throw "Server exited. See $stderrPath" }
    try {
        $status = Read-DebugHealth
        if ($status.app -eq 'cdf-sam3-debug') { $ready = $true; break }
    } catch {}
}
if (-not $ready) { throw "Server did not become ready. PID: $($process.Id). See $stderrPath" }
@{ pid = $process.Id; url = $localUrl; stdout = $stdoutPath; stderr = $stderrPath; python = $PythonPath } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $runtimeDir 'server.json') -Encoding utf8
Write-Output "SAM3 Debug: $localUrl"
Write-Output "PID: $($process.Id)"
Write-Output "SAM3: $Sam3Url"
Write-Output "Stop: Stop-Process -Id $($process.Id)"
