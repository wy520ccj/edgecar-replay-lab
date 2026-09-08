$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

cmake --preset release
cmake --build --preset release --parallel
ctest --preset debug --output-on-failure

$out = Join-Path $root 'out/demo'
if (Test-Path -LiteralPath $out) { Remove-Item -LiteralPath $out -Recurse -Force }
New-Item -ItemType Directory -Path $out | Out-Null
Get-ChildItem -LiteralPath (Join-Path $root 'scenarios') -Filter '*.yaml' | Sort-Object Name | ForEach-Object {
  $name = [IO.Path]::GetFileNameWithoutExtension($_.Name)
  & (Join-Path $root 'build/release/bin/edgecar-replay.exe') --scenario $_.FullName --output (Join-Path $out $name)
}
& (Join-Path $root 'build/release/bin/edgecar-report.exe') --telemetry (Join-Path $out '03_s_curve_nominal/telemetry.v1.jsonl') --output (Join-Path $out 'index.html')
Write-Host "Demo report: $out/index.html"
