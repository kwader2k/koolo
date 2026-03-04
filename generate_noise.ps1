# generate_noise.ps1 — Creates internal/buildnoise/noise_gen.go
# with cryptographically random constants each build.
$ErrorActionPreference = 'Stop'

$rng  = [System.Security.Cryptography.RandomNumberGenerator]::Create()
$buf  = New-Object byte[] 32
$rng.GetBytes($buf)

$e0 = '0x' + [BitConverter]::ToString($buf, 0, 8).Replace('-','')
$e1 = '0x' + [BitConverter]::ToString($buf, 8, 8).Replace('-','')
$e2 = '0x' + [BitConverter]::ToString($buf, 16, 8).Replace('-','')
$e3 = '0x' + [BitConverter]::ToString($buf, 24, 8).Replace('-','')

$content = @"
//go:build noisegen

package buildnoise

var (
	entropy0 uint64 = $e0
	entropy1 uint64 = $e1
	entropy2 uint64 = $e2
	entropy3 uint64 = $e3
)
"@

$outPath = Join-Path (Join-Path (Join-Path $PSScriptRoot 'internal') 'buildnoise') 'noise_gen.go'
Set-Content -Path $outPath -Value $content -Encoding UTF8 -NoNewline
Write-Host "Noise generated: $e0 $e1 $e2 $e3"
