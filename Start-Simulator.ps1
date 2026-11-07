param([int]$Port = 4188)
$ErrorActionPreference = 'Stop'
$nodeCommand = Get-Command node -ErrorAction SilentlyContinue
if (!$nodeCommand) { throw 'Install Node.js 20 or later, then run this launcher again. The standalone Open Warehouse Lab.html can also be opened directly.' }
$nodeExecutable = $nodeCommand.Source
Set-Location -LiteralPath $PSScriptRoot
$env:PORT = [string]$Port
Write-Host "Open http://127.0.0.1:$Port/ . Keep this terminal running. Press Ctrl+C to stop."
& $nodeExecutable tools/server.mjs
