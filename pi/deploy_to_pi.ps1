# Copy the four Pi files to ~/precland on the board and run the post-deploy check.
#
#     .\pi\deploy_to_pi.ps1                       # default target watarrow@watarrow-pi
#     .\pi\deploy_to_pi.ps1 -Target user@host
#
# Needs an SSH key on the Pi (see CLAUDE.md); it never prompts for a password.

param(
    [string]$Target = "watarrow@watarrow-pi"
)

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$sshOptions = @("-o", "BatchMode=yes", "-o", "ConnectTimeout=5")

$files = @(
    "scripts\precland_core.py",
    "scripts\pi_landing_bridge.py",
    "pi\pi_landing_config.json",
    "pi\setup_pi.sh"
) | ForEach-Object { Join-Path $repo $_ }

foreach ($file in $files) {
    if (-not (Test-Path $file)) { throw "missing file: $file" }
}

Write-Host "Checking $Target ..."
ssh @sshOptions $Target "mkdir -p ~/precland && hostname"
if ($LASTEXITCODE -ne 0) {
    throw "cannot reach $Target. Is the Pi plugged in and booted, and is the SSH key installed? See CLAUDE.md."
}

Write-Host "Copying files ..."
scp @sshOptions $files "${Target}:~/precland/"
if ($LASTEXITCODE -ne 0) { throw "scp failed" }

# Files written on Windows may carry CRLF line endings, which break bash scripts.
ssh @sshOptions $Target "cd ~/precland && sed -i 's/\r$//' setup_pi.sh *.py *.json && chmod +x setup_pi.sh"
if ($LASTEXITCODE -ne 0) { throw "post-copy fix-up failed" }

Write-Host "Running the post-deploy check ..."
ssh @sshOptions $Target "cd ~/precland && ./setup_pi.sh check"
if ($LASTEXITCODE -ne 0) {
    Write-Host "Check failed. If the venv is missing, run ./setup_pi.sh on the Pi once (needs internet)."
    exit 1
}
Write-Host "Deploy complete."
