# Launch the crankcam GUI, creating/reusing a local venv and installing
# the python/ package into it as needed. No manual venv steps required.
$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$VenvDir = Join-Path $ScriptDir ".venv"
$VenvPython = Join-Path $VenvDir "Scripts\python.exe"

if (-not (Test-Path $VenvDir)) {
    python -m venv $VenvDir
}

& $VenvPython -m pip install -q --upgrade pip
& $VenvPython -m pip install -q -e (Join-Path $ScriptDir "python")

& (Join-Path $VenvDir "Scripts\crankcam-gui.exe") @args
