# One-time developer setup. Run from the repo root in PowerShell:
#   .\scripts\setup-dev.ps1
# Safe to re-run.

$ErrorActionPreference = 'Continue'
Write-Host "`n=== Teleop research platform: dev setup ===`n"

function Check($name, $ok, $fix) {
  if ($ok) { Write-Host "  [ OK ] $name" -ForegroundColor Green }
  else     { Write-Host "  [FAIL] $name" -ForegroundColor Red; Write-Host "         -> $fix" -ForegroundColor Yellow }
}

Write-Host "Toolchain:"
Check ".NET SDK"  (Get-Command dotnet -EA SilentlyContinue) "winget install Microsoft.DotNet.SDK.8"
Check "git"       (Get-Command git    -EA SilentlyContinue) "winget install Git.Git"
Check "git-lfs"   ((git lfs version 2>$null) -ne $null)     "git lfs install"

# Claude Code uses Git Bash for its Bash tool, and the invariant hook is a bash script.
$bash = "C:\Program Files\Git\bin\bash.exe"
Check "Git Bash"  (Test-Path $bash) "install Git for Windows, or set CLAUDE_CODE_GIT_BASH_PATH in .claude/settings.json"

Write-Host "`nGit config:"
git config --global core.longpaths true      # Windows MAX_PATH vs deep Unity Library paths
git config --global core.autocrlf false      # .gitattributes is the single authority
Write-Host "  [ OK ] core.longpaths=true, core.autocrlf=false"

if (-not (Test-Path .git)) { git init | Out-Null; Write-Host "  [ OK ] git init" }
git lfs install | Out-Null

# Unity smart merge driver for scene/prefab YAML.
$editors = @(Get-ChildItem "C:\Program Files\Unity\Hub\Editor" -Directory -EA SilentlyContinue |
             Sort-Object Name -Descending)
if ($editors.Count -gt 0) {
  $merge = Join-Path $editors[0].FullName "Editor\Data\Tools\UnityYAMLMerge.exe"
  if (Test-Path $merge) {
    # Forward slashes: git's config parser treats backslashes as escapes.
    $p = $merge -replace '\\','/'
    git config merge.unityyamlmerge.driver "`"$p`" merge -p `"`$BASE`" `"`$REMOTE`" `"`$LOCAL`" `"`$MERGED`""
    git config merge.unityyamlmerge.recursive binary
    Write-Host "  [ OK ] UnityYAMLMerge driver -> $($editors[0].Name)"
  } else { Write-Host "  [WARN] UnityYAMLMerge.exe not found under $($editors[0].FullName)" -ForegroundColor Yellow }
} else { Write-Host "  [WARN] No Unity editor found under C:\Program Files\Unity\Hub\Editor" -ForegroundColor Yellow }

# The hook is bash; CRLF endings silently break it.
$hook = "scripts\hooks\core-guard.sh"
if (Test-Path $hook) {
  if ((Get-Content $hook -Raw) -match "`r`n") {
    (Get-Content $hook -Raw) -replace "`r`n","`n" | Set-Content $hook -NoNewline -Encoding utf8
    Write-Host "  [ OK ] converted $hook to LF"
  } else { Write-Host "  [ OK ] $hook has LF endings" }
}

Write-Host "`nReminders:"
Write-Host "  - Unity Hub -> Add -> unity\TeleopVR   (the subfolder, NOT the repo root)"
Write-Host "  - Editor Settings -> Asset Serialization -> Force Text"
Write-Host "  - Player Settings -> ARM64 + IL2CPP + Vulkan, Internet Access = Require"
Write-Host "  - core\Teleop.sln for Core work; ignore Unity's generated .csproj files`n"
