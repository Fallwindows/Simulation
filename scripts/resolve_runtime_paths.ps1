function Resolve-PixiExecutable {
  param([string]$RequestedPath = "")

  if ($RequestedPath) {
    if (-not (Test-Path -LiteralPath $RequestedPath)) {
      throw "Pixi executable not found: $RequestedPath"
    }
    return (Resolve-Path -LiteralPath $RequestedPath).Path
  }

  $pixiCommand = Get-Command pixi -ErrorAction SilentlyContinue
  if ($pixiCommand) { return $pixiCommand.Source }

  $localPixi = Join-Path ([Environment]::GetFolderPath("LocalApplicationData")) "pixi\bin\pixi.exe"
  if (Test-Path -LiteralPath $localPixi) { return (Resolve-Path -LiteralPath $localPixi).Path }

  throw "Pixi was not found on PATH or in the current user's LocalApplicationData directory. Pass -PixiPath explicitly."
}

function Resolve-RosWorkspace {
  param([string]$RequestedPath = "")

  $workspace = if ($RequestedPath) {
    $RequestedPath
  } elseif ($env:ISAACSIM_ROS_WORKSPACE) {
    $env:ISAACSIM_ROS_WORKSPACE
  } else {
    "C:\IsaacSim-ros_workspaces\jazzy_ws"
  }

  $manifest = Join-Path $workspace "pixi.toml"
  if (-not (Test-Path -LiteralPath $manifest)) {
    throw "ROS 2 Pixi workspace manifest not found: $manifest. Set ISAACSIM_ROS_WORKSPACE or pass -RosWorkspace."
  }
  return (Resolve-Path -LiteralPath $workspace).Path
}
