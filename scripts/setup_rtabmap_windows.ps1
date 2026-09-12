param(
  [string]$PixiPath = "",
  [string]$RosWorkspace = "",
  [switch]$Build
)
$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "resolve_runtime_paths.ps1")
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$pixi = Resolve-PixiExecutable $PixiPath
$workspace = Resolve-RosWorkspace $RosWorkspace
$src = Join-Path $workspace "src"
$rtabmap = Join-Path $src "rtabmap"
$rtabmapRos = Join-Path $src "rtabmap_ros"
$coreCommit = "2fbbe19d707e6b9fada74bc6b74c284117197a1c"
$rosCommit = "61edb4ee85e35f4cc967fa6b502b58d8e3bc6e4f"

if (-not (Test-Path (Join-Path $rtabmap ".git"))) {
  git clone https://github.com/introlab/rtabmap.git $rtabmap
}
if (-not (Test-Path (Join-Path $rtabmapRos ".git"))) {
  git clone https://github.com/introlab/rtabmap_ros.git $rtabmapRos
}
git -C $rtabmap fetch --all --tags
git -C $rtabmap checkout $coreCommit
git -C $rtabmapRos fetch --all --tags
git -C $rtabmapRos checkout $rosCommit

$patch = Join-Path $repo "patches\rtabmap_ros_windows.patch"
$alreadyPatched = (Select-String -Path (Join-Path $rtabmapRos "rtabmap_odom\CMakeLists.txt") -Pattern "RTABMAP_ODOM_BUILDING_DLL" -Quiet) -and
  (Select-String -Path (Join-Path $rtabmapRos "rtabmap_slam\CMakeLists.txt") -Pattern "RTABMAP_SLAM_BUILDING_DLL" -Quiet) -and
  (Select-String -Path (Join-Path $rtabmapRos "rtabmap_util\CMakeLists.txt") -Pattern "RTABMAP_UTIL_BUILDING_DLL" -Quiet)
if ($alreadyPatched) {
  Write-Host "Windows RTAB-Map patch already applied"
} else {
  git -C $rtabmapRos apply --whitespace=nowarn $patch
}

$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) { throw "Visual Studio 2022 Build Tools were not found: $vswhere" }
$vsInstall = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsInstall) { throw "MSVC v143 Desktop C++ workload is not installed" }
$sdk = Get-ChildItem "${env:ProgramFiles(x86)}\Windows Kits\10\Include" -Directory | Select-Object -Last 1
if (-not $sdk) { throw "Windows 11 SDK was not found" }

if ($Build) {
  $manifest = Join-Path $workspace "pixi.toml"
  $buildArgs = @("run", "--manifest-path", $manifest, "colcon", "build", "--merge-install", "--cmake-clean-cache", "--cmake-args", "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF", "-DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON", "--packages-up-to", "rtabmap_odom", "rtabmap_slam")
  $vsDevCmd = Join-Path $vsInstall "Common7\Tools\VsDevCmd.bat"
  $command = "call `"$vsDevCmd`" -arch=x64 && `"$pixi`" " + (($buildArgs | ForEach-Object { '"' + $_.Replace('"', '""') + '"' }) -join ' ')
  cmd.exe /d /s /c $command
  if ($LASTEXITCODE -ne 0) { throw "RTAB-Map ROS 2 build failed with code $LASTEXITCODE" }
}

$odomExe = Join-Path $workspace "install\Lib\rtabmap_odom\icp_odometry.exe"
$slamExe = Join-Path $workspace "install\Lib\rtabmap_slam\rtabmap.exe"
if (-not (Test-Path $odomExe) -or -not (Test-Path $slamExe)) {
  throw "Expected RTAB-Map executables were not found; rerun with -Build"
}
Write-Host "rtabmap_odom: $odomExe"
Write-Host "rtabmap_slam: $slamExe"
