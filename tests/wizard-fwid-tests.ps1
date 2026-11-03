# 向导版本判定回归：只测纯逻辑，不打开串口、不接触任何设备。
# 运行：powershell -NoProfile -ExecutionPolicy Bypass -File tests/wizard-fwid-tests.ps1 [-KitDir <套件目录>]
param(
    # 套件目录（含 固件\固件清单.json）；默认为构建好的 590 保姆包。
    [string]$KitDir = ''
)
$ErrorActionPreference='Stop'
$repoRoot=Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
if([string]::IsNullOrEmpty($KitDir)){ $KitDir = Join-Path $repoRoot 'release-kit\ESP32-590RPM-保姆烧录-左右20度' }
$KitDir=(Resolve-Path -LiteralPath $KitDir).Path

# wizard.ps1 用 -KitRoot 定位清单；这里显式指向套件目录。
. (Join-Path $repoRoot 'handoff\wizard.ps1') -LoadOnly -KitRoot $KitDir

$fail=0; $pass=0
function Check([string]$Name,[scriptblock]$Body){
  try { & $Body; $script:pass++; Write-Host "  [通过] $Name" }
  catch { $script:fail++; Write-Host "  [失败] $Name -> $($_.Exception.Message)" -ForegroundColor Red }
}
function Expect-Throw([scriptblock]$Body){
  try { & $Body } catch { return }
  throw '预期应当抛出异常，但没有抛出'
}
function Expect-NoThrow([scriptblock]$Body){
  try { & $Body } catch { throw ('预期不应抛出异常，但抛出了：' + $_.Exception.Message) }
}

Write-Host '1. 从本包清单读取期望 FWID'
$boatWant = Get-ExpectedFwid 'boat'
$remoteWant = Get-ExpectedFwid 'remote'
Check '小船期望 FWID 含 R590 与 OS600' { if($boatWant -notmatch 'R590' -or $boatWant -notmatch 'OS600'){ throw "got $boatWant" } }
Check '遥控器期望 FWID 含 20 度契约' { if($remoteWant -notmatch 'S20-A70-110-U1278-1500-1722'){ throw "got $remoteWant" } }
Check '小船与遥控器 FWID 不同' { if($boatWant -eq $remoteWant){ throw 'same fwid' } }
Write-Host "    小船：$boatWant"
Write-Host "    遥控器：$remoteWant"

Write-Host '2. 版本判定：只有 FWID 完全一致才算最新'
Check '一致 -> 通过' {
  Expect-NoThrow { Assert-Board @{role='boat';mac='24:62:AB:F0:72:94';fwid=$boatWant} 'boat' }
}
Check '旧 20 度版（无 FWID）-> 必须升级' {
  Expect-Throw { Assert-Board @{role='boat';mac='24:62:AB:F0:72:94';fwid=$null;steer='20';version='2'} 'boat' }
}
Check '旧 580 / 无 OS600 的 FWID -> 必须升级' {
  Expect-Throw { Assert-Board @{role='boat';mac='24:62:AB:F0:72:94';fwid='JAYFWB3-S20-A70-110-U1278-1500-1722-R580-C580pi2'} 'boat' }
}
Check '自制相似 FWID（R599）-> 必须升级' {
  Expect-Throw { Assert-Board @{role='boat';mac='24:62:AB:F0:72:94';fwid='JAYFWB4-S20-A70-110-U1278-1500-1722-R599-OS600-Cx'} 'boat' }
}
Check '角色不符 -> 拒绝' {
  Expect-Throw { Assert-Board @{role='remote';mac='24:62:AB:F0:72:94';fwid=$boatWant} 'boat' }
}
Check '遥控器固件装在小船上 -> 拒绝' {
  Expect-Throw { Assert-Board @{role='boat';mac='24:62:AB:F0:72:94';fwid=$remoteWant} 'boat' }
}
Check '登记地址不一致 -> 拒绝' {
  Expect-Throw { Assert-Board @{role='boat';mac='24:62:AB:F1:36:64';fwid=$boatWant} 'boat' '24:62:AB:F0:72:94' }
}
Check '遥控器一致 -> 通过' {
  Expect-NoThrow { Assert-Board @{role='remote';mac='24:62:AB:EF:CF:84';fwid=$remoteWant} 'remote' }
}

Write-Host '3. 身份行解析（纯正则，不依赖串口）'
function ParseLine([string]$line){
  if($line -match 'JAYv3 ROLE=(BOAT|REMOTE) MAC=([0-9A-Fa-f:]{17}) PEER=([0-9A-Fa-f:]{17})(?: PAIR=(\d+))?(?: STEER=(-?\d+))?(?: FWID=(\S+))?'){
    return @{role=$Matches[1].ToLower();mac=$Matches[2].ToUpper();peer=$Matches[3].ToUpper();
             version=$Matches[4];steer=$Matches[5];fwid=$Matches[6]}
  }
  return $null
}
Check '解析新版身份行并取出 FWID' {
  $v=ParseLine "JAYv3 ROLE=BOAT MAC=24:62:AB:F0:72:94 PEER=24:62:AB:EF:CF:84 PAIR=2 STEER=20 FWID=$boatWant VER=20261103-590rpm-r4"
  if($v.fwid -ne $boatWant){ throw "fwid=$($v.fwid)" }
  if($v.mac -ne '24:62:AB:F0:72:94'){ throw "mac=$($v.mac)" }
}
Check '旧格式身份行不会被误当成新版' {
  $v=ParseLine 'JAY ROLE=BOAT MAC=24:62:AB:F0:72:94 PEER=24:62:AB:EF:CF:84 VERSION=2 STEER=20'
  if($v -ne $null){ throw '旧格式被新版正则匹配了' }
}

Write-Host ''
if($fail -eq 0){ Write-Host "通过 $pass 项，失败 0 项"; exit 0 }
Write-Host "通过 $pass 项，失败 $fail 项"
exit 1
