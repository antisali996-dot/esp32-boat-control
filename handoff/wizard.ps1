param([switch]$LoadOnly, [string]$KitRoot='')
$ErrorActionPreference='Stop'
# 默认以脚本所在目录的上一级（套件根）为基准；测试可显式传入。
if([string]::IsNullOrEmpty($KitRoot)){
 $script:Root=Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
} else {
 $script:Root=(Resolve-Path -LiteralPath $KitRoot).Path
}
$script:Session=$null
# 本包固件身份：只有板上 FWID 与之一致才算“已是最新”。
# 旧 20 度版本虽然也是 STEER=20 / VERSION=2，但没有 FWID 或 FWID 不同，
# 必须升级，否则会继续按 580 RPM 运行、且没有 600 超速锁存。
function Get-ExpectedFwid([string]$Role){
 $manifest=Get-Content -LiteralPath (Join-Path $script:Root '固件\固件清单.json') -Raw | ConvertFrom-Json
 $key=if($Role -eq 'boat'){'BOAT'}else{'REMOTE'}
 return $manifest.roles.$key.fwid
}
function Assert-Board($Info,[string]$Role,[string]$ExpectedMac='') {
 if(!$Info -or $Info.role -ne $Role){throw '板子用途不符，请检查插入的是小船还是遥控器。'}
 if($ExpectedMac -and $Info.mac -ne $ExpectedMac){throw '芯片地址与本次登记不一致，已停止，没有给这块板写入配对。'}
 $want=Get-ExpectedFwid $Role
 if(!$Info.fwid -or $Info.fwid -ne $want){
  throw ('板上固件不是本包版本（需要 ' + $want + '，实际 ' + $(if($Info.fwid){$Info.fwid}else{'无 FWID，属旧版'}) + '）。')
 }
}
function Wait-Board([string]$Role){
 $name=if($Role -eq 'boat'){'小船主控'}else{'遥控器'}
 Write-Host "请只连接【$name】的USB；其他ESP32请先拔下。" -ForegroundColor Cyan
 [void](Read-Host '接好后按回车')
 $ports=@(Get-PnpDevice -PresentOnly -Class Ports | Where-Object {$_.InstanceId -like 'USB\*'} | ForEach-Object {
  if($_.FriendlyName -match '\((COM\d+)\)'){$Matches[1]}
 } | Select-Object -Unique)
 if($ports.Count -ne 1){throw '没有检测到唯一的USB串口。请只插当前这块板，检查数据线及驱动后重试。'}
 return $ports[0]
}
function Run-Tool([string[]]$ToolArgs,[string]$Name){
 $result=& (Join-Path $script:Root '工具\esptool.exe') @ToolArgs 2>&1
 $code=$LASTEXITCODE
 $result | Out-File -LiteralPath (Join-Path $script:Session ($Name+'.log')) -Encoding utf8
 if($code -ne 0){throw '工具操作失败。请检查连接，并保留“备份与记录”中的诊断文件。'}
 return ($result -join "`n")
}
function Open-Serial([string]$Port){
 $serial=New-Object System.IO.Ports.SerialPort $Port,115200,None,8,one
 $serial.DtrEnable=$false;$serial.RtsEnable=$false;$serial.ReadTimeout=250;$serial.NewLine="`n"
 $serial.Open();Start-Sleep -Milliseconds 500
 return $serial
}
function Read-SerialInfo($Serial){
 $Serial.DiscardInBuffer();$Serial.WriteLine('JAY INFO')
 $until=[DateTime]::UtcNow.AddSeconds(5)
 $legacy=$null
 while([DateTime]::UtcNow -lt $until){
  try{
   $line=$Serial.ReadLine()
   # 优先读新版 JAYv3 身份行（带 FWID）；它才是版本判定依据。
   if($line -match 'JAYv3 ROLE=(BOAT|REMOTE) MAC=([0-9A-Fa-f:]{17}) PEER=([0-9A-Fa-f:]{17})(?: PAIR=(\d+))?(?: STEER=(-?\d+))?(?: FWID=(\S+))?'){
    return @{role=$Matches[1].ToLower();mac=$Matches[2].ToUpper();peer=$Matches[3].ToUpper();
             version=$Matches[4];steer=$Matches[5];fwid=$Matches[6]}
   }
   # 旧格式没有 FWID：先记下来，继续等新版身份行，等不到再按“旧版”返回。
   if($line -match 'JAY ROLE=(BOAT|REMOTE) MAC=([0-9A-Fa-f:]{17}) PEER=([0-9A-Fa-f:]{17})(?: VERSION=(\d+))?(?: STEER=(\d+))?'){
    $legacy=@{role=$Matches[1].ToLower();mac=$Matches[2].ToUpper();peer=$Matches[3].ToUpper();
              version=$Matches[4];steer=$Matches[5];fwid=$null}
   }
  }catch [System.TimeoutException]{}
 }
 if($legacy){return $legacy}
 throw '没有收到板子身份回应。请检查串口或固件。'
}
function Read-Board([string]$Port,[switch]$Optional){
 $serial=$null
 try{$serial=Open-Serial $Port;return (Read-SerialInfo $serial)}
 catch{if($Optional){return $null};throw}
 finally{if($serial){if($serial.IsOpen){$serial.Close()};$serial.Dispose()}}
}
function Prepare-Board([string]$Role){
 $port=Wait-Board $Role
 $info=Read-Board $port -Optional
 if($info -and $info.role -ne $Role){throw '当前插入的板子用途不符。已停止，请换成提示要求的板子。'}
 $want=Get-ExpectedFwid $Role
 $needsUpgrade = (-not $info) -or (-not $info.fwid) -or ($info.fwid -ne $want)
 if($needsUpgrade){
  $seen=if($info.fwid){$info.fwid}else{'无 FWID（旧版）'}
  Write-Host ('这块板需要升级程序（板上：' + $seen + '）。正在识别芯片。')
  $text=Run-Tool -ToolArgs @('--chip','esp32','--port',$port,'--baud','115200','flash_id') -Name ($Role+'-芯片')
  if($text -notmatch '(?m)^MAC:\s*([0-9a-fA-F:]{17})'){throw '无法读取芯片地址，已停止。'}
  $mac=$Matches[1].ToUpper()
  if($info -and $mac -ne $info.mac){throw '芯片识别前后地址不一致，已停止。'}
  if($text -notmatch 'Detected flash size:\s*(4|8|16)MB'){throw '未识别到支持的Flash容量，已停止。'}
  $size=[int]$Matches[1]*1024*1024
  Write-Host "已读到芯片地址：$mac。正在备份完整原固件，可能需要3至5分钟，请勿拔线。"
  $backup=Join-Path $script:Session ($Role+'-原固件.bin')
  Run-Tool -ToolArgs @('--chip','esp32','--port',$port,'--baud','460800','read_flash','0x0',("0x{0:X}" -f $size),$backup) -Name ($Role+'-备份') | Out-Null
  if((Get-Item -LiteralPath $backup).Length -ne $size){throw '原固件备份大小错误，已禁止烧录。'}
  Get-FileHash -LiteralPath $backup -Algorithm SHA256 | Format-List | Out-File -LiteralPath (Join-Path $script:Session ($Role+'-备份校验.txt'))
  Write-Host '原固件备份完成，正在写入程序并校验。'
  $image=Join-Path $script:Root ("固件\$Role.merged.bin")
  $written=Run-Tool -ToolArgs @('--chip','esp32','--port',$port,'--baud','460800','write_flash','-z','--flash_mode','dio','--flash_freq','40m','--flash_size','keep','0x0',$image) -Name ($Role+'-烧录')
  if($written -notmatch 'Hash of data verified'){throw '没有收到写入校验成功结果，已停止。'}
  Start-Sleep -Seconds 2
  $info=Read-Board $port
  Assert-Board $info $Role $mac
 }
 Assert-Board $info $Role
 $info.port=$port
 $info | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $script:Session ($Role+'-身份.json')) -Encoding utf8
 Write-Host ('本次登记地址：'+$info.mac) -ForegroundColor Green
 return $info
}
function Set-Peer($Expected,[string]$Peer){
 $serial=$null
 try{
  $serial=Open-Serial $Expected.port
  $before=Read-SerialInfo $serial
  Assert-Board $before $Expected.role $Expected.mac
  if($Peer -eq $Expected.mac -or $Peer -notmatch '^[0-9A-F]{2}(:[0-9A-F]{2}){5}$'){throw '配对地址无效，已停止。'}
  $serial.DiscardInBuffer();$serial.WriteLine("JAY BIND $Peer")
  $ack=$false;$until=[DateTime]::UtcNow.AddSeconds(5)
  while([DateTime]::UtcNow -lt $until){
   try{$line=$serial.ReadLine();if($line -match 'JAY SAVED REBOOT'){$ack=$true;break};if($line -match 'JAY ERROR='){throw '板子拒绝保存配对，请保留记录后检查。'}}catch [System.TimeoutException]{}
  }
  if(!$ack){throw '没有收到配对保存回应，已停止。'}
  Start-Sleep -Seconds 2
  $after=Read-SerialInfo $serial
  Assert-Board $after $Expected.role $Expected.mac
  if($after.peer -ne $Peer){throw '重启后配对地址不一致，未通过校验。'}
  $after | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $script:Session ($Expected.role+'-配对读回.json')) -Encoding utf8
 }finally{if($serial){if($serial.IsOpen){$serial.Close()};$serial.Dispose()}}
}
function Invoke-Wizard {
 Write-Host '第一步：登记小船。'
 $boat=Prepare-Board 'boat'
 Write-Host '第二步：登记遥控器，并让遥控器记住本次小船。'
 $remote=Prepare-Board 'remote'
 if($boat.mac -eq $remote.mac){throw '两次登记的是同一块芯片，已停止。'}
 Set-Peer $remote $boat.mac
 Write-Host '第三步：最后接回刚才那块小船，完成另一边配对。'
 $port=Wait-Board 'boat'
 $sameBoat=Read-Board $port
 Assert-Board $sameBoat 'boat' $boat.mac
 $sameBoat.port=$port
 Set-Peer $sameBoat $remote.mac
 Write-Host '两端配置已保存，并通过重启读回校验。' -ForegroundColor Green
 Write-Host ('小船地址：'+$boat.mac)
 Write-Host ('遥控器地址：'+$remote.mac)
 Write-Host '请断开USB，按正常接线给两端上电，检查转向及收包。配置校验不代表实际动作已经验证。'
}
if($LoadOnly){return}
try{
 Write-Host '20度一键换板：不用编译，不用填写地址，不搜索附近无线设备。' -ForegroundColor Cyan
 Write-Host '全程断开动力电池和舵机电源，只用USB供电。向导可能给两块板升级程序。' -ForegroundColor Yellow
 if((Read-Host '已确认上述条件，输入“确认”并回车开始') -cne '确认'){throw '操作已取消，没有访问串口或修改板子。'}
 $hashes=Get-Content -LiteralPath (Join-Path $script:Root '文件校验.json') -Raw | ConvertFrom-Json
 foreach($item in $hashes.PSObject.Properties){if((Get-FileHash -LiteralPath (Join-Path $script:Root $item.Name) -Algorithm SHA256).Hash -ne $item.Value){throw '工具包文件校验失败，请重新解压原始包。'}}
 $stamp=(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,6)
 $script:Session=Join-Path $script:Root ('备份与记录\'+$stamp)
 New-Item -ItemType Directory -Path $script:Session -ErrorAction Stop | Out-Null
 Invoke-Wizard
}catch{
 Write-Host ('操作未完成：'+$_.Exception.Message) -ForegroundColor Red
 if($script:Session){$_ | Out-String | Out-File -LiteralPath (Join-Path $script:Session '错误详情.txt') -Encoding utf8}
 exit 1
}
