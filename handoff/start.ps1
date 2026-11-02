param([ValidateSet('boat','remote','pair')][string]$Action)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location -LiteralPath $root
function Get-Port {
 $ports=@([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object)
 Write-Host ('检测到的串口：'+($ports -join '，'))
 $port=(Read-Host '输入这块板子的串口，例如 COM8').Trim().ToUpper()
 if($port -notmatch '^COM[0-9]+$' -or $ports -notcontains $port){throw '找不到该串口，请检查串口名称、USB数据线和CH340驱动。'}
 return $port
}
function Esp([string[]]$ToolArgs,[string]$LogName){
 $lines=& "$root\tools\esptool.exe" @ToolArgs 2>&1
 $code=$LASTEXITCODE
 $lines | Out-File -LiteralPath $LogName -Encoding utf8
 Write-Host '本步工具操作已结束，诊断日志已保存。'
 if($code -ne 0){throw "工具执行失败，错误码：$code。诊断日志：$LogName"}
 return $lines
}
function Query-Info($serial){
 $serial.DiscardInBuffer();$serial.WriteLine('JAY INFO')
 $until=[DateTime]::UtcNow.AddSeconds(5)
 while([DateTime]::UtcNow -lt $until){
  try{$line=$serial.ReadLine();if($line -match 'JAY ROLE=(BOAT|REMOTE) MAC=([0-9A-Fa-f:]{17}) PEER=([0-9A-Fa-f:]{17})'){
   return @{role=$Matches[1].ToLower();mac=$Matches[2].ToUpper();peer=$Matches[3].ToUpper()}
  }}catch [System.TimeoutException]{}
 }
 throw '无法读取配对信息。请先用本包新版固件分别烧录两块板子，并确认选对串口。'
}
try {
 Write-Host '烧录及配对工具：操作前请断开动力电池和舵机电源，仅保留USB供电。' -ForegroundColor Yellow
 if((Read-Host '确认只有USB供电、动力已断开？请输入“确认”并回车') -cne '确认'){throw '操作已取消，没有访问串口或修改板子。'}
 $manifest=Get-Content -LiteralPath "$root\checksums.json" -Raw | ConvertFrom-Json
 foreach($item in $manifest.PSObject.Properties){
  $actual=(Get-FileHash -LiteralPath (Join-Path $root $item.Name) -Algorithm SHA256).Hash
  if($actual -ne $item.Value){throw "文件完整性检查失败，请重新解压原始工具包：$($item.Name)"}
 }
 $port=Get-Port
 $stamp=Get-Date -Format 'yyyyMMdd-HHmmss'
 $session=Join-Path $root "备份与日志\$stamp-$Action";New-Item -ItemType Directory -Path $session -Force | Out-Null
 if($Action -eq 'pair'){
  $serial=New-Object System.IO.Ports.SerialPort $port,115200,None,8,one
  $serial.DtrEnable=$false;$serial.RtsEnable=$false;$serial.ReadTimeout=300;$serial.NewLine="`n"
  try {
   $serial.Open();Start-Sleep -Seconds 2
   $info=Query-Info $serial
   $statePath=Join-Path $root 'pair-data.json'
   $state=@{boat='24:62:AB:F0:72:94';remote='24:62:AB:EF:CF:84'}
   if(Test-Path -LiteralPath $statePath){
    $saved=Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json
    foreach($r in @('boat','remote')){if($saved.$r -notmatch '^[0-9A-Fa-f]{2}(:[0-9A-Fa-f]{2}){5}$'){throw '配对记录文件无效，请停止操作并检查配对记录。'};$state[$r]=$saved.$r.ToUpper()}
   }
   $other=if($info.role -eq 'boat'){'remote'}else{'boat'}
   $peer=$state[$other]
   if($peer -eq $info.mac){throw '两种角色的地址相同，请检查是否插错板子或使用了错误的配对记录。'}
   $roleName=if($info.role -eq 'boat'){'小船主控'}else{'遥控器'}
   $otherName=if($other -eq 'boat'){'小船主控'}else{'遥控器'}
   Write-Host "当前板子：$roleName，自身地址：$($info.mac)；将配对到：$otherName，地址：$peer"
   if((Read-Host '请核对以上两块板子的身份及地址，正确请输入“确认”') -cne '确认'){throw '配对已取消。'}
   $serial.DiscardInBuffer();$serial.WriteLine("JAY BIND $peer")
   $ack=$false;$until=[DateTime]::UtcNow.AddSeconds(5)
   while([DateTime]::UtcNow -lt $until){
    try{$line=$serial.ReadLine();if($line -match 'JAY SAVED REBOOT'){$ack=$true;break};if($line -match 'JAY ERROR='){throw ('板子拒绝保存配对配置，请查看诊断信息。')}}catch [System.TimeoutException]{}
   }
   if(!$ack){throw '未收到保存成功回应，不能确认配对成功。'}
   Start-Sleep -Seconds 2
   $verified=Query-Info $serial
   if($verified.mac -ne $info.mac -or $verified.peer -ne $peer){throw '重启后读回的配对信息不一致，配对未通过校验。'}
   $state[$info.role]=$info.mac
   $state | ConvertTo-Json | Set-Content -LiteralPath $statePath -Encoding utf8
   $verified | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $session 'pair-readback.json') -Encoding utf8
   Write-Host '当前板子的配对配置已保存，并通过重启读回校验。请接上另一块板，再运行“03_换板配对.cmd”。此提示不代表双向无线通信已经验证。' -ForegroundColor Green
  } finally {if($serial.IsOpen){$serial.Close()};$serial.Dispose()}
 } else {
  $result=Esp -ToolArgs @('--chip','esp32','--port',$port,'--baud','115200','flash_id') -LogName (Join-Path $session 'chip.log')
  $text=$result -join "`n"
  if($text -notmatch '(?m)^MAC:\s*([0-9a-fA-F:]{17})'){throw '无法读取芯片地址，请检查连接。'}
  $mac=$Matches[1].ToUpper()
  if($text -notmatch 'Detected flash size:\s*(4|8|16)MB'){throw '无法确认存储容量，已停止，避免错误备份或写入。'}
  $size=[int]$Matches[1]*1024*1024
  $roleName=if($Action -eq 'boat'){'小船主控'}else{'遥控器'}
  Write-Host "准备烧录：$roleName；芯片地址：$mac；串口：$port"
  if((Read-Host "请确认当前连接的确实是$roleName，正确请输入“确认”") -cne '确认'){throw '操作已取消，尚未备份或写入固件。'}
  $backup=Join-Path $session 'flash-before.bin'
  Write-Host '正在读取整片原固件备份，可能需要3至5分钟，请勿拔线。'
  Esp -ToolArgs @('--chip','esp32','--port',$port,'--baud','460800','read_flash','0x0',("0x{0:X}" -f $size),$backup) -LogName (Join-Path $session 'backup.log') | Out-Null
  if((Get-Item -LiteralPath $backup).Length -ne $size){throw '原固件备份大小不正确，已禁止写入新固件。'}
  Get-FileHash -LiteralPath $backup -Algorithm SHA256 | Format-List | Out-File -LiteralPath (Join-Path $session 'backup-sha256.txt')
  $image=Join-Path $root "firmware\$Action.merged.bin"
  Write-Host '原固件备份已完成，正在写入新固件并校验，请勿断电。'
  $flashed=Esp -ToolArgs @('--chip','esp32','--port',$port,'--baud','460800','write_flash','-z','--flash_mode','dio','--flash_freq','40m','--flash_size','keep','0x0',$image) -LogName (Join-Path $session 'flash.log')
  if(($flashed -join "`n") -notmatch 'Hash of data verified'){throw '未找到写入校验成功记录，请保留日志并检查。'}
  Write-Host '固件烧录及写入校验成功。请给两块板分别运行“03_换板配对.cmd”，并保留“备份与日志”目录。' -ForegroundColor Green
 }
} catch {Write-Host ("操作未完成："+$_.Exception.Message) -ForegroundColor Red;exit 1}
