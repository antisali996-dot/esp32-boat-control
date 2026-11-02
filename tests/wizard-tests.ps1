. (Join-Path $PSScriptRoot '..\handoff\wizard.ps1') -LoadOnly
$script:Events=@()
$script:Mode='ok'
function Prepare-Board([string]$Role){
 $mac=if($Role -eq 'boat'){'24:62:AB:F1:36:64'}else{'24:62:AB:EF:CF:84'}
 if($script:Mode -eq 'same'){$mac='24:62:AB:F1:36:64'}
 return @{role=$Role;mac=$mac;version='2';steer='20';port='COM_TEST'}
}
function Wait-Board([string]$Role){return 'COM_TEST'}
function Read-Board([string]$Port){
 $mac=if($script:Mode -eq 'wrong-third'){'24:62:AB:00:00:01'}else{'24:62:AB:F1:36:64'}
 return @{role='boat';mac=$mac;version='2';steer='20';port=$Port}
}
function Set-Peer($Expected,[string]$Peer){$script:Events+=($Expected.role+'->'+$Peer)}
Invoke-Wizard
if($script:Events.Count -ne 2 -or $script:Events[0] -ne 'remote->24:62:AB:F1:36:64' -or $script:Events[1] -ne 'boat->24:62:AB:EF:CF:84'){throw '流程配对地址不正确'}
$script:Mode='same';$script:Events=@();$rejected=$false
try{Invoke-Wizard}catch{$rejected=$true}
if(!$rejected -or $script:Events.Count -ne 0){throw '重复登记同一芯片时未阻止写入'}
$script:Mode='wrong-third';$script:Events=@();$rejected=$false
try{Invoke-Wizard}catch{$rejected=$true}
if(!$rejected -or $script:Events.Count -ne 1){throw '最后插错小船时未阻止写入'}
$rejected=$false
try{Assert-Board @{role='remote';mac='24:62:AB:EF:CF:84';version='2';steer='20'} 'boat'}catch{$rejected=$true}
if(!$rejected){throw '未拒绝错误角色'}
Write-Output '通过：流程配对地址、同一芯片重复登记拦截、最后插错板拦截、角色不符拦截；未访问真实串口。'

$rejected=$false
try{Assert-Board @{role='boat';mac='24:62:AB:F1:36:64';version='2';steer='30'} 'boat'}catch{$rejected=$true}
if(!$rejected){throw '没有拒绝旧30度固件'}
$rejected=$false
try{Assert-Board @{role='boat';mac='24:62:AB:F1:36:64';version='2'} 'boat'}catch{$rejected=$true}
if(!$rejected){throw '没有拒绝缺少角度标记的旧固件'}
Write-Output '通过：旧30度与无角度标记固件必须升级，不能跳过。'
