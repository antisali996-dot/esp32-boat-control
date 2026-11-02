. (Join-Path $PSScriptRoot '..\handoff\wizard.ps1') -LoadOnly
function Fake-Serial([string]$Line){
 $o=New-Object psobject -Property @{Line=$Line}
 $o | Add-Member ScriptMethod DiscardInBuffer {}
 $o | Add-Member ScriptMethod WriteLine {param($value)}
 $o | Add-Member ScriptMethod ReadLine {return $this.Line}
 return $o
}
$current=Read-SerialInfo (Fake-Serial 'JAY ROLE=BOAT MAC=24:62:AB:F1:36:64 PEER=24:62:AB:EF:CF:84 VERSION=2 STEER=20')
Assert-Board $current 'boat' '24:62:AB:F1:36:64'
if($current.steer -ne '20'){throw '没有正确解析20度标记'}
$old=Read-SerialInfo (Fake-Serial 'JAY ROLE=BOAT MAC=24:62:AB:F1:36:64 PEER=24:62:AB:EF:CF:84 VERSION=2')
$rejected=$false;try{Assert-Board $old 'boat'}catch{$rejected=$true}
if(!$rejected){throw '旧程序被错误当作20度程序'}
Write-Output '通过：真实身份行格式的20度标记读取、旧固件升级识别；无串口访问。'
