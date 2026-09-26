[CmdletBinding()]
param([string]$Workspace='C:\XNav',[Parameter(Mandatory=$true)][int]$ProcessId,[ValidatePattern('^[a-z0-9-]+$')][string]$Name='navigation')
. (Join-Path $PSScriptRoot 'Common.ps1')
$installed=Get-Installed;$directory=New-RunDirectory $Workspace 'capture'
$result=Invoke-InteractiveJob $Workspace ([pscustomobject]@{action='Capture';executable=$installed.executable;executableSha256=(Get-Digest $installed.executable);processId=$ProcessId;imagePath=(Join-Path $directory ($Name+'.png'))})
$result | Add-Member -NotePropertyName buildCommit -NotePropertyValue $installed.ownership.commit
Write-Record (Join-Path $directory 'capture.json') $result
$result | ConvertTo-Json -Depth 8
