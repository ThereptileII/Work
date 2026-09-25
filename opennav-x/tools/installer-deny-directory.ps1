# Test fixture only. Uses Windows PowerShell 5.1's .NET Framework ACL API,
# independent of any PowerShell 7 PSModulePath inherited by the native process.
param([Parameter(Mandatory=$true)][string]$Directory,
      [Parameter(Mandatory=$true)][string]$Saved,
      [switch]$Restore)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
if ($env:GITHUB_ACTIONS -ne 'true' -or $env:OS -ne 'Windows_NT') { throw 'Disposable Windows CI only.' }
if ($PSVersionTable.PSVersion.Major -ne 5) { throw 'Use native Windows PowerShell 5.1.' }
$Sections = [Security.AccessControl.AccessControlSections]::Access
$Acl = [IO.Directory]::GetAccessControl($Directory, $Sections)
if ($Restore) {
  $Acl.SetSecurityDescriptorSddlForm([IO.File]::ReadAllText($Saved), $Sections)
  [IO.Directory]::SetAccessControl($Directory, $Acl)
  return
}
[IO.File]::WriteAllText($Saved, $Acl.GetSecurityDescriptorSddlForm($Sections))
$Sid = [Security.Principal.WindowsIdentity]::GetCurrent().User
$Rule = New-Object System.Security.AccessControl.FileSystemAccessRule($Sid, [Security.AccessControl.FileSystemRights]::CreateDirectories, [Security.AccessControl.AccessControlType]::Deny)
$Acl.AddAccessRule($Rule)
[IO.Directory]::SetAccessControl($Directory, $Acl)
