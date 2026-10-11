# Restriction only. Retain the target image's existing native ancestry custody;
# never derive a later marker lookup from a mutable DOS/SUBST request alias.
# Absence is checked at a point in time, not continuously pinned or authority.
function Initialize-InstalledLaunchBarrierType {
  if('OpenNavX.InstalledLaunchBarrierNative' -as [type]){return}
  Add-Type -TypeDefinition @'
using System;
using System.IO;
using System.Text;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
namespace OpenNavX {
 public static class InstalledLaunchBarrierNative {
  [StructLayout(LayoutKind.Sequential,Pack=4)] struct Info {
   public uint Attributes;public long Created,Accessed,Written;
   public uint Volume,SizeHigh,SizeLow,Links,IndexHigh,IndexLow;
  }
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true,ExactSpelling=true)] static extern uint GetFileAttributesW(string path);
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] static extern SafeFileHandle CreateFile(string path,uint access,uint share,IntPtr security,uint disposition,uint flags,IntPtr template);
  [DllImport("kernel32.dll",SetLastError=true)] static extern bool GetFileInformationByHandle(SafeFileHandle file,out Info info);
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] static extern uint GetFinalPathNameByHandle(SafeFileHandle file,StringBuilder name,uint capacity,uint flags);
  public static string ManagedRoot(string nativeImage,string generation,string filename) {
   if(!System.Text.RegularExpressions.Regex.IsMatch(generation??"","^[a-f0-9]{32}$") ||
      (filename!="opencpn.exe" && filename!="skager-start.exe") || nativeImage==null ||
      !System.Text.RegularExpressions.Regex.IsMatch(nativeImage,@"^\\Device\\HarddiskVolume[0-9]+\\")) throw new IOException("Held managed image identity required.");
   string suffix="\\generations\\"+generation+"\\app\\"+filename;
   if(!nativeImage.EndsWith(suffix,StringComparison.OrdinalIgnoreCase) || nativeImage.Length<=suffix.Length) throw new IOException("Held image escaped the exact installed generation.");
   string root=nativeImage.Substring(0,nativeImage.Length-suffix.Length);
   if(!System.Text.RegularExpressions.Regex.IsMatch(root,@"^\\Device\\HarddiskVolume[0-9]+\\.+"))throw new IOException("Installation cannot be a volume root.");
   // Full component/NTFS/ancestor validation belongs to UpdateImageOwner.
   // No independent DOS root or a newly resolved alias enters this lookup.
   return @"\\?\GLOBALROOT"+root;
  }
  static void CheckAbsent(string image,string generation,string filename,Func<string,uint> read) {
   string root=ManagedRoot(image,generation,filename);
   uint attributes=read(root);
   if((attributes&0x410)!=0x10)throw new IOException("Held installation root is not a plain directory.");
   try {read(root+"\\signed-update-armed.jsonl");}
   catch(System.ComponentModel.Win32Exception error){if(error.NativeErrorCode==2)return;throw;}
   throw new IOException("Signed update session is armed; installed launch and restart are restricted.");
  }
  public static void AssertAbsent(string image,string generation,string filename) {
   CheckAbsent(image,generation,filename,delegate(string path){uint value=GetFileAttributesW(path);if(value==0xffffffff)throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error());return value;});
  }
  public static void AssertRequestedImage(string requested,string nativeImage) {
   using(var file=CreateFile(requested,0x80000000,1,IntPtr.Zero,3,0x00200000,IntPtr.Zero)) {
    if(file.IsInvalid)throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error());
    Info info;if(!GetFileInformationByHandle(file,out info))throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error());
    var name=new StringBuilder(32768);uint n=GetFinalPathNameByHandle(file,name,(uint)name.Capacity,2);
    if(n==0 || n>=(uint)name.Capacity || (info.Attributes&0x410)!=0 || info.Links!=1 ||
       !String.Equals(name.ToString(),nativeImage,StringComparison.Ordinal))throw new IOException("Requested image no longer resolves to the held installed image.");
   }
  }
 }
}
'@
}
function Open-InstalledLaunchBarrier($Installed,[string]$ImageName='opencpn.exe') {
  if([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT){throw 'Installed launch restriction requires native Windows.'}
  if($ImageName -cnotin @('opencpn.exe','skager-start.exe') -or $Installed.state.current -cnotmatch '^[a-f0-9]{32}$'){throw 'Exact managed launch target required.'}
  $image=Join-Path $Installed.generation ('app/'+$ImageName)
  $imageEntry=@($Installed.ownership.managedFiles|Where-Object {$_.path -ceq ('app/'+$ImageName)})
  $moduleEntry=@($Installed.ownership.managedFiles|Where-Object {$_.path -ceq 'UpdateTransaction.ps1'})
  if($imageEntry.Count -ne 1 -or $moduleEntry.Count -ne 1 -or $imageEntry[0].sha256 -cnotmatch '^[a-f0-9]{64}$' -or $moduleEntry[0].sha256 -cnotmatch '^[a-f0-9]{64}$'){throw 'Owned native image custody module and target required.'}
  $module=Join-Path $Installed.generation 'UpdateTransaction.ps1'
  $heldModule=[IO.File]::Open($module,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
  try {
    if($heldModule.Length -le 0 -or $heldModule.Length -gt 1048576){throw 'Installed native image custody module exceeds bounds.'}
    $memory=New-Object IO.MemoryStream
    try {$heldModule.CopyTo($memory);$bytes=$memory.ToArray()} finally {$memory.Dispose()}
    $sha=[Security.Cryptography.SHA256]::Create()
    try {$digest=([BitConverter]::ToString($sha.ComputeHash($bytes))).Replace('-','').ToLowerInvariant()} finally {$sha.Dispose()}
    if($digest -cne $moduleEntry[0].sha256){throw 'Installed native image custody module changed.'}
    # Execute the exact hashed held bytes, never re-open the mutable request
    # alias for dot-sourcing. This module has no PSScriptRoot dependencies.
    . ([scriptblock]::Create(([Text.UTF8Encoding]::new($false,$true)).GetString($bytes).TrimStart([char]0xfeff)))
    Initialize-UpdatePipeType
  } finally {$heldModule.Dispose()}
  Initialize-InstalledLaunchBarrierType
  $owner=[Skager.UpdateImageOwner]::Acquire($image,[IO.Path]::GetDirectoryName($image),$imageEntry[0].sha256)
  try {
    $guard=[pscustomobject]@{owner=$owner;generation=$Installed.state.current;imageName=$ImageName}
    Assert-InstalledLaunchBarrier $guard
    return $guard
  } catch {$owner.Dispose();throw}
}
function Assert-InstalledLaunchBarrier($Guard) {
  $Guard.owner.Validate()
  # Check the held A root first, even if the request alias now selects B.
  [OpenNavX.InstalledLaunchBarrierNative]::AssertAbsent($Guard.owner.NativeImage,$Guard.generation,$Guard.imageName)
  [OpenNavX.InstalledLaunchBarrierNative]::AssertRequestedImage($Guard.owner.OriginalImage,$Guard.owner.NativeImage)
}
