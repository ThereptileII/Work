# SCRUM-311 persistence foundation only: NO artifact authentication, IPC,
# installation, process creation, launch permission or interrupted-update recovery.
# A trusted same-process native caller must validate real evidence separately.
# New-SignedUpdateStore accepts a pre-created private directory; it NEVER resumes
# an existing armed file. Close/failure NEVER deletes it. A future integrated gate
# MUST deny ordinary startup when the marker/custodian is missing or abandoned.
# Disk snapshots alone cannot prove freshness after owner death or whole-disk
# rollback. Receipts returned here are consumed records, NOT reusable allow tokens.
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'SignedUpdateOperations.ps1')
# Do not reset custody when this file is dot-sourced twice in the same scope.
if(-not (Get-Variable SignedUpdateStores -Scope Script -ErrorAction SilentlyContinue)){$script:SignedUpdateStores=@{}}
if(-not ('SignedUpdateStoreNative' -as [type])) {
Add-Type -TypeDefinition @'
using System;
using System.IO;
using System.Text;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
public static class SignedUpdateStoreNative {
  // Validate BEFORE PowerShell's parser, which otherwise collapses duplicate keys.
  // This protocol deliberately accepts integer numbers only, no exponent/fraction.
  public static string Json(byte[] bytes) {
    if(bytes==null || bytes.Length==0 || bytes.Length>65536) throw new InvalidDataException("JSON size.");
    string s=new UTF8Encoding(false,true).GetString(bytes);
    new Parser(s).Run(); return s;
  }
  sealed class Parser {
    readonly string s; int p, nodes;
    public Parser(string text){s=text;}
    void Fail(){throw new InvalidDataException("Invalid or ambiguous bounded JSON.");}
    void Space(){while(p<s.Length && (s[p]==' '||s[p]=='\r'||s[p]=='\n'||s[p]=='\t'))p++;}
    bool Take(char c){Space();if(p<s.Length&&s[p]==c){p++;return true;}return false;}
    public void Run(){Value(0);Space();if(p!=s.Length)Fail();}
    string Str(){
      if(!Take('"'))Fail();var b=new StringBuilder();bool done=false;
      while(p<s.Length){char c=s[p++];if(c=='"'){done=true;break;}if(c<32)Fail();
        if(c=='\\'){if(p==s.Length)Fail();c=s[p++];switch(c){
          case '"':case '\\':case '/':break;case 'b':c='\b';break;case 'f':c='\f';break;
          case 'n':c='\n';break;case 'r':c='\r';break;case 't':c='\t';break;
          case 'u': if(p+4>s.Length)Fail();int n=0;for(int i=0;i<4;i++){
            char h=s[p++];int v=h>='0'&&h<='9'?h-'0':h>='a'&&h<='f'?h-'a'+10:h>='A'&&h<='F'?h-'A'+10:-1;
            if(v<0)Fail();n=n*16+v;}c=(char)n;break;default:Fail();break;}}
        b.Append(c);if(b.Length>8192)Fail();
      }
      if(!done)Fail();string value=b.ToString();
      for(int i=0;i<value.Length;i++)if(char.IsSurrogate(value[i])){
        if(!char.IsHighSurrogate(value[i])||i+1==value.Length||!char.IsLowSurrogate(value[++i]))Fail();}
      return value;
    }
    void Value(int depth){
      if(depth>12 || ++nodes>4096)Fail();Space();if(p==s.Length)Fail();char c=s[p];
      if(c=='{'){p++;var keys=new HashSet<string>(StringComparer.OrdinalIgnoreCase);if(Take('}'))return;
        do {string k=Str();if(k.Length==0||k.Length>128||!keys.Add(k))Fail();if(!Take(':'))Fail();Value(depth+1);
          if(Take('}'))return;if(!Take(','))Fail();}while(true);
      }
      if(c=='['){p++;if(Take(']'))return;do{Value(depth+1);if(Take(']'))return;if(!Take(','))Fail();}while(true);}
      if(c=='"'){Str();return;}
      foreach(string literal in new[]{"true","false","null"})if(p+literal.Length<=s.Length && String.CompareOrdinal(s,p,literal,0,literal.Length)==0){p+=literal.Length;return;}
      int start=p;if(c=='-')p++;if(p==s.Length||s[p]<'0'||s[p]>'9')Fail();
      if(s[p]=='0')p++;else while(p<s.Length&&s[p]>='0'&&s[p]<='9')p++;
      long parsed;if(!Int64.TryParse(s.Substring(start,p-start),System.Globalization.NumberStyles.AllowLeadingSign,System.Globalization.CultureInfo.InvariantCulture,out parsed))Fail();
    }
  }
  [StructLayout(LayoutKind.Sequential)] struct Info {
    public uint Attributes; public System.Runtime.InteropServices.ComTypes.FILETIME Creation,Access,Write;
    public uint Volume,SizeHigh,SizeLow,Links,IndexHigh,IndexLow;
  }
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)]
  static extern SafeFileHandle CreateFile(string path,uint access,uint share,IntPtr security,uint mode,uint flags,IntPtr template);
  [DllImport("kernel32.dll",SetLastError=true)] static extern bool GetFileInformationByHandle(SafeFileHandle h,out Info info);
  [StructLayout(LayoutKind.Sequential)] struct SecurityAttributes {
    public int Length; public IntPtr Descriptor; [MarshalAs(UnmanagedType.Bool)] public bool Inherit;
  }
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true,EntryPoint="CreateFileW")]
  static extern SafeFileHandle CreatePrivateFile(string path,uint access,uint share,ref SecurityAttributes security,uint mode,uint flags,IntPtr template);
  [DllImport("advapi32.dll",SetLastError=true)] static extern uint GetSecurityInfo(SafeFileHandle h,int kind,uint flags,out IntPtr owner,out IntPtr group,out IntPtr dacl,out IntPtr sacl,out IntPtr descriptor);
  [DllImport("advapi32.dll")] static extern uint GetSecurityDescriptorLength(IntPtr descriptor);
  [DllImport("kernel32.dll")] static extern IntPtr LocalFree(IntPtr pointer);
  public static byte[] SecurityDescriptor(SafeFileHandle h){
    IntPtr owner,group,dacl,sacl,descriptor;
    uint error=GetSecurityInfo(h,1,5,out owner,out group,out dacl,out sacl,out descriptor);
    if(error!=0)throw new System.ComponentModel.Win32Exception((int)error);
    try{uint length=GetSecurityDescriptorLength(descriptor);if(length==0||length>65536)throw new IOException("Security descriptor bound.");
      byte[] bytes=new byte[length];Marshal.Copy(descriptor,bytes,0,bytes.Length);return bytes;
    }finally{LocalFree(descriptor);}
  }
  public static FileStream CreateJournal(string path,byte[] descriptor){
    if(descriptor==null||descriptor.Length==0||descriptor.Length>65536)throw new IOException("Private descriptor required.");
    IntPtr memory=Marshal.AllocHGlobal(descriptor.Length);SafeFileHandle h=null;
    try{Marshal.Copy(descriptor,0,memory,descriptor.Length);
      var attributes=new SecurityAttributes{Length=Marshal.SizeOf(typeof(SecurityAttributes)),Descriptor=memory,Inherit=false};
      // CREATE_NEW, exclusive sharing, write-through and open-reparse-point.
      h=CreatePrivateFile(path,0xC0000000,0,ref attributes,1,0x80200000,IntPtr.Zero);
      if(h.IsInvalid)throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error());
      Check(h,false);var stream=new FileStream(h,FileAccess.ReadWrite,4096,false);h=null;return stream;
    }finally{if(h!=null)h.Dispose();Marshal.FreeHGlobal(memory);}
  }
  public static void Check(SafeFileHandle h,bool directory){
    Info i;if(h.IsClosed||h.IsInvalid||!GetFileInformationByHandle(h,out i))throw new IOException("Custody handle lost.");
    if((i.Attributes&0x400)!=0 || ((i.Attributes&0x10)!=0)!=directory || (!directory&&i.Links!=1))throw new IOException("Reparse/type/hardlink refused.");
  }
  public static SafeFileHandle[] PinDirectories(string path){
    if(!System.Text.RegularExpressions.Regex.IsMatch(path,@"^[A-Za-z]:\\") || path.Length>240 || path!=Path.GetFullPath(path) || path.IndexOf('/')>=0)throw new IOException("Canonical local directory required.");
    var drive=new DriveInfo(Path.GetPathRoot(path));if(drive.DriveType!=DriveType.Fixed||drive.DriveFormat!="NTFS")throw new IOException("Local fixed NTFS required.");
    var pins=new List<SafeFileHandle>();try{
      string current=Path.GetPathRoot(path);var parts=path.Substring(current.Length).Split('\\');
      for(int n=-1;n<parts.Length;n++){
        if(n>=0){string part=parts[n];if(part.Length==0||part=="."||part==".."||part.EndsWith(".")||part.EndsWith(" ")||part.IndexOf(':')>=0)throw new IOException("Noncanonical directory.");current=Path.Combine(current,part);}
        // Pin each ancestor before resolving its child. Deny directory rename/delete.
        var h=CreateFile(current,0x20080,3,IntPtr.Zero,3,0x02200000,IntPtr.Zero);pins.Add(h);Check(h,true);
      }return pins.ToArray();
    }catch{foreach(var h in pins)h.Dispose();throw;}
  }
}
'@
}
function ConvertFrom-SignedStoreJson([byte[]]$Bytes) {
  $text=[SignedUpdateStoreNative]::Json($Bytes)
  if($text.TrimStart()[0] -cne '{'){throw 'Protocol root must be one object.'}
  return ConvertFrom-Json -InputObject $text
}
function Get-SignedStoreBytes($Value) {return ,([Text.UTF8Encoding]::new($false,$true).GetBytes(($Value|ConvertTo-Json -Compress -Depth 16)))}
function Get-SignedStoreDigest([byte[]]$Bytes) {
  $sha=[Security.Cryptography.SHA256]::Create()
  try{return ([BitConverter]::ToString($sha.ComputeHash($Bytes))).Replace('-','').ToLowerInvariant()}finally{$sha.Dispose()}
}
function Get-SignedStoreNow {return [long][Math]::Floor(([DateTime]::UtcNow-[DateTime]::new(1970,1,1,0,0,0,[DateTimeKind]::Utc)).TotalSeconds)}
function Get-SignedStoreOwner {
  if([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT){throw 'Native Windows custody is required.'}
  $p=[Diagnostics.Process]::GetCurrentProcess()
  try {
    $image=$p.MainModule.FileName
    $stream=[IO.File]::Open($image,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
    try {
      if($stream.Length -gt 67108864){throw 'Owner image exceeds bound.'}
      $sha=[Security.Cryptography.SHA256]::Create()
      try{$digest=([BitConverter]::ToString($sha.ComputeHash($stream))).Replace('-','').ToLowerInvariant()}finally{$sha.Dispose()}
    } finally {$stream.Dispose()}
    return [pscustomobject][ordered]@{sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value;pid=$PID;createdFiletime=$p.StartTime.ToUniversalTime().ToFileTimeUtc().ToString();sessionId=$p.SessionId;image=$image;imageSha256=$digest}
  } finally {$p.Dispose()}
}
function Assert-SignedStoreAcl($Handle,[string]$Sid,[bool]$Directory) {
  if($Directory){$acl=[Security.AccessControl.DirectorySecurity]::new()}else{$acl=[Security.AccessControl.FileSecurity]::new()}
  $acl.SetSecurityDescriptorBinaryForm([SignedUpdateStoreNative]::SecurityDescriptor($Handle))
  if($acl.GetOwner([Security.Principal.SecurityIdentifier]).Value -cne $Sid){throw 'Custody owner differs.'}
  if(-not $acl.AreAccessRulesProtected){throw 'Custody requires a protected ACL.'}
  $seen=@{}
  foreach($rule in $acl.GetAccessRules($true,$true,[Security.Principal.SecurityIdentifier])) {
    if($rule.IdentityReference.Value -cnotin @($Sid,'S-1-5-18') -or $rule.AccessControlType -ne [Security.AccessControl.AccessControlType]::Allow -or
       $rule.FileSystemRights -ne [Security.AccessControl.FileSystemRights]::FullControl -or
       $rule.PropagationFlags -ne [Security.AccessControl.PropagationFlags]::None){throw 'Unexpected custody ACL.'}
    if($Directory -and $rule.InheritanceFlags -ne ([Security.AccessControl.InheritanceFlags]::ContainerInherit -bor [Security.AccessControl.InheritanceFlags]::ObjectInherit)){throw 'Custody directory must protect child files.'}
    $seen[$rule.IdentityReference.Value]=$true
  }
  if(-not $seen.ContainsKey($Sid) -or -not $seen.ContainsKey('S-1-5-18')){throw 'Required private ACL principals missing.'}
}
function Enter-SignedStoreDirectory([string]$Directory,$Owner) {
  $pins=[SignedUpdateStoreNative]::PinDirectories($Directory)
  try {Assert-SignedStoreAcl $pins[-1] $Owner.sid $true;return ,$pins}
  catch {foreach($pin in $pins){$pin.Dispose()};throw}
}
function New-SignedStoreFile([string]$Path,[string]$Sid) {
  # Atomically create with the exact owner/protected DACL, including when an
  # elevated token would default its file owner to Administrators. Never reopen.
  $acl=[Security.AccessControl.FileSecurity]::new()
  $acl.SetOwner([Security.Principal.SecurityIdentifier]::new($Sid))
  $acl.SetAccessRuleProtection($true,$false)
  foreach($principal in @($Sid,'S-1-5-18')) {
    $acl.AddAccessRule([Security.AccessControl.FileSystemAccessRule]::new([Security.Principal.SecurityIdentifier]::new($principal),[Security.AccessControl.FileSystemRights]::FullControl,[Security.AccessControl.AccessControlType]::Allow))
  }
  return [SignedUpdateStoreNative]::CreateJournal($Path,$acl.GetSecurityDescriptorBinaryForm())
}
function Assert-SignedStoreNative($Store) {
  foreach($pin in $Store.pins){[SignedUpdateStoreNative]::Check($pin,$true)}
  [SignedUpdateStoreNative]::Check($Store.stream.SafeFileHandle,$false)
  Assert-SignedStoreAcl $Store.pins[-1] $Store.owner.sid $true
  Assert-SignedStoreAcl $Store.stream.SafeFileHandle $Store.owner.sid $false
}
function Write-SignedStoreFrame($Store,[byte[]]$Bytes) {
  # One append contains BOTH complete state and its current head; there is no
  # independently replaceable head. Any partial append/flush failure poisons the
  # live owner; restart cannot resume even an apparently complete last frame.
  $Store.stream.Position=$Store.stream.Length
  $Store.stream.Write($Bytes,0,$Bytes.Length)
  $Store.stream.Flush($true)
}
function Read-SignedStoreHeldBytes($Store) {
  if($Store.stream.Length -lt 1 -or $Store.stream.Length -gt 262144){throw 'Armed journal size differs.'}
  $data=[byte[]]::new([int]$Store.stream.Length);$Store.stream.Position=0;$offset=0
  while($offset -lt $data.Length){$n=$Store.stream.Read($data,$offset,$data.Length-$offset);if($n -le 0){throw 'Incomplete armed journal.'};$offset+=$n}
  return ,$data
}
function Assert-SignedStoreCurrent($Store) {
  if($Store.poisoned -or $Store.stream.SafeFileHandle.IsClosed){throw 'Armed custody is abandoned; no resume or ordinary-launch authority.'}
  try {
    if((Get-SignedUpdateReceiptHash (Get-SignedStoreOwner)) -cne $Store.ownerSha256){throw 'Owner SID/session/process/image changed.'}
    Assert-SignedStoreNative $Store
    $bytes=Read-SignedStoreHeldBytes $Store
    if($bytes.Length -ne $Store.length -or (Get-SignedStoreDigest $bytes) -cne $Store.digest){throw 'Armed journal/head changed or replayed.'}
    $now=Get-SignedStoreNow
    $null=New-SignedUpdateSession $Store.session.binding $now
    if($now -lt $Store.session.lastUnix -or ($null -ne $Store.operations -and $now -lt $Store.operations.lastUnix)){throw 'Clock moved backward.'}
    return $now
  } catch {$Store.poisoned=$true;throw}
}
function Get-SignedStoreLive([string]$Handle) {
  if(-not $script:SignedUpdateStores.ContainsKey($Handle)){throw 'No exclusive live signed-update custody. Disk records cannot resume it.'}
  return $script:SignedUpdateStores[$Handle]
}
function New-SignedUpdateStore([string]$Directory,[byte[]]$BindingJson,[switch]$OperationEvents) {
  $binding=ConvertFrom-SignedStoreJson $BindingJson
  $session=New-SignedUpdateSession $binding (Get-SignedStoreNow)
  $operations=$null
  if($OperationEvents){$operations=New-SignedUpdateOperationSession $binding $session.lastUnix;$session=$operations.session}
  $owner=Get-SignedStoreOwner
  $pins=Enter-SignedStoreDirectory $Directory $owner
  $stream=$null
  try {
    $path=Join-Path $Directory 'signed-update-armed.jsonl'
    # CreateNew refuses missing-custodian leftovers, including corrupt/empty files.
    $stream=New-SignedStoreFile $path $owner.sid
    $store=@{directory=$Directory;path=$path;stream=$stream;pins=$pins;owner=$owner;ownerSha256=(Get-SignedUpdateReceiptHash $owner);session=$session;operations=$operations;poisoned=$false;length=0;digest=('0'*64)}
    Assert-SignedStoreNative $store
    $frame=[pscustomobject][ordered]@{schema=1;owner=$owner;sequence=0;previousFrameSha256=('0'*64);state=$session}
    if($OperationEvents){$frame.schema=2;$frame.state=$operations}
    $bytes=Get-SignedStoreBytes $frame;$bytes=[byte[]]($bytes+10)
    Write-SignedStoreFrame $store $bytes
    $store.length=$bytes.Length;$store.digest=Get-SignedStoreDigest $bytes
    $null=Assert-SignedStoreCurrent $store
    $handle=[guid]::NewGuid().ToString('N')
    $script:SignedUpdateStores.Add($handle,$store)
    return $handle
  } catch {
    if($stream){$stream.Dispose()};foreach($pin in $pins){$pin.Dispose()}
    # Preserve any created marker, even on initial write failure.
    throw
  }
}
function Add-SignedUpdateStoreReceipt([string]$Handle,[byte[]]$RequestJson) {
  $store=Get-SignedStoreLive $Handle
  $now=Assert-SignedStoreCurrent $store
  if($null -ne $store.operations){throw 'Operation-event custody requires Add-SignedUpdateStoreEvent; receipt-only bypass refused.'}
  $request=ConvertFrom-SignedStoreJson $RequestJson
  $next=Copy-SignedUpdateValue $store.session
  $receipt=Add-SignedUpdateReceipt $next $request $now
  Write-SignedStoreTransition $store $next $null
  return Copy-SignedUpdateValue $receipt
}
function Add-SignedUpdateStoreEvent([string]$Handle,[byte[]]$RequestJson) {
  $store=Get-SignedStoreLive $Handle
  $now=Assert-SignedStoreCurrent $store
  if($null -eq $store.operations){throw 'Operation events require an explicitly armed operation store.'}
  $request=ConvertFrom-SignedStoreJson $RequestJson
  $next=Copy-SignedUpdateValue $store.operations
  $event=Add-SignedUpdateOperationEvent $next $request $now
  Write-SignedStoreTransition $store $next.session $next
  return Copy-SignedUpdateValue $event
}
function Write-SignedStoreTransition($Store,$NextSession,$Operations) {
  $frame=[pscustomobject][ordered]@{schema=1;owner=$Store.owner;sequence=@($NextSession.receipts).Count;previousFrameSha256=$Store.digest;state=$NextSession}
  if($null -ne $Operations){$frame.schema=2;$frame.sequence=@($Operations.events).Count;$frame.state=$Operations}
  $bytes=Get-SignedStoreBytes $frame;$bytes=[byte[]]($bytes+10)
  try {
    # Retain the prior complete journal in memory as the anti-replay anchor.
    $old=Read-SignedStoreHeldBytes $store
    $expected=[byte[]]($old+$bytes)
    if($expected.Length -gt 262144){throw 'Journal bound exceeded.'}
    Write-SignedStoreFrame $store $bytes
    $store.length=$expected.Length;$store.digest=Get-SignedStoreDigest $expected;$store.session=$NextSession;$store.operations=$Operations
    $null=Assert-SignedStoreCurrent $store
  } catch {$store.poisoned=$true;throw}
  # Transition is consumed durably before either API can return a result.
}
function Close-SignedUpdateStore([string]$Handle) {
  $store=Get-SignedStoreLive $Handle
  $store.poisoned=$true
  try {$store.stream.Dispose()}finally{foreach($pin in $store.pins){$pin.Dispose()};$script:SignedUpdateStores.Remove($Handle)}
  # Intentionally leave armed evidence. No disarm/delete/resume API exists.
}
