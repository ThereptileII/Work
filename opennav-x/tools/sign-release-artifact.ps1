# Explicit operator tool. Importing this file performs no certificate/key operation.
[CmdletBinding()]
param(
  [ValidateSet('Prepare','Sign')][Alias('Mode')][string]$SigningMode='Prepare',
  [Alias('InputFile')][string]$SigningInput,
  [Alias('InputSha256')][string]$SigningInputHash,
  [Alias('OutputDirectory')][string]$SigningOutput,
  [Alias('CertificateThumbprint')][string]$SigningCertificate,
  [Alias('PublisherSubject')][string]$SigningPublisher,
  [Alias('SignTool')][string]$SigningTool,
  [Alias('SignToolSha256')][string]$SigningToolHash,
  [Alias('TimestampUrl')][string]$SigningTimestamp
)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'

function Assert-SigningPolicy([string]$InputHash,[string]$ToolHash,[string]$Thumbprint,[string]$Publisher,[string]$Timestamp) {
  if ($InputHash -cnotmatch '^[0-9a-f]{64}$' -or $ToolHash -cnotmatch '^[0-9a-f]{64}$') { throw 'Explicit lowercase SHA256 pins are required.' }
  if ($Thumbprint -cnotmatch '^[0-9A-Fa-f]{40}$') { throw 'An exact certificate thumbprint is required.' }
  if ([string]::IsNullOrWhiteSpace($Publisher) -or $Publisher.Length -gt 1024 -or $Publisher -match '[\x00-\x1f\x7f]') { throw 'An exact publisher subject is required.' }
  $uri=$null
  if ($Timestamp.Length -gt 2048 -or $Timestamp -match '[\s\\"\x00-\x1f\x7f]' -or
      -not [Uri]::TryCreate($Timestamp,[UriKind]::Absolute,[ref]$uri) -or
      $uri.Scheme -cne 'https' -or $uri.Port -ne 443 -or $uri.UserInfo -or $uri.Query -or $uri.Fragment -or
      $uri.HostNameType -ne [UriHostNameType]::Dns -or $uri.IsLoopback -or $uri.Host -notmatch '\.') {
    throw 'Timestamp endpoint must be an explicit HTTPS DNS URL on port 443 without credentials, query or fragment.'
  }
}

function Assert-SigningPath([string]$Path,[switch]$NewDirectory) {
  if ([string]::IsNullOrWhiteSpace($Path) -or -not [IO.Path]::IsPathRooted($Path) -or $Path -match '["\x00-\x1f\x7f]') { throw 'Signing paths must be explicit absolute local paths.' }
  if ([Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT -and $Path -notmatch '^[A-Za-z]:\\') { throw 'Drive-relative and nonlocal paths are forbidden.' }
  $full=[IO.Path]::GetFullPath($Path)
  if ([Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT -and ($full -notmatch '^[A-Za-z]:\\' -or $full.Substring(2).Contains(':'))) { throw 'UNC, device and alternate-stream paths are forbidden.' }
  $cursor=$full
  while ($cursor) {
    if (Test-Path -LiteralPath $cursor) {
      if (((Get-Item -LiteralPath $cursor -Force).Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) { throw 'Signing paths must not traverse symlinks or reparse points.' }
    }
    $next=[IO.Path]::GetDirectoryName($cursor)
    if ($next -eq $cursor) { break }; $cursor=$next
  }
  if ($NewDirectory) {
    if (Test-Path -LiteralPath $full) { throw 'Output directory already exists; nothing will be overwritten.' }
    if (-not (Test-Path -LiteralPath ([IO.Path]::GetDirectoryName($full)) -PathType Container)) { throw 'Output parent must already exist.' }
  } elseif (-not (Test-Path -LiteralPath $full -PathType Leaf)) { throw 'Required signing file is absent.' }
  return $full
}

function Get-SigningStreamHash([IO.Stream]$Stream) {
  $sha=[Security.Cryptography.SHA256]::Create()
  try { $Stream.Position=0; return ([BitConverter]::ToString($sha.ComputeHash($Stream))).Replace('-','').ToLowerInvariant() }
  finally { $Stream.Position=0; $sha.Dispose() }
}

function Assert-SigningPE([IO.Stream]$Stream) {
  if ($Stream.Length -lt 256 -or $Stream.Length -gt 2147483648) { throw 'PE input is outside the supported size bounds.' }
  $reader=New-Object IO.BinaryReader($Stream,[Text.Encoding]::ASCII,$true)
  try {
    $Stream.Position=0
    if ($reader.ReadUInt16() -ne 0x5a4d) { throw 'Input is not a PE artifact.' }
    $Stream.Position=0x3c; $offset=$reader.ReadUInt32()
    if ($offset -lt 64 -or $offset -gt $Stream.Length-26) { throw 'Invalid PE header.' }
    $Stream.Position=$offset
    if ($reader.ReadUInt32() -ne 0x4550) { throw 'Invalid PE signature.' }
    $Stream.Position=$offset+24; $magic=$reader.ReadUInt16()
    if ($magic -notin @(0x10b,0x20b)) { throw 'Unsupported PE optional header.' }
  } finally { $reader.Dispose(); $Stream.Position=0 }
}

function Initialize-SigningWinTrust {
  if ('Skager.SigningTrust' -as [type]) { return }
  Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
namespace Skager {
  public static class SigningTrust {
    [StructLayout(LayoutKind.Sequential)]
    struct SecurityAttributes { public int Size; public IntPtr Descriptor; public int Inherit; }
    [DllImport("advapi32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    static extern bool ConvertStringSecurityDescriptorToSecurityDescriptor(string text,uint revision,out IntPtr descriptor,out uint size);
    [DllImport("kernel32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    static extern bool CreateDirectory(string path,ref SecurityAttributes security);
    [DllImport("kernel32.dll")] static extern IntPtr LocalFree(IntPtr value);
    public static void CreatePrivateDirectory(string path,string sid) {
      IntPtr descriptor=IntPtr.Zero; uint size;
      if (!ConvertStringSecurityDescriptorToSecurityDescriptor("D:P(A;OICI;FA;;;"+sid+")(A;OICI;FA;;;SY)",1,out descriptor,out size))
        throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error());
      try {
        var security=new SecurityAttributes { Size=Marshal.SizeOf(typeof(SecurityAttributes)), Descriptor=descriptor };
        // Atomic creation with protected DACL, and failure if path exists.
        if (!CreateDirectory(path,ref security)) throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error());
      } finally { LocalFree(descriptor); }
    }
    [StructLayout(LayoutKind.Sequential, CharSet=CharSet.Unicode)]
    struct FileInfo { public uint Size; [MarshalAs(UnmanagedType.LPWStr)] public string Path; public IntPtr File; public IntPtr Subject; }
    [StructLayout(LayoutKind.Sequential, CharSet=CharSet.Unicode)]
    struct TrustData {
      public uint Size; public IntPtr Policy, SIP; public uint UI, Revocation, Choice;
      public IntPtr File; public uint Action; public IntPtr State, URL; public uint Flags, Context; public IntPtr Settings;
    }
    [DllImport("wintrust.dll", ExactSpelling=true, PreserveSig=true)]
    static extern int WinVerifyTrust(IntPtr window, ref Guid action, ref TrustData data);
    public static void Verify(string path) {
      var file=new FileInfo { Size=(uint)Marshal.SizeOf(typeof(FileInfo)), Path=path };
      IntPtr pointer=Marshal.AllocHGlobal(Marshal.SizeOf(typeof(FileInfo)));
      Marshal.StructureToPtr(file,pointer,false);
      var data=new TrustData { Size=(uint)Marshal.SizeOf(typeof(TrustData)), UI=2, Revocation=1,
        Choice=1, File=pointer, Action=1, Flags=0x80|0x1000|0x2000 };
      var action=new Guid("00AAC56B-CD44-11d0-8CC2-00C04FC295EE");
      try {
        int status=WinVerifyTrust(new IntPtr(-1),ref action,ref data);
        if (status!=0) throw new InvalidOperationException("WinVerifyTrust rejected artifact: 0x"+status.ToString("X8"));
      } finally {
        data.Action=2; WinVerifyTrust(new IntPtr(-1),ref action,ref data);
        Marshal.DestroyStructure(pointer,typeof(FileInfo)); Marshal.FreeHGlobal(pointer);
      }
    }
  }
}
'@
}

function Get-SigningArguments([string]$Thumbprint,[string]$Timestamp,[string]$Artifact) {
  return @('sign','/s','My','/sha1',$Thumbprint,'/fd','SHA256','/tr',$Timestamp,'/td','SHA256',$Artifact)
}

function Invoke-PinnedSignTool([string]$Tool,[string[]]$Arguments) {
  # All arguments are individually quoted; quotes/control characters are rejected
  # by the policy/path validators. No shell, certificate auto-selection or PFX.
  foreach ($argument in $Arguments) { if ($argument -match '["\x00-\x1f\x7f]' -or $argument.EndsWith('\')) { throw 'Unsafe signing argument.' } }
  $start=New-Object Diagnostics.ProcessStartInfo
  $start.FileName=$Tool; $start.UseShellExecute=$false; $start.CreateNoWindow=$true
  $start.Arguments=($Arguments | ForEach-Object { '"'+$_+'"' }) -join ' '
  $process=New-Object Diagnostics.Process
  $process.StartInfo=$start
  try {
    if (-not $process.Start()) { throw 'SignTool did not start.' }
    if (-not $process.WaitForExit(120000)) {
      $process.Kill(); $null=$process.WaitForExit(10000)
      throw 'SignTool exceeded its single bounded attempt.'
    }
    if ($process.ExitCode -ne 0) { throw "SignTool failed or returned a warning (exit $($process.ExitCode))." }
  } finally { $process.Dispose() }
}

function Assert-SigningCertificate($Certificate,[string]$Thumbprint,[string]$Publisher) {
  if ($null -eq $Certificate -or $Certificate.Thumbprint -ine $Thumbprint -or $Certificate.Subject -cne $Publisher -or
      -not $Certificate.HasPrivateKey -or $Certificate.NotBefore.ToUniversalTime() -gt [DateTime]::UtcNow -or
      $Certificate.NotAfter.ToUniversalTime() -le [DateTime]::UtcNow) { throw 'Exact usable CurrentUser signing certificate is unavailable.' }
  $eku=@($Certificate.Extensions | Where-Object { $_.Oid.Value -eq '2.5.29.37' })
  if ($eku.Count -ne 1 -or @($eku[0].EnhancedKeyUsages | Where-Object { $_.Value -eq '1.3.6.1.5.5.7.3.3' }).Count -ne 1) { throw 'Explicit Code Signing EKU is required.' }
}

function Assert-SigningResult($Signature,[string]$Thumbprint,[string]$Publisher) {
  if ($Signature.Status.ToString() -cne 'Valid' -or $Signature.SignatureType.ToString() -cne 'Authenticode' -or
      $null -eq $Signature.SignerCertificate -or $Signature.SignerCertificate.Thumbprint -ine $Thumbprint -or
      $Signature.SignerCertificate.Subject -cne $Publisher -or $null -eq $Signature.TimeStamperCertificate) {
    throw 'Signed artifact lacks the exact valid publisher and authenticated timestamp.'
  }
}

function New-SigningPrivateDirectory([string]$Path) {
  Initialize-SigningWinTrust
  [Skager.SigningTrust]::CreatePrivateDirectory($Path,[Security.Principal.WindowsIdentity]::GetCurrent().User.Value)
}

function Write-SigningReceipt([string]$Directory,$Receipt) {
  $destination=Join-Path $Directory 'signing-receipt.json'
  if (Test-Path -LiteralPath $destination) { throw 'Signing receipt already exists.' }
  $temporary=Join-Path $Directory ('receipt-'+[guid]::NewGuid().ToString('N')+'.tmp')
  try {
    $bytes=[Text.Encoding]::UTF8.GetBytes(($Receipt|ConvertTo-Json -Depth 6))
    $stream=[IO.File]::Open($temporary,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
    try { $stream.Write($bytes,0,$bytes.Length); $stream.Flush($true) } finally { $stream.Dispose() }
    [IO.File]::Move($temporary,$destination)
  } finally { if (Test-Path -LiteralPath $temporary) { Remove-Item -LiteralPath $temporary } }
}

function Invoke-ReleaseArtifactSigning([string]$Mode,[string]$InputFile,[string]$InputHash,[string]$OutputDirectory,[string]$Thumbprint,[string]$Publisher,[string]$Tool,[string]$ToolHash,[string]$Timestamp) {
  if ($Mode -cnotin @('Prepare','Sign')) { throw 'Unknown signing mode.' }
  Assert-SigningPolicy $InputHash $ToolHash $Thumbprint $Publisher $Timestamp
  if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw 'Native Windows is required; no signing emulation is accepted.' }
  $inputPath=Assert-SigningPath $InputFile
  $toolPath=Assert-SigningPath $Tool
  $outputPath=Assert-SigningPath $OutputDirectory -NewDirectory
  if ([IO.Path]::GetFileName($toolPath) -ine 'signtool.exe' -or [IO.Path]::GetExtension($inputPath) -notin @('.exe','.dll')) { throw 'Only explicit SignTool and PE EXE/DLL inputs are supported.' }
  $inputStream=$null; $toolStream=$null; $signedStream=$null; $store=$null; $created=$false
  $receipt=[ordered]@{schema=1;outcome='failed';mode=$Mode;inputSha256=$InputHash;outputSha256=$null;artifact=[IO.Path]::GetFileName($inputPath);certificateThumbprint=$Thumbprint.ToUpperInvariant();publisherSubject=$Publisher;signToolSha256=$ToolHash;timestampUrl=$Timestamp;fileDigest='SHA256';timestampDigest='SHA256';timestampSignerThumbprint=$null;utc=[DateTime]::UtcNow.ToString('o')}
  try {
    $inputStream=[IO.File]::Open($inputPath,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
    $toolStream=[IO.File]::Open($toolPath,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
    Assert-SigningPE $inputStream
    if ($toolStream.Length -lt 1 -or $toolStream.Length -gt 67108864) { throw 'SignTool exceeds its size bound.' }
    if ((Get-SigningStreamHash $inputStream) -cne $InputHash -or (Get-SigningStreamHash $toolStream) -cne $ToolHash) { throw 'Input or SignTool does not match its explicit hash pin.' }
    if ((Get-AuthenticodeSignature -LiteralPath $inputPath).Status.ToString() -cne 'NotSigned') { throw 'Input must be unsigned; appending or replacing signatures is forbidden.' }
    $toolSignature=Get-AuthenticodeSignature -LiteralPath $toolPath
    if ($toolSignature.Status.ToString() -cne 'Valid' -or $toolSignature.SignatureType.ToString() -cne 'Authenticode' -or $null -eq $toolSignature.SignerCertificate -or
        $toolSignature.SignerCertificate.GetNameInfo([Security.Cryptography.X509Certificates.X509NameType]::SimpleName,$false) -cne 'Microsoft Corporation') { throw 'Pinned SignTool must have a valid Microsoft signature.' }
    Initialize-SigningWinTrust
    [Skager.SigningTrust]::Verify($toolPath)
    $store=New-Object Security.Cryptography.X509Certificates.X509Store('My','CurrentUser')
    $store.Open([Security.Cryptography.X509Certificates.OpenFlags]::ReadOnly)
    $certificates=@($store.Certificates | Where-Object { $_.Thumbprint -ieq $Thumbprint })
    if ($certificates.Count -ne 1) { throw 'Exact certificate selection is ambiguous or absent.' }
    Assert-SigningCertificate $certificates[0] $Thumbprint $Publisher
    New-SigningPrivateDirectory $outputPath; $created=$true
    $artifact=Join-Path $outputPath $receipt.artifact
    $copy=[IO.File]::Open($artifact,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
    try { $inputStream.Position=0; $inputStream.CopyTo($copy); $copy.Flush($true) } finally { $copy.Dispose() }
    if ($Mode -ceq 'Sign') {
      Invoke-PinnedSignTool $toolPath (Get-SigningArguments $Thumbprint $Timestamp $artifact)
    }
    $signedStream=[IO.File]::Open($artifact,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
    $receipt.outputSha256=Get-SigningStreamHash $signedStream
    if ($Mode -ceq 'Sign') {
      Invoke-PinnedSignTool $toolPath @('verify','/pa','/all','/tw',$artifact)
      $signature=Get-AuthenticodeSignature -LiteralPath $artifact
      Assert-SigningResult $signature $Thumbprint $Publisher
      [Skager.SigningTrust]::Verify($artifact)
      $receipt.timestampSignerThumbprint=$signature.TimeStamperCertificate.Thumbprint
      $receipt.outcome='signed'
    } else {
      if ($receipt.outputSha256 -cne $InputHash) { throw 'Prepared copy differs from unsigned input.' }
      $receipt.outcome='prepared'
    }
    Write-SigningReceipt $outputPath $receipt
    return [pscustomobject]$receipt
  } catch {
    if ($created) {
      $receipt.outcome='failed'; $receipt.outputSha256=$null
      try { Write-SigningReceipt $outputPath $receipt } catch { }
    }
    throw
  } finally {
    if ($signedStream) { $signedStream.Dispose() }; if ($store) { $store.Close() }
    if ($toolStream) { $toolStream.Dispose() }; if ($inputStream) { $inputStream.Dispose() }
  }
}

if ($MyInvocation.InvocationName -ne '.') {
  Invoke-ReleaseArtifactSigning $SigningMode $SigningInput $SigningInputHash $SigningOutput $SigningCertificate $SigningPublisher $SigningTool $SigningToolHash $SigningTimestamp
}
