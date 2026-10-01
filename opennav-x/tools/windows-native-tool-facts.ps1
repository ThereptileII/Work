param(
    [Parameter(Mandatory=$true)][ValidateSet('Capture','Verify')][string]$Mode,
    [Parameter(Mandatory=$true)][ValidateSet('openssl-parent','openssl-child','zlib-child','zlib-parent','curl-parent')][string]$Kind,
    [Parameter(Mandatory=$true)][string]$Output,
    [Parameter(Mandatory=$true)][string]$ProducerScript,
    [Parameter(Mandatory=$true)][string]$Vswhere,
    [Parameter(Mandatory=$true)][string]$VisualStudio,
    [string]$VcVars,
    [string]$Dumpbin,
    [string]$CMakeCache,
    [string]$CMakeHookFacts
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Sha([string]$Path) {
    $Stream=[IO.File]::Open($Path,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
    $Hasher=[Security.Cryptography.SHA256]::Create()
    try { ([BitConverter]::ToString($Hasher.ComputeHash($Stream))).Replace('-','').ToLowerInvariant() }
    finally { $Hasher.Dispose(); $Stream.Dispose() }
}
function RequiredFile([string]$Path,[string]$Label) {
    if (-not $Path -or -not (Test-Path -LiteralPath $Path -PathType Leaf)) { throw "$Label missing" }
    (Resolve-Path -LiteralPath $Path).Path
}
function FileFact([string]$Path,[string]$Label) {
    $Resolved = RequiredFile $Path $Label
    $Info = (Get-Item -LiteralPath $Resolved).VersionInfo
    [ordered]@{
        path=$Resolved
        sha256=Sha $Resolved
        bytes=(Get-Item -LiteralPath $Resolved).Length
        fileVersion=[string]$Info.FileVersion
        productVersion=[string]$Info.ProductVersion
    }
}
function TextSha([string]$Value) {
    $Data=[Text.Encoding]::UTF8.GetBytes($Value)
    $Hasher=[Security.Cryptography.SHA256]::Create()
    try { ([BitConverter]::ToString($Hasher.ComputeHash($Data))).Replace('-','').ToLowerInvariant() }
    finally { $Hasher.Dispose() }
}
function ResolvedTool([string]$Name) {
    $Command=Get-Command $Name -CommandType Application -ErrorAction Stop | Select-Object -First 1
    if (-not $Command) { throw "Native tool missing: $Name" }
    $Command.Source
}
function ProbeVersion([string]$Path,[string[]]$Arguments) {
    # Native version/help switches may return nonzero (notably cl /Bv and
    # nmake /?). Record that code, not a fabricated success state.
    $Previous=$ErrorActionPreference
    $ErrorActionPreference='Continue'
    try {
        $OutputText=(& $Path @Arguments 2>&1 | Out-String)
        $Code=$LASTEXITCODE
    } finally { $ErrorActionPreference=$Previous }
    if ($null -eq $Code) { throw 'Native version probe returned no exit status' }
    if ($OutputText.Length -gt 1048576) { throw 'Native version output exceeds 1 MiB' }
    $VersionLine=@($OutputText -split '\r?\n' | ForEach-Object { $_.Trim() } | Where-Object { $_ }) |
        Select-Object -First 1
    if($VersionLine -and $VersionLine.Length -gt 512){$VersionLine=$VersionLine.Substring(0,512)}
    [ordered]@{ arguments=$Arguments; exitCode=[int]$Code; versionLine=[string]$VersionLine
        outputSha256=TextSha $OutputText.Trim() }
}
function ToolFact([string]$Name,[string[]]$Arguments,[string]$ExplicitPath='') {
    $Path=if($ExplicitPath){$ExplicitPath}else{ResolvedTool $Name}
    [ordered]@{ file=FileFact $Path $Name; version=ProbeVersion $Path $Arguments }
}
function CacheFacts([string]$Path) {
    $Path=RequiredFile $Path 'Generated CMakeCache.txt'
    $Allowed=@('CMAKE_GENERATOR','CMAKE_GENERATOR_INSTANCE','CMAKE_GENERATOR_PLATFORM',
        'CMAKE_GENERATOR_TOOLSET','CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION',
        'CMAKE_VS_PLATFORM_TOOLSET','CMAKE_MSVC_RUNTIME_LIBRARY','CMAKE_INSTALL_PREFIX',
        'CMAKE_C_COMPILER','CMAKE_CXX_COMPILER','CMAKE_MAKE_PROGRAM')
    $Values=[ordered]@{}
    foreach($Name in $Allowed){$Values[$Name]=$null}
    foreach($Line in [IO.File]::ReadAllLines($Path)) {
        if($Line -notmatch '^([^/:=]+):[^=]*=(.*)$'){continue}
        $Name=$Matches[1]
        if($Allowed -cnotcontains $Name){continue}
        if($null -ne $Values[$Name]){throw "Duplicate CMake cache key: $Name"}
        $Values[$Name]=$Matches[2]
    }
    if($Values['CMAKE_GENERATOR'] -cne 'Visual Studio 17 2022' -or
       $Values['CMAKE_GENERATOR_PLATFORM'] -cne 'Win32' -or
       -not $Values['CMAKE_GENERATOR_INSTANCE']) {
        throw 'Generated CMake cache lacks the reviewed Win32 generator instance'
    }
    $CompilerFacts=[ordered]@{}
    $Build=Split-Path $Path -Parent
    foreach($Language in @('C','CXX')) {
        $Files=@(Get-ChildItem -LiteralPath (Join-Path $Build 'CMakeFiles') -Recurse -File -Filter "CMake$($Language)Compiler.cmake" -ErrorAction Stop)
        if($Files.Count -gt 1){throw "Ambiguous CMake $Language compiler metadata"}
        if($Files.Count -eq 0){continue}
        $Text=[IO.File]::ReadAllText($Files[0].FullName)
        $CompilerMatch=[regex]::Match($Text,('(?m)^set\(CMAKE_{0}_COMPILER "([^"]+)"\)' -f $Language))
        if(-not $CompilerMatch.Success){throw "CMake $Language compiler path missing"}
        $CompilerFacts[$Language]=[ordered]@{
            metadata=FileFact $Files[0].FullName "CMake $Language compiler metadata"
            executable=FileFact $CompilerMatch.Groups[1].Value "CMake $Language compiler"
        }
    }
    if(-not $CompilerFacts.Contains('C')){throw 'CMake C compiler metadata missing'}
    $HookFile=RequiredFile $CMakeHookFacts 'Generated CMake tool facts'
    $HookValues=[ordered]@{}
    foreach($Line in [IO.File]::ReadAllLines($HookFile)) {
        if($Line -notmatch '^([A-Z_]+)=(.*)$'){throw 'Malformed generated CMake tool-fact line'}
        if($HookValues.Contains($Matches[1])){throw 'Duplicate generated CMake tool-fact key'}
        $HookValues[$Matches[1]]=$Matches[2]
    }
    $RequiredHookKeys=@('CMAKE_GENERATOR','CMAKE_GENERATOR_INSTANCE','CMAKE_GENERATOR_PLATFORM',
        'CMAKE_VS_MSBUILD_COMMAND','CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION',
        'CMAKE_VS_PLATFORM_TOOLSET','CMAKE_LINKER','CMAKE_C_COMPILER','CMAKE_MAKE_PROGRAM')
    if(@(Compare-Object $RequiredHookKeys @($HookValues.Keys)).Count){throw 'Generated CMake tool-fact keys differ'}
    if($HookValues['CMAKE_GENERATOR'] -cne 'Visual Studio 17 2022' -or
       $HookValues['CMAKE_GENERATOR_PLATFORM'] -cne 'Win32' -or
       $HookValues['CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION'] -notmatch '^10\.0\.\d+\.\d+$' -or
       $HookValues['CMAKE_VS_PLATFORM_TOOLSET'] -cne 'v143') {
        throw 'Generated CMake tool facts lack exact Win32 SDK/toolset identity'
    }
    $ProjectName=if($Kind -eq 'zlib-child'){'zlib.vcxproj'}else{'libcurl_shared.vcxproj'}
    $Projects=@(Get-ChildItem -LiteralPath $Build -Recurse -File -Filter $ProjectName)
    if($Projects.Count -ne 1){throw "Generated project identity missing or ambiguous: $ProjectName"}
    [xml]$ProjectXml=[IO.File]::ReadAllText($Projects[0].FullName)
    $SdkValues=@($ProjectXml.SelectNodes('//*[local-name()="WindowsTargetPlatformVersion"]') |
        ForEach-Object { $_.InnerText } | Where-Object { $_ } | Select-Object -Unique)
    $Toolsets=@($ProjectXml.SelectNodes('//*[local-name()="PlatformToolset"]') |
        ForEach-Object { $_.InnerText } | Where-Object { $_ } | Select-Object -Unique)
    if($SdkValues.Count -ne 1 -or $Toolsets.Count -ne 1 -or
       $SdkValues[0] -cne $HookValues['CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION'] -or
       $Toolsets[0] -cne $HookValues['CMAKE_VS_PLATFORM_TOOLSET']) {
        throw 'Generated project lacks one exact Windows SDK and v143 platform toolset'
    }
    $VsInstance=(Resolve-Path -LiteralPath $Values['CMAKE_GENERATOR_INSTANCE']).Path
    if($VsInstance -cne $VisualStudio -or
       (Resolve-Path -LiteralPath $HookValues['CMAKE_GENERATOR_INSTANCE']).Path -cne $VsInstance){
       throw 'CMake generator instance differs from producer Visual Studio'
    }
    $CompilerPath=$CompilerFacts['C'].executable.path
    if((Resolve-Path -LiteralPath $HookValues['CMAKE_C_COMPILER']).Path -cne $CompilerPath){
        throw 'Generated CMake compiler identity differs from compiler metadata'
    }
    $LinkerMatch=[regex]::Match([IO.File]::ReadAllText($CompilerFacts['C'].metadata.path),
        '(?m)^set\(CMAKE_LINKER "([^"]+)"\)')
    if(-not $LinkerMatch.Success -or -not $LinkerMatch.Groups[1].Value){
        throw 'Generated CMake linker identity missing'
    }
    $LinkPath=$LinkerMatch.Groups[1].Value
    if((Resolve-Path -LiteralPath $HookValues['CMAKE_LINKER']).Path -cne
       (Resolve-Path -LiteralPath $LinkPath).Path){throw 'CMake linker facts disagree'}
    $MsbuildPath=$HookValues['CMAKE_VS_MSBUILD_COMMAND']
    $MakePath=$HookValues['CMAKE_MAKE_PROGRAM']
    $BuilderSource=if($MakePath){'CMAKE_MAKE_PROGRAM'}else{'CMAKE_VS_MSBUILD_COMMAND'}
    if(-not $MakePath){$MakePath=$MsbuildPath}
    if(-not (Test-Path -LiteralPath $MakePath -PathType Leaf)){$MakePath=ResolvedTool $MakePath}
    if((Resolve-Path -LiteralPath $MakePath).Path -cne
       (Resolve-Path -LiteralPath $MsbuildPath).Path){throw 'CMake build program differs from selected MSBuild command'}
    if($Values['CMAKE_MAKE_PROGRAM']) {
        $CacheMake=$Values['CMAKE_MAKE_PROGRAM']
        if(-not (Test-Path -LiteralPath $CacheMake -PathType Leaf)){$CacheMake=ResolvedTool $CacheMake}
        if((Resolve-Path -LiteralPath $CacheMake).Path -cne
           (Resolve-Path -LiteralPath $MakePath).Path){throw 'CMake cache build program differs'}
    }
    [ordered]@{
        file=FileFact $Path 'CMake cache'; selected=$Values; compilers=$CompilerFacts
        generatedProject=FileFact $Projects[0].FullName 'Generated MSBuild project'
        hookFile=FileFact $HookFile 'Generated CMake tool facts'; hook=$HookValues
        hookInput=FileFact (Join-Path $PSScriptRoot 'windows-native-tool-facts.cmake') 'CMake tool-facts include'
        windowsSdkVersion=$SdkValues[0]; platformToolset=$Toolsets[0]
        linker=FileFact $LinkPath 'MSVC linker'; msbuild=FileFact $MsbuildPath 'Visual Studio MSBuild'
        builderSource=$BuilderSource
    }
}

if([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT){throw 'Windows native tool facts require Windows'}
$VswhereFact=FileFact $Vswhere 'Visual Studio locator'
$ProducerFact=FileFact $ProducerScript 'Dependency producer script'
$HelperFact=FileFact $PSCommandPath 'Tool-facts helper'
$VisualStudio=(Resolve-Path -LiteralPath $VisualStudio).Path
$ExpectedVs=& $Vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if($LASTEXITCODE -ne 0 -or @($ExpectedVs).Count -ne 1 -or
   (Resolve-Path -LiteralPath $ExpectedVs).Path -cne $VisualStudio){
    throw 'Visual Studio selection differs from the producer'
}
$VcVarsFact=$null
if($Kind -like '*-child') {
    $VcVarsFact=FileFact $VcVars 'vcvarsall x86 initializer'
    if($env:VSCMD_ARG_TGT_ARCH -cne 'x86'){throw 'Child process is not initialized for x86 MSVC'}
}
$ToolSpecs=switch($Kind) {
    'openssl-parent' { [ordered]@{'tar.exe'=@('--version');'perl.exe'=@('-v');'nasm.exe'=@('-v');'cmd.exe'=@('/c','ver')} }
    'openssl-child' { [ordered]@{'cl.exe'=@('/Bv');'link.exe'=@('/?');'nmake.exe'=@('/?');'perl.exe'=@('-v');'nasm.exe'=@('-v');'cmd.exe'=@('/c','ver')} }
    'zlib-child' { [ordered]@{'cl.exe'=@('/Bv');'cmake.exe'=@('--version');'ctest.exe'=@('--version');'tar.exe'=@('--version');'cmd.exe'=@('/c','ver')} }
    'zlib-parent' { [ordered]@{'cmake.exe'=@('--version');'tar.exe'=@('--version')} }
    'curl-parent' { [ordered]@{'cmake.exe'=@('--version');'perl.exe'=@('-v')} }
}
$Tools=[ordered]@{}
foreach($Name in $ToolSpecs.Keys){$Tools[$Name]=ToolFact $Name $ToolSpecs[$Name]}
if($Kind -eq 'zlib-parent' -or $Kind -eq 'curl-parent'){$Tools['dumpbin.exe']=ToolFact 'dumpbin.exe' @('/?') $Dumpbin}
$Environment=[ordered]@{}
foreach($Name in @('PROCESSOR_ARCHITECTURE','VSCMD_ARG_TGT_ARCH','VSCMD_ARG_HOST_ARCH',
    'VCToolsVersion','WindowsSDKVersion','UCRTVersion','VSINSTALLDIR','VCINSTALLDIR',
    'VCToolsInstallDir','WindowsSdkDir','UniversalCRTSdkDir','VisualStudioVersion',
    'CMAKE_WINDOWS_KITS_10_DIR')) {
    $Environment[$Name]=[Environment]::GetEnvironmentVariable($Name)
}
foreach($Name in @('PATH','INCLUDE','LIB','LIBPATH')) {
    $Value=[Environment]::GetEnvironmentVariable($Name)
    $Environment["$($Name)Sha256"]=if($null -eq $Value){$null}else{TextSha $Value}
}
$Facts=[ordered]@{
    schemaVersion=1; kind=$Kind; producer=$ProducerFact; helper=$HelperFact
    powerShell=[ordered]@{
        file=FileFact ([Diagnostics.Process]::GetCurrentProcess().MainModule.FileName) 'PowerShell interpreter'
        version=$PSVersionTable.PSVersion.ToString()
        edition=[string]$PSVersionTable.PSEdition
    }
    vswhere=$VswhereFact; visualStudio=$VisualStudio; vcvarsall=$VcVarsFact
    tools=$Tools; environment=$Environment
    cmake=if($Kind -eq 'zlib-child' -or $Kind -eq 'curl-parent'){CacheFacts $CMakeCache}else{$null}
}
$Json=$Facts | ConvertTo-Json -Depth 16 -Compress
if($Mode -eq 'Capture') {
    [IO.File]::WriteAllText($Output,$Json,(New-Object Text.UTF8Encoding($false)))
} else {
    $Existing=RequiredFile $Output 'Captured native tool facts'
    if([IO.File]::ReadAllText($Existing) -cne $Json){throw "Native tool facts changed: $Kind"}
}
Write-Output "Native tool facts $($Mode.ToLowerInvariant())d: $Kind"
