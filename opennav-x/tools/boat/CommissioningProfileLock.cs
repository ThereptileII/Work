// Shared exclusion for cooperating commissioning tools; not loader-tree custody.
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Runtime.InteropServices;
using System.Security.AccessControl;
using System.Security.Principal;
using System.Text;
using Microsoft.Win32.SafeHandles;

namespace OpenNavX {
  public sealed class CommissioningProfileLock : IDisposable {
    [StructLayout(LayoutKind.Sequential)] struct Info {
      public uint attributes; public System.Runtime.InteropServices.ComTypes.FILETIME created, accessed, written;
      public uint volume, sizeHigh, sizeLow, links, indexHigh, indexLow;
    }
    [StructLayout(LayoutKind.Sequential)] struct Attributes {
      public int length; public IntPtr descriptor; public int inherit;
    }
    [StructLayout(LayoutKind.Sequential)] struct FileIdentity {
      public ulong volume, low, high;
    }
    [DllImport("kernel32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    static extern SafeFileHandle CreateFileW(string path, uint access, uint share, IntPtr security, uint creation, uint flags, IntPtr template);
    [DllImport("kernel32.dll", SetLastError=true)] static extern bool GetFileInformationByHandle(SafeFileHandle file, out Info info);
    [DllImport("kernel32.dll", SetLastError=true)] static extern bool GetFileInformationByHandleEx(SafeFileHandle file, int kind, out FileIdentity identity, uint size);
    [DllImport("kernel32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    static extern uint GetFinalPathNameByHandleW(SafeFileHandle file, StringBuilder path, uint length, uint flags);
    [DllImport("advapi32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    static extern bool ConvertStringSecurityDescriptorToSecurityDescriptorW(string text, uint revision, out IntPtr descriptor, out uint size);
    [DllImport("kernel32.dll")] static extern IntPtr LocalFree(IntPtr pointer);
    [DllImport("kernel32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    static extern SafeWaitHandle CreateMutexExW(ref Attributes attributes, string name, uint flags, uint access);
    [DllImport("advapi32.dll", SetLastError=true)]
    static extern bool GetKernelObjectSecurity(SafeWaitHandle handle, uint information, byte[] descriptor, uint length, out uint needed);
    [DllImport("kernel32.dll", SetLastError=true)] static extern uint WaitForSingleObject(SafeWaitHandle handle, uint milliseconds);
    [DllImport("kernel32.dll", SetLastError=true)] static extern bool ReleaseMutex(SafeWaitHandle handle);
    [DllImport("kernel32.dll")] static extern uint GetCurrentThreadId();

    static readonly object gate = new object();
    static readonly HashSet<string> active = new HashSet<string>(StringComparer.Ordinal);
    readonly List<SafeFileHandle> directories = new List<SafeFileHandle>();
    SafeWaitHandle mutex;
    string name;
    uint thread;
    bool registered, owned, disposed;
    CommissioningProfileLock() {}
    public string Identity { get { if (disposed) throw new ObjectDisposedException("CommissioningProfileLock"); return name; } }

    static Exception Error(string operation) { return new Win32Exception(Marshal.GetLastWin32Error(), operation); }
    static string NativePath(SafeFileHandle handle) {
      var value = new StringBuilder(32768);
      uint size = GetFinalPathNameByHandleW(handle, value, (uint)value.Capacity, 2); // VOLUME_NAME_NT
      if (size == 0 || size >= value.Capacity) throw Error("Profile directory identity unavailable");
      string result = value.ToString().TrimEnd('\\');
      if (!result.StartsWith("\\Device\\HarddiskVolume", StringComparison.OrdinalIgnoreCase))
        throw new IOException("Only a local physical disk profile is supported");
      return result;
    }
    Info Pin(string path, string expected) {
      var handle = CreateFileW(path, 0x80000000, 3, IntPtr.Zero, 3, 0x02200000, IntPtr.Zero);
      if (handle.IsInvalid) { handle.Dispose(); throw Error("Cannot retain profile directory"); }
      directories.Add(handle);
      Info info;
      if (!GetFileInformationByHandle(handle, out info)) throw Error("Cannot inspect profile directory");
      if ((info.attributes & 0x10) == 0 || (info.attributes & 0x400) != 0)
        throw new IOException("Profile ancestry must be plain directories");
      if (expected != null && !String.Equals(NativePath(handle), expected, StringComparison.OrdinalIgnoreCase))
        throw new IOException("Profile directory physical identity changed");
      return info;
    }
    static void AssertSecurity(SafeWaitHandle handle, string sid) {
      uint size;
      GetKernelObjectSecurity(handle, 5, null, 0, out size); // OWNER | DACL
      if (size == 0 || size > 65536) throw new IOException("Profile lock security unavailable");
      var bytes = new byte[size];
      if (!GetKernelObjectSecurity(handle, 5, bytes, size, out size)) throw Error("Cannot inspect profile lock security");
      var security = new RawSecurityDescriptor(bytes, 0);
      if (security.Owner == null || security.Owner.Value != sid ||
          (security.ControlFlags & ControlFlags.DiscretionaryAclProtected) == 0 ||
          security.DiscretionaryAcl == null || security.DiscretionaryAcl.Count != 2)
        throw new IOException("Profile lock ownership or ACL differs");
      var identities = new HashSet<string>(StringComparer.Ordinal);
      foreach (GenericAce entry in security.DiscretionaryAcl) {
        var ace = entry as CommonAce;
        if (ace == null || ace.AceFlags != AceFlags.None || ace.AceQualifier != AceQualifier.AccessAllowed ||
            ace.AccessMask != 0x1f0001 || !identities.Add(ace.SecurityIdentifier.Value))
          throw new IOException("Profile lock grants unexpected access");
      }
      if (!identities.Contains(sid) || !identities.Contains("S-1-5-18"))
        throw new IOException("Profile lock owner/SYSTEM grants required");
    }
    public static CommissioningProfileLock Acquire(string profileDirectory) {
      if (Environment.OSVersion.Platform != PlatformID.Win32NT)
        throw new PlatformNotSupportedException("Native Windows profile exclusion required");
      var owner = new CommissioningProfileLock();
      try {
        if (String.IsNullOrEmpty(profileDirectory) || profileDirectory.Length > 32700 ||
            profileDirectory.Length < 3 || !Char.IsLetter(profileDirectory[0]) ||
            profileDirectory[1] != ':' || profileDirectory[2] != '\\' || profileDirectory.IndexOf('/') >= 0 ||
            profileDirectory.IndexOf(':', 2) >= 0 || profileDirectory.IndexOf('\0') >= 0)
          throw new IOException("An absolute local DOS profile directory is required");
        string path = profileDirectory.TrimEnd('\\');
        if (path.Length < 3 || !String.Equals(Path.GetFullPath(path), path, StringComparison.OrdinalIgnoreCase))
          throw new IOException("Canonical profile directory required");
        string[] parts = path.Substring(3).Split('\\');
        foreach (string part in parts)
          if (part.Length == 0 || part == "." || part == ".." || part.EndsWith(".") || part.EndsWith(" "))
            throw new IOException("Ambiguous profile directory component");
        owner.Pin(path.Substring(0, 3), null);
        string physical = NativePath(owner.directories[0]);
        foreach (string part in parts) {
          physical += "\\" + part;
          owner.Pin("\\\\?\\GLOBALROOT" + physical, physical);
        }
        FileIdentity identity;
        if (!GetFileInformationByHandleEx(owner.directories[owner.directories.Count-1], 18, out identity, 24))
          throw Error("Full physical profile file identity unavailable");
        // The replaceable opencpn.ini file, workspace and DOS alias are not keys.
        owner.name = "Global\\SKAGER.CommissioningProfile.1." + identity.volume.ToString("x16") +
          "." + identity.high.ToString("x16") + identity.low.ToString("x16") + ".opencpn.ini";
        lock (gate) {
          if (!active.Add(owner.name)) throw new IOException("Profile commissioning already active in this process");
          owner.registered = true;
        }
        string sid = WindowsIdentity.GetCurrent().User.Value;
        IntPtr descriptor;
        uint size;
        if (!ConvertStringSecurityDescriptorToSecurityDescriptorW("O:" + sid + "D:P(A;;0x1f0001;;;" + sid + ")(A;;0x1f0001;;;SY)", 1, out descriptor, out size))
          throw Error("Cannot construct profile lock security");
        try {
          var attributes = new Attributes { length=Marshal.SizeOf(typeof(Attributes)), descriptor=descriptor, inherit=0 };
          owner.mutex = CreateMutexExW(ref attributes, owner.name, 0, 0x1f0001);
          if (owner.mutex.IsInvalid) throw Error("Cannot open machine-wide profile exclusion");
        } finally { LocalFree(descriptor); }
        AssertSecurity(owner.mutex, sid);
        uint wait = WaitForSingleObject(owner.mutex, 0);
        if (wait == 0 || wait == 0x80) { owner.owned = true; owner.thread = GetCurrentThreadId(); }
        if (wait == 0x80) throw new IOException("Abandoned profile commissioning acquisition refused; inspect durable commissioning state");
        if (wait == 0x102) throw new IOException("Another commissioning operation owns this physical profile");
        if (wait != 0) throw Error("Profile exclusion acquisition failed");
        return owner;
      } catch { owner.Dispose(); throw; }
    }
    public void Dispose() {
      if (disposed) return;
      if (owned) {
        if (thread != GetCurrentThreadId()) throw new IOException("Profile exclusion must be released by its owning thread");
        if (!ReleaseMutex(mutex)) throw Error("Profile exclusion release failed");
        owned = false;
      }
      if (mutex != null) mutex.Dispose();
      if (registered) { lock (gate) { active.Remove(name); } registered = false; }
      for (int i=directories.Count-1; i>=0; --i) directories[i].Dispose();
      disposed = true;
    }
  }
}
