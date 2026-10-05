// Explicit commissioning helper. Secret input is framed binary stdin, never an
// argument, environment value, transcript expression, file or diagnostic field.
using System;
using System.IO;
using System.Runtime.InteropServices;

public static class XNavAisCredentialImport {
  [StructLayout(LayoutKind.Sequential, CharSet=CharSet.Unicode)]
  struct Credential {
    public uint Flags, Type;
    public string TargetName, Comment;
    public System.Runtime.InteropServices.ComTypes.FILETIME LastWritten;
    public uint CredentialBlobSize;
    public IntPtr CredentialBlob;
    public uint Persist, AttributeCount;
    public IntPtr Attributes;
    public string TargetAlias, UserName;
  }
  [DllImport("advapi32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
  static extern bool CredWriteW(ref Credential c, uint flags);
  [DllImport("advapi32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
  static extern bool CredReadW(string target, uint type, uint flags, out IntPtr value);
  [DllImport("advapi32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
  static extern bool CredDeleteW(string target, uint type, uint flags);
  [DllImport("advapi32.dll")] static extern void CredFree(IntPtr value);
  static void Zero(IntPtr p, int n) {
    for (int i=0;i<n;i++) Marshal.WriteByte(p,i,0);
  }
  // 4-byte little-endian length followed by 1..512 printable ASCII bytes and
  // EOF. A malformed or truncated stream can never replace a stored secret.
  public static byte[] ReadPayload(Stream input) {
    byte[] header=new byte[4], key=null;
    try {
      for(int i=0;i<4;i++) {int b=input.ReadByte();if(b<0)return null;header[i]=(byte)b;}
      uint n=(uint)(header[0]|header[1]<<8|header[2]<<16|header[3]<<24);
      if(n<1||n>512)return null;
      key=new byte[n];
      for(int i=0;i<key.Length;i++) {int b=input.ReadByte();if(b<33||b>126) return null;key[i]=(byte)b;}
      if(input.ReadByte()!=-1)return null;
      byte[] result=key;key=null;return result;
    } finally {Array.Clear(header,0,header.Length);if(key!=null)Array.Clear(key,0,key.Length);}
  }
  static string ReadMatch(string target, byte[] key) {
    IntPtr p;
    if(!CredReadW(target,1,0,out p)) {
      int error=Marshal.GetLastWin32Error();return error==1168?"missing":"read-failed:"+error;
    }
    try {
      Credential c=(Credential)Marshal.PtrToStructure(p,typeof(Credential));
      bool same=c.Type==1&&c.CredentialBlobSize==key.Length&&c.CredentialBlob!=IntPtr.Zero;
      if(c.CredentialBlob!=IntPtr.Zero&&c.CredentialBlobSize<=512) {
        uint difference=0;
        for(int i=0;i<c.CredentialBlobSize;i++) difference|=(uint)(Marshal.ReadByte(c.CredentialBlob,i)^(i<key.Length?key[i]:0));
        same&=difference==0;Zero(c.CredentialBlob,(int)c.CredentialBlobSize);
      } else same=false;
      return same?"same":"different";
    } finally {CredFree(p);}
  }
  static string Store(Stream input,string target) {
    byte[] key=null;IntPtr bytes=IntPtr.Zero;
    try {
      key=ReadPayload(input);if(key==null)return "invalid-input";
      string before=ReadMatch(target,key);
      if(before=="same")return "already-stored-and-verified";
      if(before=="different")return "existing-different-key-preserved";
      if(before!="missing")return before;
      bytes=Marshal.AllocHGlobal(key.Length);Marshal.Copy(key,0,bytes,key.Length);
      Credential c=new Credential();c.Type=1;c.TargetName=target;c.Persist=2;
      c.UserName="SKAGER AISStream";c.CredentialBlob=bytes;c.CredentialBlobSize=(uint)key.Length;
      if(!CredWriteW(ref c,0))return "store-failed:"+Marshal.GetLastWin32Error();
      return ReadMatch(target,key)=="same"?"stored-and-verified":"readback-failed";
    } catch {return "import-failed";}
    finally {
      if(bytes!=IntPtr.Zero){Zero(bytes,key.Length);Marshal.FreeHGlobal(bytes);}
      if(key!=null)Array.Clear(key,0,key.Length);
    }
  }
  public static string StoreProduction(Stream input) {return Store(input,"OpenNavX/AISStream/v1");}
  static string TestTarget(string id) {
    Guid value;if(!Guid.TryParseExact(id,"N",out value))throw new ArgumentException("Invalid isolated test identity");
    return "OpenNavX/Tests/AISStream/import-"+id;
  }
  public static string StoreForTest(Stream input,string id) {return Store(input,TestTarget(id));}
  public static void RemoveForTest(string id) {
    if(!CredDeleteW(TestTarget(id),1,0)&&Marshal.GetLastWin32Error()!=1168)throw new InvalidOperationException("Isolated credential cleanup failed: "+Marshal.GetLastWin32Error());
  }
}
