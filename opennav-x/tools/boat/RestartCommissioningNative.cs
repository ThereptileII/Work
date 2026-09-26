using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.IO.Pipes;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.RegularExpressions;
using Microsoft.Win32.SafeHandles;

namespace OpenNavX {
  public static class RestartCommissioningNative {
    const int Maximum=65536;
    static readonly UTF8Encoding Utf8=new UTF8Encoding(false,true);
    [StructLayout(LayoutKind.Sequential)] struct SecurityAttributes {public int Length;public IntPtr Descriptor;[MarshalAs(UnmanagedType.Bool)]public bool Inherit;}
    [DllImport("advapi32.dll",CharSet=CharSet.Unicode,SetLastError=true)] static extern bool ConvertStringSecurityDescriptorToSecurityDescriptorW(string sddl,uint revision,out IntPtr descriptor,out uint bytes);
    [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] static extern SafePipeHandle CreateNamedPipeW(string name,uint openMode,uint pipeMode,uint instances,uint output,uint input,uint timeout,ref SecurityAttributes security);
    [DllImport("kernel32.dll")] static extern IntPtr LocalFree(IntPtr value);
    [DllImport("kernel32.dll",SetLastError=true)] static extern bool GetNamedPipeClientProcessId(SafePipeHandle pipe,out uint pid);
    public static NamedPipeServerStream NewPipe(string session,string sid) {
      if(!Regex.IsMatch(session,"^[a-f0-9]{64}$") || !Regex.IsMatch(sid,"^S-1-5-[0-9-]+$"))throw new ArgumentException("Invalid local pipe identity.");
      IntPtr descriptor;uint size;
      if(!ConvertStringSecurityDescriptorToSecurityDescriptorW("D:P(A;;GA;;;"+sid+")(A;;GA;;;SY)(A;;GA;;;BA)",1,out descriptor,out size))throw new Win32Exception();
      try {
        var sa=new SecurityAttributes{Length=Marshal.SizeOf(typeof(SecurityAttributes)),Descriptor=descriptor,Inherit=false};
        // Duplex, FIRST_PIPE_INSTANCE, overlapped; byte mode and remote clients
        // rejected. A pre-existing same-name server causes failure, never reuse.
        var handle=CreateNamedPipeW(@"\\.\pipe\OpenNavX-CommissioningRestart-"+session,0x40080003,8,1,Maximum,Maximum,0,ref sa);
        if(handle.IsInvalid){handle.Dispose();throw new Win32Exception();}
        try{return new NamedPipeServerStream(PipeDirection.InOut,true,false,handle);}catch{handle.Dispose();throw;}
      } finally {LocalFree(descriptor);}
    }
    static int Remaining(DateTime deadline) {var ms=(deadline-DateTime.UtcNow).TotalMilliseconds;if(ms<=0)throw new TimeoutException("Restart audit deadline expired.");return (int)Math.Min(ms,Int32.MaxValue);}
    public static void Connect(NamedPipeServerStream pipe,DateTime deadline) {
      Remaining(deadline);
      var result=pipe.BeginWaitForConnection(null,null);
      using(var wait=result.AsyncWaitHandle){if(!wait.WaitOne(Remaining(deadline)))throw new TimeoutException("No explicit restart connected.");}
      pipe.EndWaitForConnection(result);
    }
    public static uint ClientPid(NamedPipeServerStream pipe) {uint pid;if(!GetNamedPipeClientProcessId(pipe.SafePipeHandle,out pid)||pid==0)throw new Win32Exception();return pid;}
    static byte[] ReadExactly(Stream stream,int size,DateTime deadline) {
      var bytes=new byte[size];int offset=0;
      while(offset<size){Remaining(deadline);var result=stream.BeginRead(bytes,offset,size-offset,null,null);using(var wait=result.AsyncWaitHandle){if(!wait.WaitOne(Remaining(deadline)))throw new TimeoutException("Restart peer read timed out.");}int count=stream.EndRead(result);if(count<=0)throw new EndOfStreamException();offset+=count;}
      return bytes;
    }
    public static byte[] ReadFrame(Stream stream,DateTime deadline) {
      var prefix=ReadExactly(stream,4,deadline);uint size=(uint)(prefix[0]|prefix[1]<<8|prefix[2]<<16|prefix[3]<<24);
      if(size==0||size>Maximum)throw new InvalidDataException("Restart frame outside bound.");return ReadExactly(stream,(int)size,deadline);
    }
    public static void WriteFrame(Stream stream,byte[] payload,DateTime deadline) {
      if(payload==null||payload.Length==0||payload.Length>Maximum)throw new InvalidDataException("Restart frame outside bound.");
      var bytes=new byte[payload.Length+4];int n=payload.Length;for(int i=0;i<4;i++)bytes[i]=(byte)(n>>(8*i));Buffer.BlockCopy(payload,0,bytes,4,n);
      Remaining(deadline);
      var result=stream.BeginWrite(bytes,0,bytes.Length,null,null);using(var wait=result.AsyncWaitHandle){if(!wait.WaitOne(Remaining(deadline)))throw new TimeoutException("Restart peer write timed out.");}stream.EndWrite(result);
    }
    public static byte[] Reply(string[] fields) {
      if(fields==null||(fields.Length!=2&&fields.Length!=17))throw new InvalidDataException("Fixed restart reply required.");
      using(var memory=new MemoryStream()){using(var writer=new BinaryWriter(memory,Utf8,true)){writer.Write((uint)fields.Length);foreach(var field in fields){if(field==null)throw new InvalidDataException();var bytes=Utf8.GetBytes(field);writer.Write((uint)bytes.Length);writer.Write(bytes);}}if(memory.Length>Maximum)throw new InvalidDataException();return memory.ToArray();}
    }
    // Deliberately restricted JSON object grammar. No duplicate keys, nested
    // structures, nulls, booleans, floats or trailing input. Required schemas
    // are additionally checked by the policy. Never deserialize .NET objects.
    public static Dictionary<string,object> Message(byte[] bytes) {
      if(bytes==null||bytes.Length==0||bytes.Length>Maximum)throw new InvalidDataException("Message outside bound.");
      return new Json(Utf8.GetString(bytes)).Read();
    }
    sealed class Json {
      readonly string text;int p;
      public Json(string value){text=value;}
      void White(){while(p<text.Length&&(text[p]==' '||text[p]=='\t'||text[p]=='\r'||text[p]=='\n'))p++;}
      bool Take(char c){White();if(p<text.Length&&text[p]==c){p++;return true;}return false;}
      void Need(char c){if(!Take(c))throw new InvalidDataException("Invalid restart JSON grammar.");}
      string String(){
        Need('"');var result=new StringBuilder();bool ended=false;
        while(p<text.Length){char c=text[p++];if(c=='"'){ended=true;break;}if(c<' ')throw new InvalidDataException("Control character in JSON string.");
          if(c=='\\'){if(p>=text.Length)throw new InvalidDataException();c=text[p++];switch(c){case '"':case '\\':case '/':result.Append(c);break;case 'b':result.Append('\b');break;case 'f':result.Append('\f');break;case 'n':result.Append('\n');break;case 'r':result.Append('\r');break;case 't':result.Append('\t');break;case 'u':if(p+4>text.Length)throw new InvalidDataException();ushort u;if(!UInt16.TryParse(text.Substring(p,4),NumberStyles.AllowHexSpecifier,CultureInfo.InvariantCulture,out u))throw new InvalidDataException();p+=4;result.Append((char)u);break;default:throw new InvalidDataException();}}
          else result.Append(c);
        }
        if(!ended)throw new InvalidDataException();var value=result.ToString();Utf8.GetBytes(value);return value;
      }
      object Value(){White();if(p>=text.Length)throw new InvalidDataException();if(text[p]=='"')return String();
        if(Take('[')){var array=new List<string>();if(!Take(']')){do{if(array.Count>=4)throw new InvalidDataException();array.Add(String());}while(Take(','));Need(']');}return array.ToArray();}
        // Protocol is the only JSON numeric field, always the integer 1.
        if(Take('1'))return 1;throw new InvalidDataException("Only protocol integer1, strings and bounded string arrays allowed.");
      }
      public Dictionary<string,object> Read(){Need('{');var result=new Dictionary<string,object>(StringComparer.Ordinal);if(!Take('}')){do{if(result.Count>=32)throw new InvalidDataException();var key=String();Need(':');if(result.ContainsKey(key))throw new InvalidDataException("Duplicate JSON key.");result.Add(key,Value());}while(Take(','));Need('}');}White();if(p!=text.Length)throw new InvalidDataException("Trailing JSON input.");return result;}
    }
  }
}
