using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
namespace OpenNavX {
  // Metadata comes from the exact open file supplying diagnostic bytes, never
  // a second path lookup after the publisher's atomic rename.
  public static class ManualPilotDiagnosticsNative {
    [StructLayout(LayoutKind.Sequential)] private struct FileInformation {
      public uint Attributes;
      public System.Runtime.InteropServices.ComTypes.FILETIME Creation,Access,Write;
      public uint Volume,SizeHigh,SizeLow,Links,IndexHigh,IndexLow;
    }
    public sealed class Metadata {public int Length,Attributes;public DateTime WrittenUtc;}
    [DllImport("kernel32.dll",SetLastError=true)] private static extern uint GetFileType(SafeFileHandle file);
    [DllImport("kernel32.dll",SetLastError=true)] [return:MarshalAs(UnmanagedType.Bool)]
    private static extern bool GetFileInformationByHandle(SafeFileHandle file,out FileInformation info);
    public static Metadata Inspect(SafeFileHandle file) {
      FileInformation info;
      if(file==null || file.IsInvalid || file.IsClosed || GetFileType(file)!=1 ||
         !GetFileInformationByHandle(file,out info))throw new Win32Exception(Marshal.GetLastWin32Error(),"Regular held diagnostics file unavailable.");
      // Directory/device/reparse or multiple hard links are never diagnostics.
      if((info.Attributes & (0x10u|0x40u|0x400u))!=0 || info.Links!=1 ||
         info.SizeHigh!=0 || info.SizeLow==0 || info.SizeLow>4194304)
        throw new InvalidOperationException("Held diagnostics is not a bounded ordinary single-link file.");
      ulong ticks=((ulong)(uint)info.Write.dwHighDateTime<<32)|(uint)info.Write.dwLowDateTime;
      if(ticks>Int64.MaxValue)throw new InvalidOperationException("Invalid held diagnostics timestamp.");
      return new Metadata{Length=(int)info.SizeLow,Attributes=(int)info.Attributes,WrittenUtc=DateTime.FromFileTimeUtc((long)ticks)};
    }
  }
}
