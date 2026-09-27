// One source-defined local o-charts CMD_EXIT request. No process termination,
// executable launch, arbitrary pipe, payload, retry, or equipment transport.
using System;
using System.Diagnostics;
using System.IO;
using System.IO.Pipes;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Threading.Tasks;
using Microsoft.Win32.SafeHandles;

namespace OpenNavX {
  public static class ChartHelperShutdownNative {
    public const string HelperSha256 = "ec27c947fc9ba4ae09345961780b885deddc2e4fcf094805a14cdf19504e0adb";
    [DllImport("kernel32.dll", SetLastError=true)]
    static extern bool GetNamedPipeServerProcessId(SafePipeHandle pipe, out uint pid);
    [DllImport("kernel32.dll", SetLastError=true)]
    static extern bool CancelIoEx(SafePipeHandle pipe, IntPtr overlapped);

    public sealed class Result {
      public int ProcessId, ParentProcessId, SessionId, ReplyBytes, ExitCode;
      public long StartedUtcTicks;
      public string PipeName, ReplyHex, Stage, Failure;
      public bool ProcessHandleRetained, PipePeerVerified, WriteAttempted,
        WriteCompleted, ExitObserved, ExitCodeKnown, Succeeded,
        IoCancellationRequested, PendingIoDrained;
      public bool ReplyMeaningValidated = false;
    }
    static byte[] ExitPacket() {
      // c98bf5f Osenc.h fifo_msg: cmd + char[256] + char[256] + char[512].
      // All-char structure has no padding. Reserved/empty fields stay zero,
      // rather than reproducing upstream's uninitialized fifo_name stack bytes.
      byte[] value = new byte[1025]; value[0] = 2; return value;
    }
    static string Hash(Stream stream) {
      stream.Position=0;
      using (SHA256 hash=SHA256.Create())
        return BitConverter.ToString(hash.ComputeHash(stream)).Replace("-", "").ToLowerInvariant();
    }
    static int Remaining(Stopwatch timer, int milliseconds) {
      long remaining=milliseconds-timer.ElapsedMilliseconds;
      if (remaining<=0) throw new TimeoutException("Bounded chart-helper operation expired; no retry.");
      return (int)remaining;
    }
    static void Wait(Task task, NamedPipeClientStream pipe, Stopwatch timer, int milliseconds, Result result) {
      try {
        if (!task.Wait(Remaining(timer,milliseconds)))
          throw new TimeoutException("Local pipe I/O timed out; delivery may be uncertain; no retry.");
        task.GetAwaiter().GetResult();
        Remaining(timer,milliseconds);
      } catch(TimeoutException) {
        // Request cancellation only for our pipe I/O, never the server process.
        // Drain at most five seconds and observe any later fault. Runtime tasks
        // retain their buffers until completion, including delayed cancellation.
        if(!pipe.SafePipeHandle.IsClosed)
          result.IoCancellationRequested=CancelIoEx(pipe.SafePipeHandle,IntPtr.Zero);
        try { result.PendingIoDrained=task.Wait(5000); }
        catch(AggregateException) { result.PendingIoDrained=task.IsCompleted; }
        task.ContinueWith(t=>{var ignored=t.Exception;},TaskContinuationOptions.OnlyOnFaulted);
        throw;
      } catch(AggregateException) { task.GetAwaiter().GetResult();throw; }
    }
    static void Check(Process process, long ticks, int session, string path) {
      process.Refresh();
      if (process.HasExited || process.StartTime.ToUniversalTime().Ticks!=ticks ||
          process.SessionId!=session || !String.Equals(process.MainModule.FileName,path,StringComparison.OrdinalIgnoreCase))
        throw new InvalidOperationException("Exact retained chart-helper process identity changed.");
    }
    // Private transport boundary is exercised with disposable marker processes
    // via reflection in tests. The public operation never accepts another hash,
    // packet, pipe name or deadline.
    static Result StopExact(int pid, long ticks, int parent, int session,
                            string path, string hash, int milliseconds) {
      Result result=new Result { ProcessId=pid, StartedUtcTicks=ticks,
        ParentProcessId=parent, SessionId=session,
        PipeName="OCPN"+(parent%10000).ToString("D4",System.Globalization.CultureInfo.InvariantCulture),
        Stage="identity" };
      Stopwatch timer=Stopwatch.StartNew();
      Process process=null;
      try {
        if(pid<=0 || parent<=0 || pid==parent || ticks<=0 || session<0 || milliseconds<100 || milliseconds>20000)
          throw new InvalidOperationException("Invalid exact chart-helper identity.");
        process=Process.GetProcessById(pid);
        IntPtr retained=process.Handle;
        if(retained==IntPtr.Zero || retained==new IntPtr(-1))
          throw new InvalidOperationException("Could not retain chart-helper handle.");
        result.ProcessHandleRetained=true;
        Check(process,ticks,session,path);
        // Deny concurrent replacement/write while identity is checked and the
        // request is dispatched. The process handle remains held through exit.
        using(FileStream image=new FileStream(path,FileMode.Open,FileAccess.Read,FileShare.Read)) {
          if(!String.Equals(Hash(image),hash,StringComparison.Ordinal))
            throw new InvalidOperationException("Chart-helper executable hash changed.");
          using(NamedPipeClientStream pipe=new NamedPipeClientStream(".",result.PipeName,
              PipeDirection.InOut,PipeOptions.Asynchronous)) {
            result.Stage="connect";
            pipe.Connect(Remaining(timer,milliseconds));
            result.Stage="peer";
            uint peer;
            if(!GetNamedPipeServerProcessId(pipe.SafePipeHandle,out peer) || peer!=(uint)pid)
              throw new InvalidOperationException("Connected pipe is not served by the exact reviewed helper PID.");
            result.PipePeerVerified=true;
            Check(process,ticks,session,path);
            // Opening a pipe is not a command. Recheck the connected instance
            // immediately before the single fixed write.
            if(!GetNamedPipeServerProcessId(pipe.SafePipeHandle,out peer) || peer!=(uint)pid)
              throw new InvalidOperationException("Pipe peer changed before CMD_EXIT.");
            Remaining(timer,milliseconds);
            result.Stage="write"; result.WriteAttempted=true;
            byte[] packet=ExitPacket();
            Wait(pipe.WriteAsync(packet,0,packet.Length),pipe,timer,milliseconds,result);
            result.WriteCompleted=true;
            result.Stage="reply";
            byte[] reply=new byte[3];
            while(result.ReplyBytes<reply.Length) {
              Remaining(timer,milliseconds);
              Task<int> read=pipe.ReadAsync(reply,result.ReplyBytes,reply.Length-result.ReplyBytes);
              Wait(read,pipe,timer,milliseconds,result);
              if(read.Result==0) throw new EndOfStreamException("Helper closed without the three-byte source-protocol reply.");
              result.ReplyBytes+=read.Result;
            }
            result.ReplyHex=BitConverter.ToString(reply).Replace("-", "");
            // Upstream reads three bytes but does not define their meaning.
            // Never fabricate an OK acknowledgement; actual exit is required.
          }
          result.Stage="exit";
          if(!process.WaitForExit(Remaining(timer,milliseconds)))
            throw new TimeoutException("Helper did not exit after CMD_EXIT; no retry or force termination.");
          result.ExitObserved=true; result.ExitCode=process.ExitCode; result.ExitCodeKnown=true;
          if(result.ExitCode!=0) throw new InvalidOperationException("Helper returned a measured nonzero exit code.");
          result.Stage="complete"; result.Succeeded=true;
        }
      } catch(Exception error) {
        result.Failure=error.GetBaseException().Message;
        // A refusal/timeout never becomes success solely because the process
        // subsequently exits, but preserve observed exit facts when available.
        if(process!=null && result.ProcessHandleRetained) {
          try { if(process.HasExited) { result.ExitObserved=true;result.ExitCode=process.ExitCode;result.ExitCodeKnown=true; } } catch { }
        }
      } finally { if(process!=null) process.Dispose(); }
      return result;
    }
    public static Result Shutdown(int pid,long startedUtcTicks,int parentPid,int sessionId,string executable) {
      if(!String.Equals(Path.GetFileName(executable),"oexserverd.exe",StringComparison.OrdinalIgnoreCase))
        throw new InvalidOperationException("Only the exact reviewed chart decoder is supported.");
      return StopExact(pid,startedUtcTicks,parentPid,sessionId,executable,HelperSha256,20000);
    }
  }
}
