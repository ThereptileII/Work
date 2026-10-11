// Inert native-Windows test image only. No installed app, profile or equipment.
using System;
using System.IO;
using System.IO.Pipes;
using System.Text;
using System.Threading;
using System.Windows.Forms;
internal static class UpdateImageMarker {
 const string FixtureImage="A";
 [STAThread] static int Main() {
  try {
   string root=Environment.GetEnvironmentVariable("SKAGER_IMAGE_FIXTURE_ROOT");
   string label=FixtureImage;
   if(String.IsNullOrEmpty(root) || (label!="A" && label!="B")) return 125;
   // Process.CloseMainWindow discovers visible, unowned top-level windows.
   // Keep this inert fixture discoverable so normal owner cleanup is tested.
   var form=new Form { ShowInTaskbar=true,Opacity=1,Width=1,Height=1,Text="Inert update image fixture" };
   form.Shown+=delegate {
    // The child only writes to its parent's unique disposable fixture root.
    File.WriteAllText(Path.Combine(root,"marker-"+label),"started");
    string pipe=Environment.GetEnvironmentVariable("SKAGER_UPDATE_PIPE");
    if(!String.IsNullOrEmpty(pipe)) {
     Thread.Sleep(250);
     using(var client=new NamedPipeClientStream(".",pipe,PipeDirection.Out)) {
      client.Connect(5000);
      string frame="SKAGER-UPDATE-READY/1 "+Environment.GetEnvironmentVariable("SKAGER_UPDATE_GENERATION")+" "+Environment.GetEnvironmentVariable("SKAGER_IMAGE_FIXTURE_COMMIT")+" "+Environment.GetEnvironmentVariable("SKAGER_UPDATE_CHALLENGE")+"\n";
      byte[] bytes=Encoding.ASCII.GetBytes(frame);client.Write(bytes,0,bytes.Length);client.Flush();
     }
    }
   };
   Application.Run(form);return 0;
  } catch { return 124; }
 }
}
