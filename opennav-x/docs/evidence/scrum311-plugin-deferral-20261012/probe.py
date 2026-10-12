from pathlib import Path
import hashlib,json,subprocess
out=Path(__file__).resolve().parent
source=(out/'plugin_loader.cpp').read_text()
start=source.index('bool PluginLoader::LoadPluginCandidate(')
end=source.index('  wxDateTime plugin_modification',start)
prefix=source[start:end]
prelude=r'''
#include <string>
#include <sstream>
#include <set>
#include <iostream>
#include <stdexcept>
static bool restricted=false;
static int bookkeeping=0;
static int stamps=0;
namespace opennav::integration { bool QualificationOutputSuppressed() { return restricted; } }
struct wxString : std::string {
 using std::string::string;
 wxString(std::string s):std::string(s){}
 std::string ToStdString() const { return *this; }
 std::string mb_str() const { return *this; }
};
struct wxFileName {
 wxString s;
 explicit wxFileName(wxString v):s(v){}
 wxString GetFullName() const { auto i=s.find_last_of('/');return s.substr(i==s.npos?0:i+1); }
 wxString GetName() const { auto v=GetFullName();return v.substr(0,v.find_last_of('.')); }
};
void wxLogMessage(const char*,const char*) { ++bookkeeping; }
static std::ostringstream messages;
#define MESSAGE_LOG messages
class PluginLoader {
 std::set<std::string> loaded;
 bool IsSystemPluginPath(const std::string& s) { ++bookkeeping;return s=="system_pi.dll"; }
 bool HasLoadStamp(const std::string& s) { ++bookkeeping;return loaded.count(s); }
 void CreateLoadStamp(const std::string& s) { ++bookkeeping;++stamps;loaded.insert(s); }
 public: bool LoadPluginCandidate(const wxString&,bool);
};
'''
postlude=r'''
int main() {
 try {
  PluginLoader p;
  restricted=true;
#ifdef SKAGER_UPDATE_QUALIFICATION
  for(int i=0;i<2;++i) {
   if(p.LoadPluginCandidate("/plugins/custom_pi.dll",true) || bookkeeping || stamps)
    throw std::runtime_error("restricted discovery caused bookkeeping/load stamp");
  }
  std::cout<<"PASS restricted repeated discovery leaves no stamp or bookkeeping\n";
#endif
  restricted=false;
  if(!p.LoadPluginCandidate("/plugins/custom_pi.dll",true) || stamps!=1 || !bookkeeping)
   throw std::runtime_error("ordinary plugin candidate path changed");
  if(p.LoadPluginCandidate("/plugins/custom_pi.dll",true) || stamps!=1)
   throw std::runtime_error("ordinary preexisting crash stamp handling changed");
  if(!p.LoadPluginCandidate("/plugins/system_pi.dll",true) || stamps!=1)
   throw std::runtime_error("ordinary system plugin handling changed");
  std::cout<<"PASS ordinary candidate/system/preexisting-stamp behavior preserved\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
'''
results=[]
for name,body,defines,expected in [
 ('qualified',prefix,['-DSKAGER_UPDATE_QUALIFICATION=1'],0),
 ('ordinary',prefix,[],0),
 ('negative-control',prefix.replace('  if (opennav::integration::QualificationOutputSuppressed()) return false;',''),['-DSKAGER_UPDATE_QUALIFICATION=1'],1)]:
 path=out/(name+'.cpp');path.write_text(prelude+body+'  return true;\n}\n'+postlude)
 binary=out/name
 compile=subprocess.run(['c++','-std=c++17','-Wall','-Wextra',*defines,str(path),'-o',str(binary)],capture_output=True,text=True)
 (out/(name+'-compile.log')).write_text(compile.stdout+compile.stderr)
 if compile.returncode:raise SystemExit(compile.stderr)
 result=subprocess.run([str(binary)],capture_output=True,text=True)
 (out/(name+'-run.log')).write_text(result.stdout+result.stderr)
 results.append({'case':name,'returncode':result.returncode,'expectedReturncode':expected,'sourceSha256':hashlib.sha256(path.read_bytes()).hexdigest(),'binarySha256':hashlib.sha256(binary.read_bytes()).hexdigest()})
 if result.returncode!=expected:raise SystemExit(result.stdout+result.stderr)
 print(name, 'PASS', result.stdout.strip() or result.stderr.strip())
(out/'receipt.json').write_text(json.dumps({'scope':'Actual patched LoadPluginCandidate prefix before wxDateTime with inert wx/log/stamp seams; not whole DLL/plugin acceptance','patchedPluginLoaderSha256':hashlib.sha256(source.encode()).hexdigest(),'cases':results},indent=2)+'\n')
