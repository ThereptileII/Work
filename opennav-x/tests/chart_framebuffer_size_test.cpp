// Actual production repair method, with bounded resize/allocation outcomes.
#include <wx/gdicmn.h>
#include <iostream>
#include <stdexcept>
struct ParentCanvas {wxSize size;wxSize GetClientSize()const{return size;}};
class glChartCanvas {
 public:
  ParentCanvas *m_pParentCanvas;
  wxSize child;
  bool m_b_BuiltFBO=true,fail=false,resize_allocates=false;
  int m_cache_tex_x=0,m_cache_tex_y=0,scale=1,resizes=0,builds=0;
  bool undersized_result=false;
  wxSize GetSize()const{return child;}
  void SetSize(wxSize size){child=size;++resizes;if(resize_allocates)BuildFBO();}
  void BuildFBO(){++builds;m_b_BuiltFBO=!fail;m_cache_tex_x=child.x*scale;m_cache_tex_y=child.y*scale;if(undersized_result)m_cache_tex_y-=8;}
  bool PrepareChartFramebuffer(int,int);
};
#include "chart-framebuffer-method.inc"
static int checks=0;
void Check(bool yes,const char *why){++checks;if(!yes)throw std::runtime_error(why);}
int main(){try{
 for(int scale:{1,2})for(bool delivered:{false,true})for(bool synchronous:{false,true}){
  ParentCanvas parent{{1014,566}};glChartCanvas c;c.m_pParentCanvas=&parent;c.scale=scale;
  c.child=delivered?parent.size:wxSize(1012,558);c.resize_allocates=synchronous;
  c.m_cache_tex_x=1012*scale;c.m_cache_tex_y=558*scale;
  // This is the measured failing capture: the first eight viewport rows would
  // sample beyond the texture's end, then wrap to its opposite edge.
  Check((566.*scale)/(558.*scale)>1.,"Baseline sampling exceeds allocated texture");
  Check(c.PrepareChartFramebuffer(1014*scale,566*scale),"Final layout can use repaired cache");
  Check(c.child==parent.size,"Child adopts final native client size");
  Check(c.m_cache_tex_x==1014*scale&&c.m_cache_tex_y==566*scale,"Allocated cache covers full physical viewport");
  Check(c.builds==1,"One allocation regardless of resize-event delivery order");
  const int resizes=c.resizes,builds=c.builds;
  for(int i=0;i<20;++i)Check(c.PrepareChartFramebuffer(1014*scale,566*scale),"Unchanged frames stay cached");
  Check(c.resizes==resizes&&c.builds==builds,"Normal repaint never allocates or resizes");
  parent.size={1012,558};Check(c.PrepareChartFramebuffer(1012*scale,558*scale),"Temporary stock theme layout remains covered");
  parent.size={1014,566};Check(c.PrepareChartFramebuffer(1014*scale,566*scale),"Restored final layout remains covered");
 }
 for(bool fail:{false,true}){
  ParentCanvas parent{{1014,566}};glChartCanvas c;c.m_pParentCanvas=&parent;c.child=parent.size;c.m_cache_tex_x=1012;c.m_cache_tex_y=558;
  c.fail=fail;c.undersized_result=!fail;
  Check(!c.PrepareChartFramebuffer(1014,566),"Failed or insufficient allocation must bypass FBO, never sample wrapped chart");
  Check(c.builds==1,"One bounded allocation attempt");
 }
 ParentCanvas parent{{1014,566}};glChartCanvas c;c.m_pParentCanvas=&parent;c.child=parent.size;c.m_cache_tex_x=2048;c.m_cache_tex_y=2048;
 Check(c.PrepareChartFramebuffer(1014,566)&&c.builds==0,"Larger valid cache remains usable");
 c.m_b_BuiltFBO=false;Check(!c.PrepareChartFramebuffer(1014,566)&&c.builds==0,"Disabled/unavailable FBO stays on direct renderer");
 Check(!c.PrepareChartFramebuffer(0,566)&&!c.PrepareChartFramebuffer(1014,0)&&c.builds==0,"Invalid viewport cannot trigger allocation");
 std::cout<<checks<<" actual framebuffer lifecycle assertions passed\n";
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
