#include <wx/wx.h>
#include <cstddef>
#include <iostream>
#define private public
#include "s52plib.h"
#undef private
#define OFF(type,member) std::cout << "\"" #type "." #member "\":" << offsetof(type,member) << ",\n"
int main(){
 static_assert(sizeof(void*)==8 && sizeof(double)==8 && sizeof(int)==4);
 std::cout<<"{\n";
 OFF(s52plib,m_presentationLightSymbols);OFF(s52plib,m_presentationCaLights);OFF(s52plib,m_useGLSL);OFF(s52plib,m_pdc);OFF(s52plib,vp_plib);OFF(s52plib,m_nSymbolStyle);
 OFF(VPointCompat,pix_width);OFF(VPointCompat,pix_height);
 OFF(ObjRazRules,next);OFF(ObjRazRules,obj);OFF(ObjRazRules,LUP);
 OFF(S57Obj,Primitive_type);OFF(S57Obj,m_chart_context);OFF(S57Obj,x);OFF(S57Obj,y);OFF(S57Obj,att_array);OFF(S57Obj,n_attr);OFF(S57Obj,FeatureName);OFF(S57Obj,Index);OFF(S57Obj,m_lat);OFF(S57Obj,m_lon);
 OFF(Rules,next);OFF(Rules,ruleType);OFF(Rules,b_private_razRule);OFF(Rules,razRule);OFF(Rules,INSTstr);OFF(Rule,RCID);OFF(Rule,pos.symb.bnbox_w.SYHL);OFF(Rule,pos.symb.bnbox_h.SYVL);OFF(Rule,pos.symb.pivot_x.SYCL);OFF(Rule,pos.symb.pivot_y.SYRW);OFF(Rule,pos.symb.bnbox_x.SBXC);OFF(Rule,pos.symb.bnbox_y.SBXR);OFF(Rule,name);OFF(Rule,definition);OFF(LUPrec,ruleList);OFF(LUPrec,FTYP);OFF(LUPrec,RPRI);OFF(LUPrec,DISC);OFF(LUPrec,LUCM);OFF(LUPrec,OBCL);OFF(LUPrec,TNAM);OFF(LUPrec,RCID);OFF(LUPrec,DPRI);OFF(wxPoint,x);OFF(wxPoint,y);
 std::unordered_map<const S57Obj*,const char*> example;
 example.emplace(reinterpret_cast<const S57Obj*>(0x1000),"one");
 example.emplace(reinterpret_cast<const S57Obj*>(0x2000),"two");
 example.emplace(reinterpret_cast<const S57Obj*>(0x3000),"three");
 size_t observed=0;std::memcpy(&observed,reinterpret_cast<const char*>(&example)+24,sizeof(observed));
 if(observed!=example.size() || observed!=3)return 2;
 std::cout<<"\"inventoryMapCountOffset\":24,\n";
 std::cout<<"\"pointerBytes\":"<<sizeof(void*)<<"}\n";
}
