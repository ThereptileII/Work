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
 OFF(s52plib,vp_plib);OFF(s52plib,m_nSymbolStyle);
 OFF(VPointCompat,pix_width);OFF(VPointCompat,pix_height);
 OFF(ObjRazRules,obj);OFF(ObjRazRules,LUP);
 OFF(S57Obj,att_array);OFF(S57Obj,n_attr);OFF(S57Obj,FeatureName);OFF(S57Obj,Index);OFF(S57Obj,m_lat);OFF(S57Obj,m_lon);
 OFF(Rules,razRule);OFF(Rules,INSTstr);OFF(Rule,name);OFF(Rule,definition);OFF(LUPrec,TNAM);OFF(LUPrec,RCID);OFF(LUPrec,DPRI);OFF(wxPoint,x);OFF(wxPoint,y);
 std::cout<<"\"pointerBytes\":"<<sizeof(void*)<<"}\n";
}
