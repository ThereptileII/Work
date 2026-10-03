#include <wx/wx.h>
#include <cstddef>
#include <iostream>
#define private public
#include "s52plib.h"
#undef private
#include "integration/ChartCaFan.h"
#define OFF(type,member) std::cout << "\"" #type "." #member "\":" << offsetof(type,member) << ",\n"
int main(){
 static_assert(sizeof(void*)==8 && sizeof(double)==8 && sizeof(int)==4);
 std::cout<<"{\n";
 using CaFanGeometry=opennav::integration::CaFanGeometry;
 OFF(CaFanGeometry,center);OFF(CaFanGeometry,leg1);OFF(CaFanGeometry,leg2);OFF(CaFanGeometry,radius);OFF(CaFanGeometry,start);OFF(CaFanGeometry,end);OFF(CaFanGeometry,pixelScale);
 OFF(S52_TextC,bsize);OFF(S52_TextC,rText);
 OFF(S52_TextC,avgCharWidth);OFF(S52_TextC,pFont);OFF(S52_TextC,xoffs);OFF(S52_TextC,yoffs);OFF(S52_TextC,hjust);OFF(S52_TextC,vjust);OFF(S52_TextC,rendered_char_height);OFF(S52_TextC,letter_spacing);OFF(S52_TextC,text_opacity);OFF(S52_TextC,light_label);OFF(S52_TextC,bspecial_char);OFF(S52_TextC,texobj);OFF(S52_TextC,frmtd);
 OFF(s52plib,s_txf);OFF(s52plib,m_colortable_index);OFF(s52plib,m_TextScaleFactor);OFF(s52plib,m_dipfactor);OFF(s52plib,m_ContentScaleFactor);OFF(s52plib,m_FinalTextScaleFactor);
 OFF(TexFontCache,cache);OFF(TexFontCache,key);std::cout<<"\"TexFontCache.size\":"<<sizeof(TexFontCache)<<",\n";
 OFF(S57Obj,FText);OFF(S57Obj,bFText_Added);
 OFF(s52plib,vp_plib);OFF(s52plib,m_nSymbolStyle);
 OFF(VPointCompat,pix_width);OFF(VPointCompat,pix_height);
 OFF(ObjRazRules,obj);OFF(ObjRazRules,LUP);
 OFF(S57Obj,att_array);OFF(S57Obj,n_attr);OFF(S57Obj,FeatureName);OFF(S57Obj,Index);OFF(S57Obj,m_lat);OFF(S57Obj,m_lon);
 OFF(Rules,razRule);OFF(Rules,INSTstr);OFF(Rule,name);OFF(Rule,definition);OFF(LUPrec,TNAM);OFF(LUPrec,RCID);OFF(LUPrec,DPRI);OFF(wxPoint,x);OFF(wxPoint,y);
 std::cout<<"\"pointerBytes\":"<<sizeof(void*)<<"}\n";
}
