#pragma once
#include "integration/ChartCaFan.h"

namespace opennav::integration {
struct CaAllRoundStyle { unsigned color=0; long radius=0; };
// Full-circle extension of prototype arc paint, not a supplied all-round glyph.
// The original LIGHTS06/_selSYcol output remains authoritative for geometry.
inline CaAllRoundStyle CaAllRoundLight(bool enabled,const ObjRazRules* node) {
  if(!enabled || !node || !node->obj || !node->LUP)return {};
  const auto* obj=node->obj;const auto* lup=node->LUP;
  const auto* text=CaFanInstruction(lup->INST);
  if(obj->Primitive_type!=GEO_POINT || std::memcmp(obj->FeatureName,"LIGHTS",6) ||
     lup->RCID!=31183 || lup->TNAM!=SIMPLIFIED ||
     std::memcmp(lup->OBCL,"LIGHTS",6) || !lup->ATTArray.empty() || !text ||
     (*text!="CS(LIGHTS05)" && *text!="CS(LIGHTS05)\037") ||
     obj->n_attr<2 || obj->n_attr>4096 || !obj->att_array || !obj->attVal ||
     obj->attVal->GetCount()!=unsigned(obj->n_attr))return {};
  unsigned seen=0,color=0;double range=0,start=0,end=0;
  for(int i=0;i<obj->n_attr;++i) {
    const char* name=obj->att_array+i*6;
    for(const char* excluded:{"ORIENT","CATLIT","LITVIS","STATUS","QUAPOS","QUASOU"})
      if(!std::memcmp(name,excluded,6))return {};
    unsigned bit=0;
    if(!std::memcmp(name,"COLOUR",6))bit=1;
    else if(!std::memcmp(name,"VALNMR",6))bit=2;
    else if(!std::memcmp(name,"SECTR1",6))bit=4;
    else if(!std::memcmp(name,"SECTR2",6))bit=8;
    if(!bit)continue;
    if(seen&bit)return {};
    seen|=bit;
    const auto* value=obj->attVal->Item(i);
    if(!value || !value->value)return {};
    if(bit==1) {
      if(value->valType!=OGR_STR)return {};
      const char* s=static_cast<const char*>(value->value);
      if((s[0]!='1' && s[0]!='3' && s[0]!='4') || s[1])return {};
      color=unsigned(s[0]-'0');
    } else {
      if(value->valType!=OGR_REAL)return {};
      const double n=*static_cast<const double*>(value->value);
      if(!std::isfinite(n))return {};
      if(bit==2){if(n<=0)return {};range=n;}
      else {if(n<0 || n>360)return {};if(bit==4)start=n;else end=n;}
    }
  }
  if((seen&3)!=3)return {};
  if(!(seen&12)) {if(range<10)return {};}
  else if((seen&12)!=12 || (start!=end && std::abs(end-start)!=360))return {};
  // Pinned _selSYcol range bands and single-color offsets. This is a signature
  // check only: painters use their original, already-scaled radius and center.
  const long band=range<7?3:range<15?10:range<30?15:20;
  return {color,band+(color==1?2:color==3?1:0)};
}
inline bool CaAllRoundInstruction(CaAllRoundStyle style,const wxString& outline,
    long outlineWidth,const wxString& arc,long arcWidth,double start,double end,
    long radius,long sectorRadius) {
  return style.color && outline=="OUTLW" && outlineWidth==4 && arcWidth==2 &&
    arc==(style.color==3?"LITRD":style.color==4?"LITGN":"LITYW") &&
    start==0 && end==360 && radius==style.radius && sectorRadius==0;
}
} // namespace opennav::integration
