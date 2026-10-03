// Real RenderSY/DoRenderObject and loader methods are supplied by the runner.
// Visibility and projection/painter calls remain explicit fixture boundaries.
struct YellowFixture : LightFixture {
  int shape=4;
  YellowFixture() {
    std::memcpy(FeatureName,"BOYSPP",7); Primitive_type=GEO_POINT;
    Add("BOYSHP",&shape,OGR_INT); String("COLOUR","6");
    m_chart_context=nullptr;
  }
};
struct TopmarkFixture : LightFixture {
  int shape=7;
  TopmarkFixture() {
    std::memcpy(FeatureName,"TOPMAR",7); Primitive_type=GEO_POINT;
    Add("TOPSHP",&shape,OGR_INT); String("COLOUR","6");
    m_chart_context=nullptr;
  }
};
static void CheckYellowDispatch(s52plib& owner,std::map<wxString,Rule*>& symbols) {
  Rule* stock=symbols.at("BOYSPP11"); Rule saved=*stock;
  Rule* body=symbols.at("XNSPPY01"); Rule* head=symbols.at("XNSPPT01");
  Rules rules{}; rules.razRule=stock; char instruction[]="BOYSPP11)";rules.INSTstr=instruction;
  LUPrec buoyLookup{};buoyLookup.TNAM=SIMPLIFIED;
  auto draw=[&](S57Obj& obj) {ObjRazRules rz{};rz.obj=&obj;rz.LUP=&buoyLookup;owner.painted=nullptr;owner.RenderSY(&rz,&rules);};
  YellowFixture buoy;
  owner.m_presentationLightSymbols=false;draw(buoy);Check(owner.painted==stock);
  owner.EnablePresentationLightSymbols();draw(buoy);Check(owner.painted==body);
  for(int shape:{3,4,5,6,8}) {YellowFixture x;x.shape=shape;draw(x);Check(owner.painted==body);}
  for(int shape:{0,1,2,7,9,-1}) {YellowFixture x;x.shape=shape;draw(x);Check(owner.painted==stock);}
  for(const char* color:{"","1","11","1,11","6,1","06","6 ","6,6"}) {
    YellowFixture x;x.entries[1].value=const_cast<char*>(color);draw(x);Check(owner.painted==stock);
  }
  for(const char* cat:{"15","52","1,15","52,27","015","","1,","1,,2"," 27","100"}) {
    YellowFixture x;x.String("CATSPM",cat);draw(x);Check(owner.painted==stock);
  }
  for(const char* cat:{"19","27","54","1,27","9","54,27"}) {
    YellowFixture x;x.String("CATSPM",cat);draw(x);Check(owner.painted==body);
  }
  for(const char* name:{"TOPSHP","ORIENT","COLPAT"}) {YellowFixture x;if (!std::strcmp(name,"ORIENT")) x.Number(name,7); else x.String(name,"7");draw(x);Check(owner.painted==stock);}
  for(int i:{0,1}) {
    YellowFixture x;x.entries[i].valType=OGR_INT_LST;draw(x);Check(owner.painted==stock);
    YellowFixture y;y.entries[i].value=nullptr;draw(y);Check(owner.painted==stock);
    YellowFixture z;z.Add(i?"COLOUR":"BOYSHP",z.entries[i].value,z.entries[i].valType);draw(z);Check(owner.painted==stock);
  }
  buoyLookup.TNAM=PAPER_CHART;draw(buoy);Check(owner.painted==stock);buoyLookup.TNAM=SIMPLIFIED;
  Check(!std::memcmp(stock,&saved,sizeof saved) && rules.razRule==stock);

  TopmarkFixture top;chart_context context{};wxArrayPtrVoid platforms;
  context.pFloatingATONArray=&platforms;top.m_chart_context=&context;buoy.m_chart_context=&context;
  platforms.Add(&buoy);LUPrec lup{};lup.RCID=31314;lup.TNAM=SIMPLIFIED;std::memcpy(lup.OBCL,"TOPMAR",7);
  ObjRazRules rz{};rz.obj=&top;rz.LUP=&lup;
  // Actual ProcessLookups appends the canonical unit separator to empty XML.
  lup.INST="\037";
  wxMemoryDC dc;
  auto drawTop=[&](wxDC* native) {
    owner.painted=nullptr;int checks=owner.visibilityChecks;
    owner.DoRenderObject(native,&rz);Check(owner.visibilityChecks==checks+1);
  };
  for(wxDC* native:{static_cast<wxDC*>(&dc),static_cast<wxDC*>(nullptr)}) {
    drawTop(native);Check(owner.painted==head && owner.painter=='R' && owner.m_pdc==native);
    Check(owner.paintedPoint==wxPoint(101,202) && owner.paintedAngle==0);
    owner.visible=false;drawTop(native);Check(owner.painted==nullptr);owner.visible=true;
    owner.m_presentationLightSymbols=false;drawTop(native);Check(owner.painted==nullptr);owner.EnablePresentationLightSymbols();
  }
  // Original default no-draw stays intact without actual fitted-X evidence.
  for(int shape:{0,1,6,8,15,16}) {top.shape=shape;drawTop(&dc);Check(!owner.painted);}top.shape=7;
  for(const char* color:{"","1","6,1","06"}) {top.entries[1].value=const_cast<char*>(color);drawTop(&dc);Check(!owner.painted);}top.entries[1].value=top.strings["COLOUR"].data();
  for(const char* name:{"ORIENT","COLPAT","BOYSHP"}) {TopmarkFixture bad;bad.m_chart_context=&context;bad.String(name,"0");rz.obj=&bad;drawTop(&dc);Check(!owner.painted);}rz.obj=&top;
  lup.RCID=31539;drawTop(&dc);Check(!owner.painted);lup.RCID=31314;
  lup.TNAM=PAPER_CHART;drawTop(&dc);Check(!owner.painted);lup.TNAM=SIMPLIFIED;
  for (const char* inst:{"SY(TOPMAR65)", "SY(TOPMAR65)\037", "\037SY(TOPMAR65)", "\037\037", " "}) {
    lup.INST=inst;drawTop(&dc);Check(!owner.painted);
  }
  lup.INST="\037";
  Rules existing{};existing.ruleType=RUL_NONE;lup.ruleList=&existing;drawTop(&dc);Check(!owner.painted);lup.ruleList=nullptr;
  lup.ATTArray.push_back("TOPSHP7");drawTop(&dc);Check(!owner.painted);lup.ATTArray.clear();
  platforms.Clear();drawTop(&dc);Check(!owner.painted);platforms.Add(&buoy);
  platforms.Add(&buoy);drawTop(&dc);Check(!owner.painted);platforms.RemoveAt(1);
  YellowFixture second;second.m_chart_context=&context;platforms.Add(&second);drawTop(&dc);Check(!owner.painted);platforms.RemoveAt(1);
  buoy.x=1;drawTop(&dc);Check(!owner.painted);buoy.x=0;
  chart_context other{};buoy.m_chart_context=&other;drawTop(&dc);Check(!owner.painted);buoy.m_chart_context=&context;
  buoy.entries[1].value=const_cast<char*>("1,11");drawTop(&dc);Check(!owner.painted);buoy.entries[1].value=buoy.strings["COLOUR"].data();
  YellowFixture superseded;superseded.m_chart_context=&context;superseded.String("CATSPM","15");platforms[0]=&superseded;drawTop(&dc);Check(!owner.painted);platforms[0]=&buoy;
  top.x=std::numeric_limits<double>::quiet_NaN();drawTop(&dc);Check(!owner.painted);top.x=0;
  for(const char* name:{"XNSPPY01","XNSPPT01"}) {
    Rule* original=symbols.at(name);symbols[name]=nullptr;drawTop(&dc);Check(!owner.painted);symbols[name]=original;
    char type=original->definition.SYDF;original->definition.SYDF='V';drawTop(&dc);Check(!owner.painted);original->definition.SYDF=type;
  }
  platforms.Clear();for(int i=0;i<4097;++i)platforms.Add(&buoy);drawTop(&dc);Check(!owner.painted);
  Check(lup.ruleList==nullptr && lup.INST=="\037" && !std::memcmp(stock,&saved,sizeof saved));
}

static void CheckYellowLookups(s52plib& owner,pugi::xml_node lookups) {
  // Fixture marshaling only; the actual pinned comparator and FindBestLUP run.
  wxArrayOfLUPrec array(CompareLUPObjects);
  for(auto node:lookups.children("lookup")) {
    if (std::strcmp(node.attribute("name").value(),"BOYSPP") ||
        std::strcmp(node.child_value("table-name"),"Simplified")) continue;
    auto* lup=new LUPrec{};std::memcpy(lup->OBCL,"BOYSPP",7);
    lup->RCID=node.attribute("RCID").as_int();lup->nSequence=node.attribute("id").as_int();
    lup->INST=wxString::FromUTF8(node.child_value("instruction"));
    for(auto att:node.children("attrib-code"))lup->ATTArray.push_back(att.child_value());
    array.Add(lup);
  }
  Check(array.GetCount()==18);
  for(int shape:{1,2,3,4,5,6,7,8}) {
    for(const char* category:{"", "15", "52", "19", "54", "27", "1,27", "15,27"}) {
      YellowFixture obj;obj.shape=shape;if (*category)obj.String("CATSPM",category);
      auto* selected=owner.FindBestLUP(&array,0,array.GetCount(),&obj,false);Check(selected!=nullptr);
      if (opennav::integration::YellowBuoyAttributes(&obj,false))
        Check(selected->INST.StartsWith("SY(BOYSPP11)"));
      if (!std::strcmp(category,"15")) Check(selected->INST.StartsWith("SY(BOYSUP02)"));
      if (!std::strcmp(category,"52")) Check(selected->INST.StartsWith("SY(BOYDEF03)"));
    }
  }
  for(auto* lup:array)delete lup;
}
