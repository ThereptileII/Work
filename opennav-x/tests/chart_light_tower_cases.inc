struct TowerFixture : LightFixture {
  int visibility=2;
  explicit TowerFixture(bool conspicuous=false) {
    std::memcpy(FeatureName,"LNDMRK",7);Primitive_type=GEO_POINT;
    String("CATLMK","17");String("FUNCTN","33");
    if(conspicuous){visibility=1;Add("CONVIS",&visibility,OGR_INT);}
  }
};

static void CheckTowerDispatch(s52plib& owner,std::map<wxString,Rule*>& symbols,
                               pugi::xml_node lookups,ChartSymbols& loader) {
  wxArrayOfLUPrec array(CompareLUPObjects);
  for(auto node:lookups.children("lookup")) {
    if(std::strcmp(node.attribute("name").value(),"LNDMRK") ||
       std::strcmp(node.child_value("table-name"),"Simplified"))continue;
    auto* lup=new LUPrec{};std::memcpy(lup->OBCL,"LNDMRK",7);
    lup->TNAM=SIMPLIFIED;lup->RCID=node.attribute("RCID").as_int();
    lup->nSequence=node.attribute("id").as_int();
    lup->INST=wxString::FromUTF8(node.child_value("instruction"));
    for(auto att:node.children("attrib-code"))lup->ATTArray.push_back(att.child_value());
    array.Add(lup);
  }
  Check(array.GetCount()>20);
  for(bool conspicuous:{false,true}) {
    const char* source=conspicuous?"TOWERS03":"TOWERS01";
    const char* name=conspicuous?"XNLTWR03":"XNLTWR01";
    auto* stock=symbols.at(source);auto* alias=symbols.at(name);Rule saved=*stock;
    // The tower resources remain, but a light-support tower now paints the
    // prototype lighthouse instead of a re-inked tower (owner, 2026-10-10).
    auto* lighthouse=symbols.at("XNLIT013");
    Check(stock->definition.SYDF=='R' && alias->definition.SYDF=='R');
    Check(alias->RCID==(conspicuous?60019:60018));
    Check(alias->pos.symb.pivot_x.SYCL==6 && alias->pos.symb.pivot_y.SYRW==22);
    Check(alias->pos.symb.bnbox_w.SYHL==14 && alias->pos.symb.bnbox_h.SYVL==26);
    wxRect rect;loader.GetGLTextureRect(rect,name);
    Check(rect==wxRect(conspicuous?1004:980,1160,14,26));
    TowerFixture valid(conspicuous);
    auto* selected=owner.FindBestLUP(&array,0,array.GetCount(),&valid,false);
    Check(selected && selected->RCID==(conspicuous?31192:31199));
    Check(selected->INST.StartsWith(conspicuous?"SY(TOWERS03);TX(OBJNAM":"SY(TOWERS01);TX(OBJNAM"));
    Rules rules{};rules.razRule=stock;std::string instruction=std::string(source)+")";rules.INSTstr=instruction.data();
    auto draw=[&](S57Obj& object,LUPrec* lookup) {
      ObjRazRules rz{};rz.obj=&object;rz.LUP=lookup;owner.RenderSY(&rz,&rules);
      Check(rules.razRule==stock && !std::memcmp(stock,&saved,sizeof saved));
    };
    owner.m_presentationLightSymbols=false;draw(valid,selected);Check(owner.painted==stock);
    owner.EnablePresentationLightSymbols();
    for(const char* theme:{"DAY","DUSK","NIGHT","DAY"}) {
      owner.m_ColorScheme=theme;
      for(double scale:{.5,1.,2.,4.}) {
        owner.m_ChartScaleFactorExp=scale;draw(valid,selected);
        Check(owner.painted==lighthouse && owner.painter=='R' && owner.paintedPoint==wxPoint(101,202) && owner.paintedAngle==0);
      }
      Check(loader.LoadRasterFileForColorTable(loader.FindColorTable(!std::strcmp(theme,"DAY")?"DAY_BRIGHT":theme),true,ChartCtx(false,0)));
      auto old=loader.GetImage(source),owned=loader.GetImage(name);
      Check(old.IsOk() && owned.IsOk() && owned.HasAlpha() && owned.GetWidth()==14 && owned.GetHeight()==26);
      for(int y=0;y<26;++y)for(int x=0;x<14;++x)Check(old.GetAlpha(x,y)==owned.GetAlpha(x,y));
    }
    owner.m_ChartScaleFactorExp=1;
    const auto rcid=selected->RCID;selected->RCID=31220;draw(valid,selected);Check(owner.painted==stock);selected->RCID=rcid;
    selected->TNAM=PAPER_CHART;draw(valid,selected);Check(owner.painted==stock);selected->TNAM=SIMPLIFIED;
    TowerFixture other(conspicuous);std::memcpy(other.FeatureName,"LIGHTS",7);draw(other,selected);Check(owner.painted==stock);
    TowerFixture area(conspicuous);area.Primitive_type=GEO_AREA;draw(area,selected);Check(owner.painted==stock);
    for(const char* attribute:{"CATLMK","FUNCTN"}) {
      const int index=!std::strcmp(attribute,"CATLMK")?0:1;
      for(const char* value:{"", "017", "033", "17,1", "33,1", "17 ", "33 ", "20"}) {
        TowerFixture bad(conspicuous);bad.entries[index].value=const_cast<char*>(value);draw(bad,selected);Check(owner.painted==stock);
      }
      TowerFixture duplicate(conspicuous);duplicate.String(attribute,index?"33":"17");draw(duplicate,selected);Check(owner.painted==stock);
      for(auto kind:{OGR_INT,OGR_REAL,OGR_INT_LST}) {
        TowerFixture bad(conspicuous);bad.entries[index].valType=kind;draw(bad,selected);Check(owner.painted==stock);
      }
      TowerFixture missing(conspicuous);std::memcpy(missing.att_array+index*6,"UNKNWN",6);draw(missing,selected);Check(owner.painted==stock);
      TowerFixture null(conspicuous);null.entries[index].value=nullptr;draw(null,selected);Check(owner.painted==stock);
    }
    for(const char* excluded:{"ORIENT","STATUS","QUAPOS","QUASOU"}) {
      TowerFixture bad(conspicuous);bad.Number(excluded,0);draw(bad,selected);Check(owner.painted==stock);
    }
    TowerFixture visibility(conspicuous);
    if(!conspicuous)visibility.Add("CONVIS",&visibility.visibility,OGR_INT);
    draw(visibility,selected);Check(owner.painted==lighthouse);
    visibility.visibility=conspicuous?2:1;draw(visibility,selected);Check(owner.painted==stock);
    TowerFixture count(conspicuous);++count.n_attr;draw(count,selected);Check(owner.painted==stock);
    TowerFixture labelled(conspicuous);labelled.String("OBJNAM","Fixture light support");draw(labelled,selected);Check(owner.painted==lighthouse);
    // A missing, null, vector or misnamed lighthouse definition keeps stock.
    symbols.erase("XNLIT013");draw(valid,selected);Check(owner.painted==stock);symbols["XNLIT013"]=nullptr;draw(valid,selected);Check(owner.painted==stock);
    symbols["XNLIT013"]=lighthouse;lighthouse->definition.SYDF='V';draw(valid,selected);Check(owner.painted==stock);lighthouse->definition.SYDF='R';
    const char first=lighthouse->name.SYNM[0];lighthouse->name.SYNM[0]='?';draw(valid,selected);Check(owner.painted==stock);lighthouse->name.SYNM[0]=first;
  }
  for(auto* lookup:array)delete lookup;
}
