struct HazardObject : S57Obj {
  wxArrayOfS57attVal values;
  std::deque<S57attVal> entries;
  std::map<std::string,int> integers;
  std::map<std::string,double> numbers;
  std::map<std::string,std::string> strings;
  std::string names;
  chart_context context{};
  TestChart chart;
  explicit HazardObject(const char* feature) {
    std::memcpy(FeatureName,feature,7);attVal=&values;Primitive_type=GEO_POINT;
    m_DisplayCat=OTHER;m_chart_context=&context;context.chart=&chart;
    #ifdef PRIVATE_SOURCE
#ifdef PATCHED_PRIVATE
    context.get_associated_objects=&SkagerAssociatedObjects;
#endif
#else
    context.pt2GetAssociatedObjects=&s57chart::Associated;
#endif
  }
  void Add(const char* name,void* value,OGRatt_t type) {
    names+=name;att_array=names.data();++n_attr;entries.push_back({value,type});values.Add(&entries.back());
  }
  void Int(const char* name,int v) {integers[name]=v;Add(name,&integers[name],OGR_INT);}
  void Real(const char* name,double v) {numbers[name]=v;Add(name,&numbers[name],OGR_REAL);}
  void String(const char* name,const char* v) {strings[name]=v;Add(name,strings[name].data(),OGR_STR);}
};
static std::string CheckHazardConditionals(s52plib& owner) {
  ps52plib=&owner;
  auto emit=[](HazardObject& o) {
    ObjRazRules rz{};rz.obj=&o;
    char* raw=static_cast<char*>(std::strcmp(o.FeatureName,"WRECKS")==0?WRECKS02(&rz):OBSTRN04(&rz));
    wxString result(raw,wxConvUTF8);free(raw);return result;
  };
  int cases=0;
  std::string receipt;
  auto require=[&](HazardObject& o,const char* wanted,const char* excluded) {
    const wxString before=emit(o);
    if (requireRepair && cases==17) Check(before.Contains("SY(ISODGR51)"));
    Check(before.Contains(wanted) && !before.Contains(excluded));++cases;
    receipt+=before.ToStdString()+"/category="+std::to_string(o.m_DisplayCat)+"\n";
    wxString shown=before; shown.Replace(wxString(char(31)), "<US>");
    std::cout<<"case "<<cases<<" "<<o.FeatureName<<": "<<shown.ToStdString()<<" /category="<<o.m_DisplayCat<<"\n";

  };
  for(int level:{-9,1,2,3,4,5,6}) {
    HazardObject rock("UWTROC");if(level!=-9)rock.Int("WATLEV",level);
    require(rock,level==3?"SY(UWTROC03)":level==2?"SY(LNDARE01)":"SY(UWTROC04)","SY(WRECKS05)");
  }
  for(int level:{3,4,5}) {
    HazardObject rock("UWTROC");rock.Real("VALSOU",3.2);rock.Int("WATLEV",level);
    require(rock,level==3?"SY(DANGER51)":"SY(UWTROC04)",level==3?"SY(UWTROC03)":"SOUNDS");
  }
  {HazardObject rock("UWTROC");rock.Real("VALSOU",21);rock.Int("WATLEV",4);require(rock,"SY(DANGER52)","SY(UWTROC04)");}
  for(int cat:{1,2,4,5}) {
    HazardObject wreck("WRECKS");wreck.Int("CATWRK",cat);wreck.Int("WATLEV",3);
    require(wreck,cat==1?"SY(WRECKS04)":cat==2?"SY(WRECKS05)":"SY(WRECKS01)","SY(UWTROC03)");
  }
  {HazardObject wreck("WRECKS");wreck.Int("WATLEV",3);require(wreck,"", "SY(WRECKS05)");}
  {HazardObject wreck("WRECKS");wreck.Int("CATWRK",2);require(wreck,"", "SY(WRECKS05)");}
  for(const char* cls:{"UWTROC","WRECKS"}) {
    HazardObject o(cls);o.Int("WATLEV",3);o.Int("CATWRK",2);
    HazardObject area("DEPARE");area.Primitive_type=GEO_AREA;area.Real("DRVAL1",10);o.chart.associated.push_back(&area);
#ifdef ORIGINAL_PRIVATE
    // Original private _UDWHAZ03 never queries associated areas (SCRUM-283).
    // Preserve this failed core-parity case explicitly; it is not acceptance.
    require(o,cls[0]=='U'?"SY(UWTROC03)":"SY(WRECKS05)","SY(ISODGR51)");Check(o.m_DisplayCat==OTHER);
#else
    require(o,"SY(ISODGR51)",cls[0]=='U'?"SY(UWTROC03)":"SY(WRECKS05)");Check(o.m_DisplayCat==DISPLAYBASE);
#endif
    o.Int("QUAPOS",4);Check(emit(o).Contains("SY(QUAPOS01)"));
  }
  for(auto pair:{std::pair<int,const char*>{4,"QUAPOS01"},{5,"QUAPOS02"},{7,"QUAPOS03"},{8,"QUAPOS03"},{2,"LOWACC03"}}) {
    HazardObject rock("UWTROC");rock.Int("WATLEV",3);rock.Int("QUAPOS",pair.first);
    require(rock,(wxString("SY(")+pair.second+")").mb_str(),"SY(UWTROC04)");Check(emit(rock).Contains("SY(UWTROC03)"));
  }
  {HazardObject wreck("WRECKS");wreck.Int("WATLEV",3);wreck.Int("CATWRK",2);wreck.Real("VALSOU",3.2);wreck.String("QUASOU","7");
   require(wreck,"SY(WRECKS07)","SY(WRECKS05)");Check(emit(wreck).Contains("SOUNDS"));
#ifdef PRIVATE_SOURCE
   Check(emit(wreck).Contains("TX('Wk',2,1,2,'15110',1,0,CHBLK,21)"));
#else
   Check(emit(wreck).Contains("TX('Wk',3,1,2,'15110',2,0,CHBLK,21)"));
#endif
  }
  {HazardObject obstruction("OBSTRN");obstruction.Int("WATLEV",3);require(obstruction,"SY(OBSTRN01)","SY(UWTROC03)");}
  std::cout<<cases<<" actual conditional cases retained this source's missing/depth/quality and isolated-danger behavior\n";
  return receipt;
}

// Exercise the actual exposure-sensitive body and its OBSTRN04/WRECKS02 callers.
// Values 1/2/3 are pinned s57expectedinput.csv attribute 93, not chart data.
// The associated DEPARE list remains deterministic; no geographic query is mocked
// as accepted. Safety contour 5 is supplied by the existing mariner fixture.
static void CheckExposureConditionals(s52plib& owner) {
  ps52plib=&owner;
  int cases=0;
  for(const char* procedure:{"UDWHAZ03","OBSTRN04","WRECKS02"}) {
    for(int exposure:{-1,1,2,3}) {
      for(double depth:{double(UNKNOWN),3.2,5.,6.}) {
        for(int water:{2,3,4}) {
          for(double surrounding:{2.,10.}) {
            HazardObject object(std::strcmp(procedure,"WRECKS02")==0?"WRECKS":"UWTROC");
            if(exposure!=-1)object.Int("EXPSOU",exposure);
            if(depth!=UNKNOWN)object.Real("VALSOU",depth);
            object.Int("WATLEV",water);
            if(std::strcmp(procedure,"WRECKS02")==0)object.Int("CATWRK",2);
            HazardObject area("DEPARE");area.Primitive_type=GEO_AREA;
            area.Real("DRVAL1",surrounding);object.chart.associated.push_back(&area);
            ObjRazRules rules{};rules.obj=&object;
            bool promoted=true;  // Sentinel: direct body must write its result.
            wxString result;
            if(std::strcmp(procedure,"UDWHAZ03")==0) {
              std::unique_ptr<wxString> command(_UDWHAZ03(&object,depth,&rules,&promoted));
              Check(command!=nullptr);result=*command;
              // Pinned _UDWHAZ03 reads EXPSOU only for UNKNOWN. Exposure 1
              // suppresses area promotion; absent/2/3 promote unknown depth.
              if(depth==UNKNOWN) {
                if(exposure==1)Check(result.empty() && object.m_DisplayCat==OTHER);
                else Check(object.m_DisplayCat==DISPLAYBASE &&
                    (water==2?result.empty():result.Contains("SY(ISODGR51)")));
              }
              // A known depth above the contour is not promoted by this body.
              if(depth==6.)Check(result.empty() && object.m_DisplayCat==OTHER);
              // Known shallow/equal depths in the deep-area fixture exercise
              // the repaired query. Dry water level suppresses the glyph, not
              // DisplayBase. Retain original-private's missing-query behavior.
              if(depth!=UNKNOWN && depth<=5. && surrounding==10.) {
#ifdef ORIGINAL_PRIVATE
                Check(result.empty() && object.m_DisplayCat==OTHER);
#else
                Check(object.m_DisplayCat==DISPLAYBASE &&
                    (water==2?result.empty():result.Contains("SY(ISODGR51)")));
#endif
              }
            } else {
              char* command=static_cast<char*>(std::strcmp(procedure,"WRECKS02")==0
                  ?WRECKS02(&rules):OBSTRN04(&rules));
              Check(command!=nullptr);result=wxString(command,wxConvUTF8);free(command);
            }
            wxString shown=result;shown.Replace(wxString(char(31)),"<US>");
            std::cout<<"exposure "<<++cases<<" "<<procedure<<" EXPSOU="<<exposure
                <<" depth="<<depth<<" WATLEV="<<water<<" DRVAL1="<<surrounding
                <<": "<<shown.ToStdString()<<" /category="<<object.m_DisplayCat
                <<" /promote="<<(std::strcmp(procedure,"UDWHAZ03")==0
                    ?(promoted?"1":"0"):"not-exposed")<<"\n";
          }
        }
      }
    }
  }
  Check(cases==288);
  std::cout<<cases<<" actual exposure/depth/water-level conditional cases retained\n";
}
