int S57Obj::GetAttributeIndex(const char *AttrSeek) {
  char *patl = att_array;

  for (int i = 0; i < n_attr; i++) {
    if (!strncmp(patl, AttrSeek, 6)) {
      return i;
      break;
    }

    patl += 6;
  }

  return -1;
}
bool GetDoubleAttr(S57Obj *obj, const char *AttrName, double &val) {
  int idx = obj->GetAttributeIndex(AttrName);

  if (idx >= 0) {
    //      using idx to get the attribute value

    S57attVal *v = obj->attVal->Item(idx);
    assert(v->valType == OGR_REAL);
    val = *(double *)(v->value);

    return true;
  } else
    return false;
}
bool GetStringAttr(S57Obj *obj, const char *AttrName, char *pval, int nc) {
  int idx = obj->GetAttributeIndex(AttrName);

  if (idx >= 0) {
    //      using idx to get the attribute value
    S57attVal *v = obj->attVal->Item(idx);

    assert(v->valType == OGR_STR);
    char *val = (char *)(v->value);

    strncpy(pval, val, nc);

    return true;
  } else
    return false;
}
static int _parseList(const char *str_in, char *buf, int buf_size)
// Put a string of comma delimited number in an array (buf).
// Return: the number of value in buf.
// Assume: - number < 256,
//         - list size less than buf_size .
// Note: buf is \0 terminated for strpbrk().
{
  char *str = (char *)str_in;
  int i = 0;

  if (NULL != str && *str != '\0') {
    do {
      if (i >= LISTSIZE - 1) {
        printf("OVERFLOW --value in list lost!!\n");
        break;
      }

      /*
      if (255 <  (unsigned char) atoi(str)) {
          PRINTF("value overflow (>255)\n");
          exit(0);
      }
      */

      buf[i++] = (unsigned char)atoi(str);

      while (isdigit(*str)) str++;  // next

    } while (*str++ != '\0');  // skip ',' or exit
  }

  buf[i] = '\0';

  return i;
}
static void *LIGHTS06(void *param)
// Remarks: A light is one of the most complex S-57 objects. Its presentation
// depends on whether it is a light on a floating or fixed platform, its range,
// it's colour and so on. This conditional symbology procedure derives the
// correct presentation from these parameters and also generates an area that
// shows the coverage of the light.
//
// Notes on light sectors:
// 1.) The radial leg-lines defining the light sectors are normally drawn to
// only 25mm from the light to avoid clutter (see Part C). However, the mariner
// should be able to select "full light-sector lines" and have the leg-lines
// extended to the nominal range of the light (VALMAR).
//
// 2.) Part C of this procedure symbolizes the sectors at the light itself. In
// addition, it should be possible, upon request, for the mariner to be capable
// of identifying the colour and sector limit lines of the sectors affecting the
// ship even if the light itself is off the display. [ed. last sentence in bold]

{
#define UNKNOWN_DOUBLE -9;
  wxString lights06;

  ObjRazRules *rzRules = (ObjRazRules *)param;
  S57Obj *obj = rzRules->obj;

  double valnmr = 9.0;
  GetDoubleAttr(obj, "VALNMR", valnmr);

  char catlitstr[20] = {'\0'};
  GetStringAttr(obj, "CATLIT", catlitstr, 19);

  char litvisstr[20] = {'\0'};
  ;
  GetStringAttr(obj, "LITVIS", litvisstr, 19);

  char catlit[LISTSIZE] = {'\0'};
  char litvis[LISTSIZE] = {'\0'};
  char col_str[20] = {'\0'};

  bool flare_at_45 = false;
  double sectr1 = UNKNOWN_DOUBLE;
  double sectr2 = UNKNOWN_DOUBLE;
  double sweep = 0.;
  char colist[LISTSIZE] = {'\0'};  // colour list
  bool b_isflare = false;

  wxString orientstr;

  if (strlen(catlitstr)) {
    _parseList(catlitstr, catlit, sizeof(colist));

    if (strpbrk(catlit, "\010\013")) {
      lights06.Append(_T(";SY(LIGHTS82)"));
      goto l06_end;
    }

    if (strpbrk(catlit, "\011")) {
      lights06.Append(_T(";SY(LIGHTS81)"));
      goto l06_end;
    }

    /*
     *        if (strpbrk(catlit, "\001\020")) {
     *            orientstr = S57_getAttVal(geo, "ORIENT");
     *            if (NULL != orientstr) {
     *                // FIXME: create a geo object (!?) LINE of lenght VALNMR
     *                // using ORIENT (from seaward) & POINT_T position
     *                g_string_append(lights05, ";LS(DASH,1,CHBLK)");
     }
     }
     */
  }

  // Continuation A

  GetStringAttr(obj, "COLOUR", col_str, 19);

  if (strlen(col_str))
    _parseList(col_str, colist, sizeof(colist));
  else {
    colist[0] = '\014';  // magenta (12)
    colist[1] = '\000';
  }

  GetDoubleAttr(obj, "SECTR1", sectr1);
  GetDoubleAttr(obj, "SECTR2", sectr2);

  if ((-9 == sectr1) || (-9 == sectr2)) {
    // This is not a sector light

    wxString ssym;

    if (valnmr < 10.0) {
      // TODO create LightArray in s57chart.
      //  Then, if another LIGHT object is colocated here, set flare_at_45
      /*            if(_atPtPos(obj, rzRules->chart->pLIGHTSArray, false)) // Is
       * this LIGHTS feature colocated with another LIGHTS?
       *
       *
       *            //    If the light is white, yellow, or orange, make it a
       * flare at 45 degrees if(strpbrk(colist, "\001\005\011")) flare_at_45 =
       * true;
       */
      ssym = _selSYcol(colist, 0, valnmr);  // flare
      b_isflare = true;
      flare_at_45 = false;
    } else {
      ssym = _selSYcol(colist, 1, valnmr);  // all round light
      b_isflare = false;
    }

    //  Is the light a directional or moire?
    if (strpbrk(catlit, "\001\016")) {
      if (orientstr.Len()) {
        lights06.Append(ssym);
        lights06.Append(orientstr);
        lights06.Append(
            _T(";TE('%03.0lf deg','ORIENT',3,3,3,'15110',3,1,CHBLK,23)" ));
      } else
        lights06.Append(_T(";SY(QUESMRK1)"));
    } else {
      lights06.Append(ssym);

      if (b_isflare) {
        if (flare_at_45)
          lights06.Append(_T(",45)"));
        else
          lights06.Append(_T(",135)"));
      }
    }

    goto l06_end;
  }

  // Continuation B --sector light
  if (-9 == sectr1) {
    sectr1 = 0.0;
    sectr2 = 0.0;
  } else
    sweep = (sectr1 > sectr2) ? sectr2 - sectr1 + 360 : sectr2 - sectr1;

  if (sweep < 1.0 || sweep == 360.0) {
    // handle all round light
    wxString ssym = _selSYcol(colist, 1, valnmr);  // all round light
    lights06.Append(ssym);

    /*
     *        if (TRUE == S52_getMarinerParam(S52_MAR_SHOW_TEXT)) {
     *            GString *litdsn01 = _LITDSN01(geo);
     *            if (NULL != litdsn01) {
     *                g_string_append(lights05, ";TX('");
     *                g_string_append(lights05, litdsn01->str);
     *                g_string_append(lights05, "',3,2,3,'15110',2,0,CHBLK,23)"
     );
     *                g_string_free(litdsn01, TRUE);
     }

     }
     */
    goto l06_end;
  }

  /*
   *    // scan for other lights with sector overlap at this position
   *    // compute light sector radius according to other sector
   *    if (1 == S52_state)
   *    {
   *        _setPtPos(geo, SECTRLIST);
   *        g_string_free(lights05, TRUE);
   *        return NULL;
   }
   else
   {
       extend_arc_radius = _atPtPos(geo, SECTRLIST);

       // passe value via attribs to _renderAC
       if (extend_arc_radius)
           // FIXME: draw radius 25 mm
           S57_setAtt(geo, "extend_arc_radius", "Y");
       else
           // FIXME: draw radius 20 mm
           S57_setAtt(geo, "extend_arc_radius", "N");
   }
   */
  // setup sector
  {
    //        Build the (opencpn private) command string like this:
    //        e.g.  ";CA(OUTLW, 4,LITRD, 2, sectr1, sectr2, radius)"

    double arc_radius = 20.;  // mm
    double sector_radius = 25.;

    //      Another non-standard extension....
    //      Sector light arc radius is scaled if the light has a reasonable
    //      VALNMR attribute
    if (valnmr > 0) {
      if (valnmr < 15.0)
        arc_radius = 10.;
      else if (valnmr < 30.0)
        arc_radius = 15.;
      else
        arc_radius = 20.;
    }

    char sym[80];
    strcpy(sym, ";CA(OUTLW, 4");

    // max 1 color
    if ('\0' == colist[1]) {
      if (strpbrk(colist, "\003"))
        strcat(sym, ",LITRD, 2");
      else if (strpbrk(colist, "\004"))
        strcat(sym, ",LITGN, 2");
      else if (strpbrk(colist, "\001\006\013"))
        strcat(sym, ",LITYW, 2");
      else
        strcat(sym, ",CHMGD, 2");  // default is magenta

    } else if ('\0' == colist[2]) {
      if (strpbrk(colist, "\001") && strpbrk(colist, "\003"))
        strcat(sym, ",LITRD, 2");
      else if (strpbrk(colist, "\001") && strpbrk(colist, "\004"))
        strcat(sym, ",LITGN, 2");
      else
        strcat(sym, ",CHMGD, 2");  // default is magenta
    } else
      strcat(sym, ",CHMGD, 2");  // default is magenta

    if (strlen(litvisstr))  // Obscured/faint sector?
    {
      _parseList(litvisstr, litvis, sizeof(litvis));

      if (strpbrk(litvis, "\003\007\010")) strcpy(sym, ";CA(CHBLK, 4,CHBRN, 1");
    }

    if (sectr2 <= sectr1) sectr2 += 360;

    //    Sectors are defined from seaward
    if (sectr1 > 180)
      sectr1 -= 180;
    else
      sectr1 += 180;

    if (sectr2 > 180)
      sectr2 -= 180;
    else
      sectr2 += 180;

    char arc_data[80];
    sprintf(arc_data, ",%5.1f, %5.1f, %5.1f, %5.1f", sectr1, sectr2, arc_radius,
            sector_radius);

    strcat(sym, arc_data);

    wxString ssym(sym, wxConvUTF8);
    lights06 = ssym;

    goto l06_end;
  }

l06_end:

  //      if( ps52plib->m_bShowLdisText )
  {
    // Only show Light in certain position once. Otherwise there will be
    // clutter.
    static double lastLat, lastLon;
    static wxString lastDescription;
    bool isFirstSector = true;

    if (lastLat == obj->m_lat && lastLon == obj->m_lon) isFirstSector = false;
    lastLat = obj->m_lat;
    lastLon = obj->m_lon;

    wxString litdsn01 = _LITDSN01(obj);

    if (litdsn01.Len() && isFirstSector) {
      lastDescription = litdsn01;
      lights06.Append(_T(";TX('"));
      lights06.Append(litdsn01);

      if (flare_at_45)
        lights06.Append(_T("',3,3,3,'15110',2,-1,CHBLK,23)" ));
      else
        lights06.Append(_T("',3,2,3,'15110',2,0,CHBLK,23)" ));
    }

    if (!isFirstSector && lastDescription != litdsn01) {
      lastDescription = litdsn01;
      lights06.Append(_T(";TX('"));
      lights06.Append(litdsn01);
      lights06.Append(_T("',3,2,3,'15110',2,1,CHBLK,23)" ));
    }
  }

  lights06.Append('\037');

  char *r = (char *)malloc(lights06.Len() + 1);
  strcpy(r, lights06.mb_str());

  return r;
}
int s52plib::RenderSY(ObjRazRules *rzRules, Rules *rules) {
  float angle = 0;
  double orient;

  if (rules->razRule != NULL) {
    if (rules->INSTstr[8] ==
        ',')  // supplementary parameter assumed to be angle, seen in LIGHTSXX
    {
      char sangle[10];
      int cp = 0;
      while (rules->INSTstr[cp + 9] && (rules->INSTstr[cp + 9] != ')')) {
        sangle[cp] = rules->INSTstr[cp + 9];
        cp++;
      }
      sangle[cp] = 0;
      int angle_i = atoi(sangle);
      angle = angle_i;
    }

    if (GetDoubleAttr(
            rzRules->obj, "ORIENT",
            orient))  // overriding any LIGHTSXX angle, probably TSSLPT
    {
      angle = orient;
      if (strncmp(rzRules->obj->FeatureName, "LIGHTS", 6) == 0) {
        angle += 180;
        if (angle > 360) angle -= 360;
      }
    }

    //  Render symbol at object's x/y
    wxPoint r, r1;
    GetPointPixSingle(rzRules, rzRules->obj->y, rzRules->obj->x, &r);

    // Keep original vectors and metadata for any actual ORIENT. Owned aliases
    // are stable library-owned Rules with independent raster caches; never
    // rewrite the shared conditional Rule or copy its cache ownership.
    Rule *paintRule = rules->razRule;
#ifdef OPENNAV_X
    const char *alias = opennav::integration::PresentationLightAlias(
        m_presentationLightSymbols, rzRules->obj->FeatureName,
        paintRule->name.SYNM, rzRules->obj->att_array, rzRules->obj->n_attr);
    if (alias && paintRule->definition.SYDF == 'V') {
      auto found = _symb_sym->find(wxString::FromUTF8(alias));
      if (found != _symb_sym->end() && found->second &&
          found->second->definition.SYDF == 'R' &&
          !memcmp(found->second->name.SYNM, alias, 8))
        paintRule = found->second;
    }
#endif

    //  Render a raster or vector symbol, as specified by LUP rules
    if (paintRule->definition.SYDF == 'V') {
      RenderHPGL(rzRules, paintRule, r, angle, m_ChartScaleFactorExp);
    } else {
      if (paintRule->definition.SYDF == 'R')
        RenderRasterSymbol(rzRules, paintRule, r, angle);
    }
  }

  return 0;
}
