int s52plib::RenderCARC_GLSL(ObjRazRules *rzRules, Rules *rules) {

  //    glDisable( GL_SCISSOR_TEST );

  char *str = (char *)rules->INSTstr;
  //    extract the parameters from the string
  //    And creating a unique string hash as we go
  wxString inst(str, wxConvUTF8);

  wxStringTokenizer tkz(inst, _T ( ",;" ));

  //    outline color
  wxString outline_color = tkz.GetNextToken();

  //    outline width
  wxString slong = tkz.GetNextToken();
  long outline_width;
  slong.ToLong(&outline_width);

  //    arc color
  wxString arc_color = tkz.GetNextToken();

  //    arc width
  slong = tkz.GetNextToken();
  long arc_width;
  slong.ToLong(&arc_width);

  //    sectr1
  slong = tkz.GetNextToken();
  double sectr1;
  slong.ToDouble(&sectr1);

  //    sectr2
  slong = tkz.GetNextToken();
  double sectr2;
  slong.ToDouble(&sectr2);

  //    arc radius
  slong = tkz.GetNextToken();
  long radius;
  slong.ToLong(&radius);

  //    sector radius
  slong = tkz.GetNextToken();
  long sector_radius;
  slong.ToLong(&sector_radius);
#ifdef SKAGER_OCHARTS_ADAPTER
  const unsigned fanColor = opennav::integration::CaFanColor(
      PresentationCaLightsEnabled(), rzRules);
  opennav::integration::CaFanPaint fanPaint;
  const bool fanEligible = fanColor && outline_color == "OUTLW" &&
      outline_width == 4 && arc_width == 2 && sector_radius > 0 &&
      arc_color == (fanColor == 3 ? "LITRD" : fanColor == 4 ? "LITGN" : "LITYW") &&
      opennav::integration::CaFanColors(fanColor, m_ColorScheme, fanPaint);
  const auto roundStyle = opennav::integration::CaAllRoundLight(
      PresentationCaLightsEnabled(), rzRules);
  opennav::integration::CaFanPaint roundPaint;
  const bool roundEligible = opennav::integration::CaAllRoundInstruction(
      roundStyle, outline_color, outline_width, arc_color, arc_width,
      sectr1, sectr2, radius, sector_radius) &&
      opennav::integration::CaFanColors(roundStyle.color, m_ColorScheme, roundPaint);
#endif


  // Center point
  wxPoint r;
  GetPointPixSingle(rzRules, rzRules->obj->y, rzRules->obj->x, &r);

  //  radius scaled to display
  float rad = radius * canvas_pix_per_mm;
  float sec_rad = sector_radius * canvas_pix_per_mm;

  float arcw = arc_width * canvas_pix_per_mm;
  // On larger screens, make the arc_width 1.0 mm
  if ( m_display_size_mm > 200)     //200 mm, about 8 inches
    arcw = canvas_pix_per_mm;


  // Adjust size
  //  Some plain lights have no SCAMIN attribute.
  //  This causes display congestion at small viewing scales, since the objects
  //  are rendered at fixed pixel dimensions from the LUP rules. As a
  //  correction, the idea is to not allow the rendered symbol to be larger than
  //  "X" meters on the chart.
  //   and scale it down when rendered if necessary.

  float xscale = 1.0;
  if (rzRules->obj->Scamin > 1e8) {  // huge (unset) SCAMIN)
    float radius_meters_target = 200;

    float radius_meters = (radius * canvas_pix_per_mm) / vp_plib.view_scale_ppm;

    xscale = radius_meters_target / radius_meters;
    xscale = wxMin(xscale, 1.0);
    xscale = wxMax(.4, xscale);

    rad *= xscale;
    arcw *= xscale;
    arcw = wxMin(arcw, rad / 10);
    sec_rad *= xscale;
  }
  //      Enable anti-aliased lines, at best quality


  // Rotate the center point about vp center
  wxPoint point = r;
  double sin_rot = sin(vp_plib.rotation);
  double cos_rot = cos(vp_plib.rotation);

  double xp = ((point.x - vp_plib.pix_width / 2) * cos_rot) -
              ((point.y - vp_plib.pix_height / 2) * sin_rot);
  double yp = ((point.x - vp_plib.pix_width / 2) * sin_rot) +
              ((point.y - vp_plib.pix_height / 2) * cos_rot);

  point.x = (int)xp + vp_plib.pix_width / 2;
  point.y = (int)yp + vp_plib.pix_height / 2;
#ifdef SKAGER_OCHARTS_ADAPTER
  if (!m_pdc && roundEligible) {
    opennav::integration::CaFanTile tile;
    if (tile.BuildAllRound(point, rad, m_ContentScaleFactor / m_dipfactor, roundPaint) &&
        opennav::integration::DrawCaFanGL(tile, S52texture_2D_shader_program,
                                         vp_plib.pix_width, vp_plib.pix_height)) {
      // Same original all-round radius/center and bounding-box convention.
      double latmin, lonmin, latmax, lonmax;
      GetPixPointSingleNoRotate(r.x - rad, r.y + rad, &latmin, &lonmin);
      GetPixPointSingleNoRotate(r.x + rad, r.y - rad, &latmax, &lonmax);
      LLBBox box; box.Set(latmin, lonmin, latmax, lonmax);
      rzRules->obj->BBObj.Expand(box);
      return 1;
    }
  }
  if (!m_pdc && fanEligible) {
    const int leg_len = (int)sec_rad;
    float a = (sectr1 - 90) * PI / 180;
    a += vp_plib.rotation;
    const wxPoint leg1(point.x + (int)(leg_len * cosf(a)),
                       point.y + (int)(leg_len * sinf(a)));
    a = (sectr2 - 90) * PI / 180.;
    a += vp_plib.rotation;
    const wxPoint leg2(point.x + (int)(leg_len * cosf(a)),
                       point.y + (int)(leg_len * sinf(a)));
    // Match the original shader's float rounding and wrap convention.
    float sr1 = sectr1 + (vp_plib.rotation * 180 / PI);
    if (sr1 > 360.) sr1 -= 360.;
    float sr2 = sectr2 + (vp_plib.rotation * 180 / PI);
    if (sr2 > 360.) sr2 -= 360.;
    float sb = sr1, se = sr2 > sr1 ? sr2 : sr2 + 360;
    if ((sb < 0) || (se < 0)) { sb += 360.; se += 360.; }
    opennav::integration::CaFanTile tile;
    if (tile.Build({point, leg1, leg2, rad, sb, se,
                    m_ContentScaleFactor / m_dipfactor}, fanPaint) &&
        opennav::integration::DrawCaFanGL(tile, S52texture_2D_shader_program,
                                         vp_plib.pix_width, vp_plib.pix_height)) {
      // Full conservative radius includes wash center, legs and antialiasing.
      const double extent = wxMax(rad, sec_rad) +
                            ceil(1.3 * m_ContentScaleFactor / m_dipfactor) + 3;
      double latmin, lonmin, latmax, lonmax;
      GetPixPointSingleNoRotate(r.x - extent, r.y + extent, &latmin, &lonmin);
      GetPixPointSingleNoRotate(r.x + extent, r.y - extent, &latmax, &lonmax);
      LLBBox box; box.Set(latmin, lonmin, latmax, lonmax);
      rzRules->obj->BBObj.Expand(box);
      return 1;
    }
  }
#endif
  glEnable(GL_BLEND);


  float rad_fluff = rad + 10;
  float coords[8];
  coords[0] = -rad_fluff;
  coords[1] = rad_fluff;
  coords[2] = rad_fluff;
  coords[3] = rad_fluff;
  coords[4] = -rad_fluff;
  coords[5] = -rad_fluff;
  coords[6] = rad_fluff;
  coords[7] = -rad_fluff;

  glUseProgram(S52ring_shader_program);

  // Get pointers to the attributes in the program.
  GLint mPosAttrib = glGetAttribLocation(S52ring_shader_program, "aPos");

  // Disable VBO's (vertex buffer objects) for attributes.
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  glVertexAttribPointer(mPosAttrib, 2, GL_FLOAT, GL_FALSE, 0, coords);
  glEnableVertexAttribArray(mPosAttrib);

  //  Circle radius
  GLint radiusloc =
      glGetUniformLocation(S52ring_shader_program, "circle_radius");
  glUniform1f(radiusloc, rad);

  //  Circle center point, physical
  GLint centerloc =
      glGetUniformLocation(S52ring_shader_program, "circle_center");
  float ctrv[2];
  ctrv[0] = point.x;
  ctrv[1] = vp_plib.pix_height - point.y;
  glUniform2fv(centerloc, 1, ctrv);

  //  Border color
  float bcolorv[4];
  bcolorv[0] = 0;
  bcolorv[1] = 0;
  bcolorv[2] = 0;
  bcolorv[3] = 1.0;

  GLint ring_colloc = glGetUniformLocation(S52ring_shader_program, "ring_color");
  glUniform4fv(ring_colloc, 1, bcolorv);

  //  Border Width
  GLint ringWidthloc =
      glGetUniformLocation(S52ring_shader_program, "ring_width");
  glUniform1f(ringWidthloc, arcw + (1 * outline_width));

  //  Visible sectors, rotated to vp orientation
  float sr1 = sectr1 + (vp_plib.rotation * 180 / PI);
  if (sr1 > 360.) sr1 -= 360.;
  float sr2 = sectr2 + (vp_plib.rotation * 180 / PI);
  if (sr2 > 360.) sr2 -= 360.;

  float sb, se;
  if (sr2 > sr1) {
    sb = sr1;
    se = sr2;
  } else {
    sb = sr1;
    se = sr2 + 360;
  }

  //  Shader can handle angles > 360.
  if ((sb < 0) || (se < 0)) {
    sb += 360.;
    se += 360.;
  }

  GLint sector1loc = glGetUniformLocation(S52ring_shader_program, "sector_1");
  glUniform1f(sector1loc, (sb * PI / 180.));
  GLint sector2loc = glGetUniformLocation(S52ring_shader_program, "sector_2");
  glUniform1f(sector2loc, (se * PI / 180.));

  // Rotate and translate
  mat4x4 I;
  mat4x4_identity(I);

  mat4x4_translate_in_place(I, r.x, r.y, 0);

  GLint matloc =
      glGetUniformLocation(S52ring_shader_program, "TransformMatrix");
  glUniformMatrix4fv(matloc, 1, GL_FALSE, (const GLfloat *)I);

  // Perform the actual drawing.
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

  // Now draw color ring
  //  Circle color
  wxColour colorb = getwxColour(arc_color);
  float colorv[4];
  colorv[0] = colorb.Red() / float(256);
  colorv[1] = colorb.Green() / float(256);
  colorv[2] = colorb.Blue() / float(256);
  colorv[3] = 1.0;
  glUniform4fv(ring_colloc, 1, colorv);

  //  arc Width
  glUniform1f(ringWidthloc, arcw);

   // Perform the actual drawing.
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);


  // Restore the per-object transform to Identity Matrix
  mat4x4 IM;
  mat4x4_identity(IM);
  GLint matlocf =
      glGetUniformLocation(S52ring_shader_program, "TransformMatrix");
  glUniformMatrix4fv(matlocf, 1, GL_FALSE, (const GLfloat *)IM);
  glUseProgram(0);
  glDisableVertexAttribArray(mPosAttrib);


  //    Draw the sector legs directly on the target DC
  if (sector_radius > 0) {
    int leg_len = (int)(sec_rad);

    wxDash dash1[2];
    dash1[0] = (int)(3.6 * canvas_pix_per_mm / 3);  // 8// Long dash <---------+
    dash1[1] = (int)(1.8 * canvas_pix_per_mm / 3);  // 2// Short gap |

    wxPen thispen = *wxBLACK_PEN;
    thispen.SetDashes(2, dash1);
    thispen.SetWidth(2);
    thispen.SetStyle(wxPENSTYLE_USER_DASH);

    float a = (sectr1 - 90) * PI / 180;
    a += vp_plib.rotation;
    int x = point.x + (int)(leg_len * cosf(a));
    int y = point.y + (int)(leg_len * sinf(a));
    DrawDashLine(thispen, point.x, point.y, x, y);

    a = (sectr2 - 90) * PI / 180.;
    a += vp_plib.rotation;
    x = point.x + (int)(leg_len * cosf(a));
    y = point.y + (int)(leg_len * sinf(a));
    DrawDashLine(thispen, point.x, point.y, x, y);
  }
  glDisable(GL_BLEND);

  //  Update the object Bounding box,
  //  so that subsequent drawing operations will redraw the item fully

  double latmin, lonmin, latmax, lonmax;

  GetPixPointSingleNoRotate(r.x - rad, r.y + rad, &latmin, &lonmin);
  GetPixPointSingleNoRotate(r.x + rad, r.y - rad, &latmax, &lonmax);
  LLBBox symbox;
  symbox.Set(latmin, lonmin, latmax, lonmax);
  rzRules->obj->BBObj.Expand(symbox);

  //    glEnable( GL_SCISSOR_TEST );

  return 1;
}
int s52plib::RenderCARC_VBO(ObjRazRules *rzRules, Rules *rules) {

  char *str = (char *)rules->INSTstr;
  //    extract the parameters from the string
  //    And creating a unique string hash as we go
  wxString inst(str, wxConvUTF8);
  wxString carc_hash;

  wxStringTokenizer tkz(inst, _T ( ",;" ));

  //    outline color
  wxString outline_color = tkz.GetNextToken();
  carc_hash += outline_color;
  carc_hash += _T(".");

  //    outline width
  wxString slong = tkz.GetNextToken();
  long outline_width;
  slong.ToLong(&outline_width);
  carc_hash += slong;
  carc_hash += _T(".");

  //    arc color
  wxString arc_color = tkz.GetNextToken();
  carc_hash += arc_color;
  carc_hash += _T(".");

  //    arc width
  slong = tkz.GetNextToken();
  long arc_width;
  slong.ToLong(&arc_width);
  carc_hash += slong;
  carc_hash += _T(".");

  //    sectr1
  slong = tkz.GetNextToken();
  double sectr1;
  slong.ToDouble(&sectr1);
  carc_hash += slong;
  carc_hash += _T(".");

  //    sectr2
  slong = tkz.GetNextToken();
  double sectr2;
  slong.ToDouble(&sectr2);
  carc_hash += slong;
  carc_hash += _T(".");

  //    arc radius
  slong = tkz.GetNextToken();
  long radius;
  slong.ToLong(&radius);
  carc_hash += slong;
  carc_hash += _T(".");

  //    sector radius
  slong = tkz.GetNextToken();
  long sector_radius;
  slong.ToLong(&sector_radius);
#ifdef SKAGER_OCHARTS_ADAPTER
  const unsigned fanColor = opennav::integration::CaFanColor(
      PresentationCaLightsEnabled(), rzRules);
  opennav::integration::CaFanPaint fanPaint;
  const bool fanEligible = fanColor && outline_color == "OUTLW" &&
      outline_width == 4 && arc_width == 2 && sector_radius > 0 &&
      arc_color == (fanColor == 3 ? "LITRD" : fanColor == 4 ? "LITGN" : "LITYW") &&
      opennav::integration::CaFanColors(fanColor, m_ColorScheme, fanPaint);
  const auto roundStyle = opennav::integration::CaAllRoundLight(
      PresentationCaLightsEnabled(), rzRules);
  opennav::integration::CaFanPaint roundPaint;
  const bool roundEligible = opennav::integration::CaAllRoundInstruction(
      roundStyle, outline_color, outline_width, arc_color, arc_width,
      sectr1, sectr2, radius, sector_radius) &&
      opennav::integration::CaFanColors(roundStyle.color, m_ColorScheme, roundPaint);
#endif

  carc_hash += slong;
  carc_hash += _T(".");

  slong.Printf(_T("%d"), m_colortable_index);
  carc_hash += slong;

  int width;
  int height;
  int rad;
  int bm_width;
  int bm_height;
  int bm_orgx;
  int bm_orgy;

  Rule *prule = rules->razRule;

  float scale_factor = 1.0;

  // The dimensions of the light are presented here as pixels on-screen.
  // We must scale the rendered size based on the device pixel density
  // Let us declare that the width of the arc should be no less than X mm
  float wx = 1.0;

  // float pd_scale = 1.0;
  float nominal_arc_width_pix = wxMax(
      1.0,
      floor(GetPPMM() * wx));  // { wx } mm nominal, but not less than 1 pixel
  // pd_scale = nominal_arc_width_pix / arc_width;

  // scale_factor *= pd_scale;
  // qDebug() << GetPPMM() << arc_width << nominal_arc_width_pix << pd_scale;

  // Adjust size
  //  Some plain lights have no SCAMIN attribute.
  //  This causes display congestion at small viewing scales, since the objects
  //  are rendered at fixed pixel dimensions from the LUP rules. As a
  //  correction, the idea is to not allow the rendered symbol to be larger than
  //  "X" meters on the chart.
  //   and scale it down when rendered if necessary.

  float xscale = 1.0;
  if (rzRules->obj->Scamin > 1e8) {  // huge (unset) SCAMIN)
    float radius_meters_target = 200;

    float radius_meters = (radius * canvas_pix_per_mm) / vp_plib.view_scale_ppm;

    xscale = radius_meters_target / radius_meters;
    xscale = wxMin(xscale, 1.0);
    xscale = wxMax(.4, xscale);

    radius *= xscale;
    sector_radius *= xscale;
  }

  /// scale_factor *= xscale;

  carc_hash += _T(".");
  wxString xs;
  xs.Printf(_T("%5g"), xscale);
  carc_hash += xs;

  if (m_pdc) {  // DC rendering
#ifdef SKAGER_OCHARTS_ADAPTER
    if (roundEligible) {
      wxPoint center;
      GetPointPixSingle(rzRules, rzRules->obj->y, rzRules->obj->x, &center);
      const int roundRadius = (int)(radius * canvas_pix_per_mm);
      opennav::integration::CaFanTile tile;
      if (tile.BuildAllRound(center, roundRadius, m_ContentScaleFactor / m_dipfactor,
                             roundPaint) && tile.Draw(*m_pdc)) {
        // The stock full-circle bitmap has four pixels of border fluff.
        const int extent = roundRadius + 4;
        double latmin, lonmin, latmax, lonmax;
        GetPixPointSingleNoRotate(center.x - extent, center.y + extent, &latmin, &lonmin);
        GetPixPointSingleNoRotate(center.x + extent, center.y - extent, &latmax, &lonmax);
        LLBBox box; box.Set(latmin, lonmin, latmax, lonmax);
        rzRules->obj->BBObj.Expand(box);
        return 1;
      }
    }
    if (fanEligible) {
      wxPoint center;
      GetPointPixSingle(rzRules, rzRules->obj->y, rzRules->obj->x, &center);
      const int fan_radius = (int)(radius * canvas_pix_per_mm);
      const int leg_len = (int)(sector_radius * canvas_pix_per_mm);
      float a = (sectr1 - 90) * PI / 180;
      const wxPoint leg1(center.x + (int)(leg_len * cosf(a)),
                         center.y + (int)(leg_len * sinf(a)));
      a = (sectr2 - 90) * PI / 180.;
      const wxPoint leg2(center.x + (int)(leg_len * cosf(a)),
                         center.y + (int)(leg_len * sinf(a)));
      opennav::integration::CaFanTile tile;
      if (tile.Build({center, leg1, leg2, double(fan_radius), sectr1, sectr2,
                      m_ContentScaleFactor / m_dipfactor}, fanPaint) && tile.Draw(*m_pdc)) {
        double latmin, lonmin, latmax, lonmax;
        GetPixPointSingleNoRotate(tile.bounds.x, tile.bounds.GetBottom(), &latmin, &lonmin);
        GetPixPointSingleNoRotate(tile.bounds.GetRight(), tile.bounds.y, &latmax, &lonmax);
        LLBBox box; box.Set(latmin, lonmin, latmax, lonmax);
        rzRules->obj->BBObj.Expand(box);
        return 1;
      }
    }
#endif

    if (fabs(prule->parm7 - xscale) > .00001) {
      ClearRulesCache(prule);
    }

    // Instantiate the symbol if necessary
    if ((rules->razRule->pixelPtr == NULL) ||
        (rules->razRule->parm1 != m_colortable_index)) {
      //  Render the sector light to a bitmap

      rad = (int)(radius * canvas_pix_per_mm);

      width = (rad * 2) + 28;
      height = (rad * 2) + 28;
      wxBitmap bm(width, height, -1);
      wxMemoryDC mdc;
      mdc.SelectObject(bm);
      mdc.SetBackground(wxBrush(m_unused_wxColor));
      mdc.Clear();

      //    Adjust sector math for wxWidgets API
      float sb;
      float se;

      //      For some reason, the __WXMSW__ build flips the sense of
      //      start and end angles on DrawEllipticArc()
#ifndef __WXMSW__
      if (sectr2 > sectr1) {
        sb = 90 - sectr1;
        se = 90 - sectr2;
      } else {
        sb = 360 + (90 - sectr1);
        se = 90 - sectr2;
      }
#else
      if (sectr2 > sectr1) {
        se = 90 - sectr1;
        sb = 90 - sectr2;
      } else {
        se = 360 + (90 - sectr1);
        sb = 90 - sectr2;
      }
#endif

      //      Here is a goofy way of computing the dc drawing extents exactly
      //      Draw a series of fat line segments approximating the arc using
      //      dc.DrawLine() This will properly establish the drawing box in the
      //      dc

      int border_fluff = 4;  // by how much should the blit bitmap be "fluffed"

      //  wxDC min/max calculations are currently broken in wxQT, so we use the
      //  entire circle instead of arcs...
#ifndef __WXQT__
      if (fabs(sectr2 - sectr1) != 360)  // not necessary for all-round lights
      {
        mdc.ResetBoundingBox();

        wxPen *pblockpen =
            wxThePenList->FindOrCreatePen(*wxBLACK, 10, wxPENSTYLE_SOLID);
        mdc.SetPen(*pblockpen);

        float start_angle, end_angle;
        if (se < sb) {
          start_angle = se;
          end_angle = sb;
        } else {
          start_angle = sb;
          end_angle = se;
        }

        int x0 = (width / 2) + (int)(rad * cos(start_angle * PI / 180.));
        int y0 = (height / 2) - (int)(rad * sin(start_angle * PI / 180.));
        for (float a = start_angle + .1; a <= end_angle; a += 2.0) {
          int x = (width / 2) + (int)(rad * cosf(a * PI / 180.));
          int y = (height / 2) - (int)(rad * sinf(a * PI / 180.));
          mdc.DrawLine(x0, y0, x, y);
          x0 = x;
          y0 = y;
        }

        bm_width = (mdc.MaxX() - mdc.MinX()) + (border_fluff * 2);
        bm_height = (mdc.MaxY() - mdc.MinY()) + (border_fluff * 2);
        bm_orgx = mdc.MinX() - border_fluff -
                  width / 2;  // wxMax ( 0, mdc.MinX()-border_fluff );
        bm_orgy = mdc.MinY() - border_fluff -
                  height / 2;  // wxMax ( 0, mdc.MinY()-border_fluff );

        mdc.Clear();
      }

      else {
        bm_width = rad * 2 + (border_fluff * 2);
        bm_height = rad * 2 + (border_fluff * 2);
        bm_orgx = -bm_width / 2;
        bm_orgy = -bm_height / 2;
      }

#else
      bm_width = rad * 2 + (border_fluff * 2);
      bm_height = rad * 2 + (border_fluff * 2);
      bm_orgx = -bm_width / 2;
      bm_orgy = -bm_height / 2;
#endif

      wxBitmap *sbm = NULL;

      //    Draw the outer border
      wxColour color = getwxColour(outline_color);

      wxPen *pthispen = wxThePenList->FindOrCreatePen(
          color, outline_width * scale_factor, wxPENSTYLE_SOLID);
      mdc.SetPen(*pthispen);
      wxBrush *pthisbrush =
          wxTheBrushList->FindOrCreateBrush(color, wxBRUSHSTYLE_TRANSPARENT);
      mdc.SetBrush(*pthisbrush);

      mdc.DrawEllipticArc(width / 2 - rad, height / 2 - rad, rad * 2, rad * 2,
                          sb, se);

      if (arc_width) {
        wxColour colorb = getwxColour(arc_color);

        if (!colorb.IsOk()) colorb = getwxColour(_T("CHMGD"));

        pthispen = wxThePenList->FindOrCreatePen(
            colorb, arc_width * scale_factor, wxPENSTYLE_SOLID);
        mdc.SetPen(*pthispen);

        mdc.DrawEllipticArc(width / 2 - rad, height / 2 - rad, rad * 2, rad * 2,
                            sb, se);
      }

      mdc.SelectObject(wxNullBitmap);

      //          Get smallest containing bitmap
      sbm = new wxBitmap(bm.GetSubBitmap(wxRect(
          width / 2 + bm_orgx, height / 2 + bm_orgy, bm_width, bm_height)));

      //      Make the mask
      wxMask *pmask = new wxMask(*sbm, m_unused_wxColor);

      //      Associate the mask with the bitmap
      sbm->SetMask(pmask);

      // delete any old private data
      ClearRulesCache(rules->razRule);

      //      Save the bitmap ptr and aux parms in the rule
      prule->pixelPtr = sbm;
      prule->parm0 = ID_wxBitmap;
      prule->parm1 = m_colortable_index;
      prule->parm2 = bm_orgx;
      prule->parm3 = bm_orgy;
      prule->parm5 = bm_width;
      prule->parm6 = bm_height;
      prule->parm7 = xscale;
    }  // instantiation
  }

  //  Render arcs at object's x/y
  wxPoint r;
  GetPointPixSingle(rzRules, rzRules->obj->y, rzRules->obj->x, &r);

  //      Now render the symbol
  if (!m_pdc)  // opengl
  {
  } else {
    int b_width = prule->parm5;
    int b_height = prule->parm6;

    //      Get the bitmap into a memory dc
    wxMemoryDC mdc;
    mdc.SelectObject((wxBitmap &)(*((wxBitmap *)(rules->razRule->pixelPtr))));

    //      Blit it into the target dc, using mask
    m_pdc->Blit(r.x + rules->razRule->parm2, r.y + rules->razRule->parm3,
                b_width, b_height, &mdc, 0, 0, wxCOPY, true);

    mdc.SelectObject(wxNullBitmap);

    //    Draw the sector legs directly on the target DC
    //    so that anti-aliasing works against the drawn image (cannot be
    //    cached...)
    if (sector_radius > 0) {
      int leg_len = (int)(sector_radius * canvas_pix_per_mm);

      wxDash dash1[2];
      dash1[0] = (int)(3.6 * canvas_pix_per_mm);  // 8// Long dash  <---------+
      dash1[1] = (int)(1.8 * canvas_pix_per_mm);  // 2// Short gap            |

      /*
       wxPen *pthispen = new wxPen(*wxBLACK_PEN);
       pthispen->SetStyle(wxPENSTYLE_USER_DASH);
       pthispen->SetDashes( 2, dash1 );
       //      Undocumented "feature":  Pen must be fully specified <<<BEFORE>>>
       setting into DC pdc->SetPen ( *pthispen );
       */
      // wxColour c = GetGlobalColor( _T ( "CHBLK" ) );
      wxColour c;
      GetGlobalColor(_T ( "CHBLK" ), &c);

      float a = (sectr1 - 90) * PI / 180;
      int x = r.x + (int)(leg_len * cosf(a));
      int y = r.y + (int)(leg_len * sinf(a));
      DrawAALine(m_pdc, r.x, r.y, x, y, c, dash1[0], dash1[1]);

      a = (sectr2 - 90) * PI / 180.;
      x = r.x + (int)(leg_len * cosf(a));
      y = r.y + (int)(leg_len * sinf(a));
      DrawAALine(m_pdc, r.x, r.y, x, y, c, dash1[0], dash1[1]);
    }

    // Debug the symbol bounding box.....
    /*
            if(m_pdc){
                m_pdc->SetPen(wxPen(*wxGREEN, 1));
                m_pdc->SetBrush(wxBrush(*wxGREEN, wxTRANSPARENT));
                m_pdc->DrawRectangle( r.x + rules->razRule->parm2, r.y +
       rules->razRule->parm3, b_width, b_height);
            }
            */
  }

  //  Update the object Bounding box,
  //  so that subsequent drawing operations will redraw the item fully

  double latmin, lonmin, latmax, lonmax;

  GetPixPointSingleNoRotate(r.x + prule->parm2,
                            r.y + prule->parm3 + prule->parm6, &latmin, &lonmin);
  GetPixPointSingleNoRotate(r.x + prule->parm2 + prule->parm5,
                            r.y + prule->parm3, &latmax, &lonmax);
  LLBBox symbox;
  symbox.Set(latmin, lonmin, latmax, lonmax);
  rzRules->obj->BBObj.Expand(symbox);
  return 1;
}
int s52plib::StockCARC_GLSL(ObjRazRules *rzRules, Rules *rules) {

  //    glDisable( GL_SCISSOR_TEST );

  char *str = (char *)rules->INSTstr;
  //    extract the parameters from the string
  //    And creating a unique string hash as we go
  wxString inst(str, wxConvUTF8);

  wxStringTokenizer tkz(inst, _T ( ",;" ));

  //    outline color
  wxString outline_color = tkz.GetNextToken();

  //    outline width
  wxString slong = tkz.GetNextToken();
  long outline_width;
  slong.ToLong(&outline_width);

  //    arc color
  wxString arc_color = tkz.GetNextToken();

  //    arc width
  slong = tkz.GetNextToken();
  long arc_width;
  slong.ToLong(&arc_width);

  //    sectr1
  slong = tkz.GetNextToken();
  double sectr1;
  slong.ToDouble(&sectr1);

  //    sectr2
  slong = tkz.GetNextToken();
  double sectr2;
  slong.ToDouble(&sectr2);

  //    arc radius
  slong = tkz.GetNextToken();
  long radius;
  slong.ToLong(&radius);

  //    sector radius
  slong = tkz.GetNextToken();
  long sector_radius;
  slong.ToLong(&sector_radius);

  // Center point
  wxPoint r;
  GetPointPixSingle(rzRules, rzRules->obj->y, rzRules->obj->x, &r);

  //  radius scaled to display
  float rad = radius * canvas_pix_per_mm;
  float sec_rad = sector_radius * canvas_pix_per_mm;

  float arcw = arc_width * canvas_pix_per_mm;
  // On larger screens, make the arc_width 1.0 mm
  if ( m_display_size_mm > 200)     //200 mm, about 8 inches
    arcw = canvas_pix_per_mm;


  // Adjust size
  //  Some plain lights have no SCAMIN attribute.
  //  This causes display congestion at small viewing scales, since the objects
  //  are rendered at fixed pixel dimensions from the LUP rules. As a
  //  correction, the idea is to not allow the rendered symbol to be larger than
  //  "X" meters on the chart.
  //   and scale it down when rendered if necessary.

  float xscale = 1.0;
  if (rzRules->obj->Scamin > 1e8) {  // huge (unset) SCAMIN)
    float radius_meters_target = 200;

    float radius_meters = (radius * canvas_pix_per_mm) / vp_plib.view_scale_ppm;

    xscale = radius_meters_target / radius_meters;
    xscale = wxMin(xscale, 1.0);
    xscale = wxMax(.4, xscale);

    rad *= xscale;
    arcw *= xscale;
    arcw = wxMin(arcw, rad / 10);
    sec_rad *= xscale;
  }
  //      Enable anti-aliased lines, at best quality
  glEnable(GL_BLEND);

  // Rotate the center point about vp center
  wxPoint point = r;
  double sin_rot = sin(vp_plib.rotation);
  double cos_rot = cos(vp_plib.rotation);

  double xp = ((point.x - vp_plib.pix_width / 2) * cos_rot) -
              ((point.y - vp_plib.pix_height / 2) * sin_rot);
  double yp = ((point.x - vp_plib.pix_width / 2) * sin_rot) +
              ((point.y - vp_plib.pix_height / 2) * cos_rot);

  point.x = (int)xp + vp_plib.pix_width / 2;
  point.y = (int)yp + vp_plib.pix_height / 2;

  float rad_fluff = rad + 10;
  float coords[8];
  coords[0] = -rad_fluff;
  coords[1] = rad_fluff;
  coords[2] = rad_fluff;
  coords[3] = rad_fluff;
  coords[4] = -rad_fluff;
  coords[5] = -rad_fluff;
  coords[6] = rad_fluff;
  coords[7] = -rad_fluff;

  glUseProgram(S52ring_shader_program);

  // Get pointers to the attributes in the program.
  GLint mPosAttrib = glGetAttribLocation(S52ring_shader_program, "aPos");

  // Disable VBO's (vertex buffer objects) for attributes.
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  glVertexAttribPointer(mPosAttrib, 2, GL_FLOAT, GL_FALSE, 0, coords);
  glEnableVertexAttribArray(mPosAttrib);

  //  Circle radius
  GLint radiusloc =
      glGetUniformLocation(S52ring_shader_program, "circle_radius");
  glUniform1f(radiusloc, rad);

  //  Circle center point, physical
  GLint centerloc =
      glGetUniformLocation(S52ring_shader_program, "circle_center");
  float ctrv[2];
  ctrv[0] = point.x;
  ctrv[1] = vp_plib.pix_height - point.y;
  glUniform2fv(centerloc, 1, ctrv);

  //  Border color
  float bcolorv[4];
  bcolorv[0] = 0;
  bcolorv[1] = 0;
  bcolorv[2] = 0;
  bcolorv[3] = 1.0;

  GLint ring_colloc = glGetUniformLocation(S52ring_shader_program, "ring_color");
  glUniform4fv(ring_colloc, 1, bcolorv);

  //  Border Width
  GLint ringWidthloc =
      glGetUniformLocation(S52ring_shader_program, "ring_width");
  glUniform1f(ringWidthloc, arcw + (1 * outline_width));

  //  Visible sectors, rotated to vp orientation
  float sr1 = sectr1 + (vp_plib.rotation * 180 / PI);
  if (sr1 > 360.) sr1 -= 360.;
  float sr2 = sectr2 + (vp_plib.rotation * 180 / PI);
  if (sr2 > 360.) sr2 -= 360.;

  float sb, se;
  if (sr2 > sr1) {
    sb = sr1;
    se = sr2;
  } else {
    sb = sr1;
    se = sr2 + 360;
  }

  //  Shader can handle angles > 360.
  if ((sb < 0) || (se < 0)) {
    sb += 360.;
    se += 360.;
  }

  GLint sector1loc = glGetUniformLocation(S52ring_shader_program, "sector_1");
  glUniform1f(sector1loc, (sb * PI / 180.));
  GLint sector2loc = glGetUniformLocation(S52ring_shader_program, "sector_2");
  glUniform1f(sector2loc, (se * PI / 180.));

  // Rotate and translate
  mat4x4 I;
  mat4x4_identity(I);

  mat4x4_translate_in_place(I, r.x, r.y, 0);

  GLint matloc =
      glGetUniformLocation(S52ring_shader_program, "TransformMatrix");
  glUniformMatrix4fv(matloc, 1, GL_FALSE, (const GLfloat *)I);

  // Perform the actual drawing.
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

  // Now draw color ring
  //  Circle color
  wxColour colorb = getwxColour(arc_color);
  float colorv[4];
  colorv[0] = colorb.Red() / float(256);
  colorv[1] = colorb.Green() / float(256);
  colorv[2] = colorb.Blue() / float(256);
  colorv[3] = 1.0;
  glUniform4fv(ring_colloc, 1, colorv);

  //  arc Width
  glUniform1f(ringWidthloc, arcw);

   // Perform the actual drawing.
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);


  // Restore the per-object transform to Identity Matrix
  mat4x4 IM;
  mat4x4_identity(IM);
  GLint matlocf =
      glGetUniformLocation(S52ring_shader_program, "TransformMatrix");
  glUniformMatrix4fv(matlocf, 1, GL_FALSE, (const GLfloat *)IM);
  glUseProgram(0);
  glDisableVertexAttribArray(mPosAttrib);


  //    Draw the sector legs directly on the target DC
  if (sector_radius > 0) {
    int leg_len = (int)(sec_rad);

    wxDash dash1[2];
    dash1[0] = (int)(3.6 * canvas_pix_per_mm / 3);  // 8// Long dash <---------+
    dash1[1] = (int)(1.8 * canvas_pix_per_mm / 3);  // 2// Short gap |

    wxPen thispen = *wxBLACK_PEN;
    thispen.SetDashes(2, dash1);
    thispen.SetWidth(2);
    thispen.SetStyle(wxPENSTYLE_USER_DASH);

    float a = (sectr1 - 90) * PI / 180;
    a += vp_plib.rotation;
    int x = point.x + (int)(leg_len * cosf(a));
    int y = point.y + (int)(leg_len * sinf(a));
    DrawDashLine(thispen, point.x, point.y, x, y);

    a = (sectr2 - 90) * PI / 180.;
    a += vp_plib.rotation;
    x = point.x + (int)(leg_len * cosf(a));
    y = point.y + (int)(leg_len * sinf(a));
    DrawDashLine(thispen, point.x, point.y, x, y);
  }
  glDisable(GL_BLEND);

  //  Update the object Bounding box,
  //  so that subsequent drawing operations will redraw the item fully

  double latmin, lonmin, latmax, lonmax;

  GetPixPointSingleNoRotate(r.x - rad, r.y + rad, &latmin, &lonmin);
  GetPixPointSingleNoRotate(r.x + rad, r.y - rad, &latmax, &lonmax);
  LLBBox symbox;
  symbox.Set(latmin, lonmin, latmax, lonmax);
  rzRules->obj->BBObj.Expand(symbox);

  //    glEnable( GL_SCISSOR_TEST );

  return 1;
}
int s52plib::StockCARC_VBO(ObjRazRules *rzRules, Rules *rules) {

  char *str = (char *)rules->INSTstr;
  //    extract the parameters from the string
  //    And creating a unique string hash as we go
  wxString inst(str, wxConvUTF8);
  wxString carc_hash;

  wxStringTokenizer tkz(inst, _T ( ",;" ));

  //    outline color
  wxString outline_color = tkz.GetNextToken();
  carc_hash += outline_color;
  carc_hash += _T(".");

  //    outline width
  wxString slong = tkz.GetNextToken();
  long outline_width;
  slong.ToLong(&outline_width);
  carc_hash += slong;
  carc_hash += _T(".");

  //    arc color
  wxString arc_color = tkz.GetNextToken();
  carc_hash += arc_color;
  carc_hash += _T(".");

  //    arc width
  slong = tkz.GetNextToken();
  long arc_width;
  slong.ToLong(&arc_width);
  carc_hash += slong;
  carc_hash += _T(".");

  //    sectr1
  slong = tkz.GetNextToken();
  double sectr1;
  slong.ToDouble(&sectr1);
  carc_hash += slong;
  carc_hash += _T(".");

  //    sectr2
  slong = tkz.GetNextToken();
  double sectr2;
  slong.ToDouble(&sectr2);
  carc_hash += slong;
  carc_hash += _T(".");

  //    arc radius
  slong = tkz.GetNextToken();
  long radius;
  slong.ToLong(&radius);
  carc_hash += slong;
  carc_hash += _T(".");

  //    sector radius
  slong = tkz.GetNextToken();
  long sector_radius;
  slong.ToLong(&sector_radius);
  carc_hash += slong;
  carc_hash += _T(".");

  slong.Printf(_T("%d"), m_colortable_index);
  carc_hash += slong;

  int width;
  int height;
  int rad;
  int bm_width;
  int bm_height;
  int bm_orgx;
  int bm_orgy;

  Rule *prule = rules->razRule;

  float scale_factor = 1.0;

  // The dimensions of the light are presented here as pixels on-screen.
  // We must scale the rendered size based on the device pixel density
  // Let us declare that the width of the arc should be no less than X mm
  float wx = 1.0;

  // float pd_scale = 1.0;
  float nominal_arc_width_pix = wxMax(
      1.0,
      floor(GetPPMM() * wx));  // { wx } mm nominal, but not less than 1 pixel
  // pd_scale = nominal_arc_width_pix / arc_width;

  // scale_factor *= pd_scale;
  // qDebug() << GetPPMM() << arc_width << nominal_arc_width_pix << pd_scale;

  // Adjust size
  //  Some plain lights have no SCAMIN attribute.
  //  This causes display congestion at small viewing scales, since the objects
  //  are rendered at fixed pixel dimensions from the LUP rules. As a
  //  correction, the idea is to not allow the rendered symbol to be larger than
  //  "X" meters on the chart.
  //   and scale it down when rendered if necessary.

  float xscale = 1.0;
  if (rzRules->obj->Scamin > 1e8) {  // huge (unset) SCAMIN)
    float radius_meters_target = 200;

    float radius_meters = (radius * canvas_pix_per_mm) / vp_plib.view_scale_ppm;

    xscale = radius_meters_target / radius_meters;
    xscale = wxMin(xscale, 1.0);
    xscale = wxMax(.4, xscale);

    radius *= xscale;
    sector_radius *= xscale;
  }

  /// scale_factor *= xscale;

  carc_hash += _T(".");
  wxString xs;
  xs.Printf(_T("%5g"), xscale);
  carc_hash += xs;

  if (m_pdc) {  // DC rendering
    if (fabs(prule->parm7 - xscale) > .00001) {
      ClearRulesCache(prule);
    }

    // Instantiate the symbol if necessary
    if ((rules->razRule->pixelPtr == NULL) ||
        (rules->razRule->parm1 != m_colortable_index)) {
      //  Render the sector light to a bitmap

      rad = (int)(radius * canvas_pix_per_mm);

      width = (rad * 2) + 28;
      height = (rad * 2) + 28;
      wxBitmap bm(width, height, -1);
      wxMemoryDC mdc;
      mdc.SelectObject(bm);
      mdc.SetBackground(wxBrush(m_unused_wxColor));
      mdc.Clear();

      //    Adjust sector math for wxWidgets API
      float sb;
      float se;

      //      For some reason, the __WXMSW__ build flips the sense of
      //      start and end angles on DrawEllipticArc()
#ifndef __WXMSW__
      if (sectr2 > sectr1) {
        sb = 90 - sectr1;
        se = 90 - sectr2;
      } else {
        sb = 360 + (90 - sectr1);
        se = 90 - sectr2;
      }
#else
      if (sectr2 > sectr1) {
        se = 90 - sectr1;
        sb = 90 - sectr2;
      } else {
        se = 360 + (90 - sectr1);
        sb = 90 - sectr2;
      }
#endif

      //      Here is a goofy way of computing the dc drawing extents exactly
      //      Draw a series of fat line segments approximating the arc using
      //      dc.DrawLine() This will properly establish the drawing box in the
      //      dc

      int border_fluff = 4;  // by how much should the blit bitmap be "fluffed"

      //  wxDC min/max calculations are currently broken in wxQT, so we use the
      //  entire circle instead of arcs...
#ifndef __WXQT__
      if (fabs(sectr2 - sectr1) != 360)  // not necessary for all-round lights
      {
        mdc.ResetBoundingBox();

        wxPen *pblockpen =
            wxThePenList->FindOrCreatePen(*wxBLACK, 10, wxPENSTYLE_SOLID);
        mdc.SetPen(*pblockpen);

        float start_angle, end_angle;
        if (se < sb) {
          start_angle = se;
          end_angle = sb;
        } else {
          start_angle = sb;
          end_angle = se;
        }

        int x0 = (width / 2) + (int)(rad * cos(start_angle * PI / 180.));
        int y0 = (height / 2) - (int)(rad * sin(start_angle * PI / 180.));
        for (float a = start_angle + .1; a <= end_angle; a += 2.0) {
          int x = (width / 2) + (int)(rad * cosf(a * PI / 180.));
          int y = (height / 2) - (int)(rad * sinf(a * PI / 180.));
          mdc.DrawLine(x0, y0, x, y);
          x0 = x;
          y0 = y;
        }

        bm_width = (mdc.MaxX() - mdc.MinX()) + (border_fluff * 2);
        bm_height = (mdc.MaxY() - mdc.MinY()) + (border_fluff * 2);
        bm_orgx = mdc.MinX() - border_fluff -
                  width / 2;  // wxMax ( 0, mdc.MinX()-border_fluff );
        bm_orgy = mdc.MinY() - border_fluff -
                  height / 2;  // wxMax ( 0, mdc.MinY()-border_fluff );

        mdc.Clear();
      }

      else {
        bm_width = rad * 2 + (border_fluff * 2);
        bm_height = rad * 2 + (border_fluff * 2);
        bm_orgx = -bm_width / 2;
        bm_orgy = -bm_height / 2;
      }

#else
      bm_width = rad * 2 + (border_fluff * 2);
      bm_height = rad * 2 + (border_fluff * 2);
      bm_orgx = -bm_width / 2;
      bm_orgy = -bm_height / 2;
#endif

      wxBitmap *sbm = NULL;

      //    Draw the outer border
      wxColour color = getwxColour(outline_color);

      wxPen *pthispen = wxThePenList->FindOrCreatePen(
          color, outline_width * scale_factor, wxPENSTYLE_SOLID);
      mdc.SetPen(*pthispen);
      wxBrush *pthisbrush =
          wxTheBrushList->FindOrCreateBrush(color, wxBRUSHSTYLE_TRANSPARENT);
      mdc.SetBrush(*pthisbrush);

      mdc.DrawEllipticArc(width / 2 - rad, height / 2 - rad, rad * 2, rad * 2,
                          sb, se);

      if (arc_width) {
        wxColour colorb = getwxColour(arc_color);

        if (!colorb.IsOk()) colorb = getwxColour(_T("CHMGD"));

        pthispen = wxThePenList->FindOrCreatePen(
            colorb, arc_width * scale_factor, wxPENSTYLE_SOLID);
        mdc.SetPen(*pthispen);

        mdc.DrawEllipticArc(width / 2 - rad, height / 2 - rad, rad * 2, rad * 2,
                            sb, se);
      }

      mdc.SelectObject(wxNullBitmap);

      //          Get smallest containing bitmap
      sbm = new wxBitmap(bm.GetSubBitmap(wxRect(
          width / 2 + bm_orgx, height / 2 + bm_orgy, bm_width, bm_height)));

      //      Make the mask
      wxMask *pmask = new wxMask(*sbm, m_unused_wxColor);

      //      Associate the mask with the bitmap
      sbm->SetMask(pmask);

      // delete any old private data
      ClearRulesCache(rules->razRule);

      //      Save the bitmap ptr and aux parms in the rule
      prule->pixelPtr = sbm;
      prule->parm0 = ID_wxBitmap;
      prule->parm1 = m_colortable_index;
      prule->parm2 = bm_orgx;
      prule->parm3 = bm_orgy;
      prule->parm5 = bm_width;
      prule->parm6 = bm_height;
      prule->parm7 = xscale;
    }  // instantiation
  }

  //  Render arcs at object's x/y
  wxPoint r;
  GetPointPixSingle(rzRules, rzRules->obj->y, rzRules->obj->x, &r);

  //      Now render the symbol
  if (!m_pdc)  // opengl
  {
  } else {
    int b_width = prule->parm5;
    int b_height = prule->parm6;

    //      Get the bitmap into a memory dc
    wxMemoryDC mdc;
    mdc.SelectObject((wxBitmap &)(*((wxBitmap *)(rules->razRule->pixelPtr))));

    //      Blit it into the target dc, using mask
    m_pdc->Blit(r.x + rules->razRule->parm2, r.y + rules->razRule->parm3,
                b_width, b_height, &mdc, 0, 0, wxCOPY, true);

    mdc.SelectObject(wxNullBitmap);

    //    Draw the sector legs directly on the target DC
    //    so that anti-aliasing works against the drawn image (cannot be
    //    cached...)
    if (sector_radius > 0) {
      int leg_len = (int)(sector_radius * canvas_pix_per_mm);

      wxDash dash1[2];
      dash1[0] = (int)(3.6 * canvas_pix_per_mm);  // 8// Long dash  <---------+
      dash1[1] = (int)(1.8 * canvas_pix_per_mm);  // 2// Short gap            |

      /*
       wxPen *pthispen = new wxPen(*wxBLACK_PEN);
       pthispen->SetStyle(wxPENSTYLE_USER_DASH);
       pthispen->SetDashes( 2, dash1 );
       //      Undocumented "feature":  Pen must be fully specified <<<BEFORE>>>
       setting into DC pdc->SetPen ( *pthispen );
       */
      // wxColour c = GetGlobalColor( _T ( "CHBLK" ) );
      wxColour c;
      GetGlobalColor(_T ( "CHBLK" ), &c);

      float a = (sectr1 - 90) * PI / 180;
      int x = r.x + (int)(leg_len * cosf(a));
      int y = r.y + (int)(leg_len * sinf(a));
      DrawAALine(m_pdc, r.x, r.y, x, y, c, dash1[0], dash1[1]);

      a = (sectr2 - 90) * PI / 180.;
      x = r.x + (int)(leg_len * cosf(a));
      y = r.y + (int)(leg_len * sinf(a));
      DrawAALine(m_pdc, r.x, r.y, x, y, c, dash1[0], dash1[1]);
    }

    // Debug the symbol bounding box.....
    /*
            if(m_pdc){
                m_pdc->SetPen(wxPen(*wxGREEN, 1));
                m_pdc->SetBrush(wxBrush(*wxGREEN, wxTRANSPARENT));
                m_pdc->DrawRectangle( r.x + rules->razRule->parm2, r.y +
       rules->razRule->parm3, b_width, b_height);
            }
            */
  }

  //  Update the object Bounding box,
  //  so that subsequent drawing operations will redraw the item fully

  double latmin, lonmin, latmax, lonmax;

  GetPixPointSingleNoRotate(r.x + prule->parm2,
                            r.y + prule->parm3 + prule->parm6, &latmin, &lonmin);
  GetPixPointSingleNoRotate(r.x + prule->parm2 + prule->parm5,
                            r.y + prule->parm3, &latmax, &lonmax);
  LLBBox symbox;
  symbox.Set(latmin, lonmin, latmax, lonmax);
  rzRules->obj->BBObj.Expand(symbox);
  return 1;
}
