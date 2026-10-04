void ocpnDC::SetGLStipple() const {
#ifdef ocpnUSE_GL

#if !defined(USE_ANDROID_GLES2) && !defined(ocpnUSE_GLSL)
  switch (m_pen.GetStyle()) {
    case wxPENSTYLE_DOT: {
      glLineStipple(1, 0x3333);
      glEnable(GL_LINE_STIPPLE);
      break;
    }
    case wxPENSTYLE_LONG_DASH: {
      glLineStipple(1, 0xFFF8);
      glEnable(GL_LINE_STIPPLE);
      break;
    }
    case wxPENSTYLE_SHORT_DASH: {
      glLineStipple(1, 0x3F3F);
      glEnable(GL_LINE_STIPPLE);
      break;
    }
    case wxPENSTYLE_DOT_DASH: {
      glLineStipple(1, 0x8FF1);
      glEnable(GL_LINE_STIPPLE);
      break;
    }
    default:
      break;
  }
#endif
#endif
}

void ocpnDC::SetPen(const wxPen &pen) {
  if (dc) {
    if (pen == wxNullPen)
      dc->SetPen(*wxTRANSPARENT_PEN);
    else
      dc->SetPen(pen);
  } else
    m_pen = pen;
}

void ocpnDC::SetBrush(const wxBrush &brush) {
  if (dc)
    dc->SetBrush(brush);
  else
    m_brush = brush;
}

void ocpnDC::SetFont(const wxFont &font) {
  if (dc)
    dc->SetFont(font);
  else
    m_font = font;
}

void ocpnDC::SetTextForeground(const wxColour &colour) {
  if (dc)
    dc->SetTextForeground(colour);
  else
    m_textforegroundcolour = colour;
}

const wxPen &ocpnDC::GetPen() const {
  if (dc) return dc->GetPen();
  return m_pen;
}

const wxBrush &ocpnDC::GetBrush() const {
  if (dc) return dc->GetBrush();
  return m_brush;
}

const wxFont &ocpnDC::GetFont() const {
  if (dc) return dc->GetFont();
  return m_font;
}

bool ocpnDC::ConfigurePen() {
  if (!m_pen.IsOk()) return false;
  if (m_pen == *wxTRANSPARENT_PEN) return false;

  wxColour c = m_pen.GetColour();
  int width = m_pen.GetWidth();
#ifdef ocpnUSE_GL
#if !defined(USE_ANDROID_GLES2) && !defined(ocpnUSE_GLSL)
  glColor4ub(c.Red(), c.Green(), c.Blue(), c.Alpha());
#endif
  glLineWidth(wxMax(g_GLMinSymbolLineWidth, width));
#endif
  return true;
}

bool ocpnDC::ConfigureBrush() {
  if (m_brush == wxNullBrush || m_brush.GetStyle() == wxBRUSHSTYLE_TRANSPARENT)
    return false;
#ifdef ocpnUSE_GL
  wxColour c = m_brush.GetColour();
#if !defined(USE_ANDROID_GLES2) && !defined(ocpnUSE_GLSL)
  glColor4ub(c.Red(), c.Green(), c.Blue(), c.Alpha());
#endif
#endif
  return true;
}

void ocpnDC::drawrrhelperGLES2(wxCoord x0, wxCoord y0, wxCoord r, int quadrant,
                               int steps) {
#ifdef ocpnUSE_GL
  if (steps == 0) return;

  float step = 1.0 / steps, rs = 2.0 * r * step, rss = rs * step, x, y, dx, dy,
        ddx, ddy;
  switch (quadrant) {
    case 0:
      x = r, y = 0, dx = 0, dy = -rs, ddx = -rss, ddy = rss;
      break;
    case 1:
      x = 0, y = -r, dx = -rs, dy = 0, ddx = rss, ddy = rss;
      break;
    case 2:
      x = -r, y = 0, dx = 0, dy = rs, ddx = rss, ddy = -rss;
      break;
    case 3:
      x = 0, y = r, dx = rs, dy = 0, ddx = -rss, ddy = -rss;
      break;
    default:
      return;  // avoid unitialized compiler warnings
  }

  for (int i = 0; i < steps; i++) {
    workBuf[workBufIndex++] = x0 + floor(x);
    workBuf[workBufIndex++] = y0 + floor(y);

    x += dx + ddx / 2, y += dy + ddy / 2;
    dx += ddx, dy += ddy;
  }

  workBuf[workBufIndex++] = x0 + floor(x);
  workBuf[workBufIndex++] = y0 + floor(y);
#endif
}

void ocpnDC::DrawRoundedRectangle(wxCoord x, wxCoord y, wxCoord w, wxCoord h,
                                  wxCoord r) {
  if (dc) dc->DrawRoundedRectangle(x, y, w, h, r);
#ifdef ocpnUSE_GL
  else {
    if (!m_glchartCanvas) return;

    r++;
    int steps = ceil(sqrt((float)r));

    wxCoord x1 = x + r, x2 = x + w - r;
    wxCoord y1 = y + r, y2 = y + h - r;

    ConfigureBrush();
    ConfigurePen();

    //  Grow the work buffer as necessary
    size_t bufReq = (steps + 1) * 8 * 2;  // large, to be sure

    if (workBufSize < bufReq) {
      workBuf = (float *)realloc(workBuf, bufReq * sizeof(float));
      workBufSize = bufReq;
    }
    workBufIndex = 0;

    if (steps) {
      drawrrhelperGLES2(x2, y1, r, 0, steps);
      drawrrhelperGLES2(x1, y1, r, 1, steps);
      drawrrhelperGLES2(x1, y2, r, 2, steps);
      drawrrhelperGLES2(x2, y2, r, 3, steps);
    }

    GLShaderProgram *shader = pcolor_tri_shader_program[m_canvasIndex];
    shader->Bind();

    float fcolorv[4];
    fcolorv[0] = m_brush.GetColour().Red() / float(256);
    fcolorv[1] = m_brush.GetColour().Green() / float(256);
    fcolorv[2] = m_brush.GetColour().Blue() / float(256);
    fcolorv[3] = m_brush.GetColour().Alpha() / float(256);
    shader->SetUniform4fv("color", fcolorv);

    float angle = 0.;
    float xoffset = 0;
    float yoffset = 0;

    // Rotate
    mat4x4 I, Q;
    mat4x4_identity(I);
    mat4x4_rotate_Z(Q, I, angle);

    // Translate
    Q[3][0] = xoffset;
    Q[3][1] = yoffset;

    mat4x4 X;
    mat4x4_mul(X,
               (float(*)[4])m_glchartCanvas->m_pParentCanvas->GetpVP()
                   ->vp_matrix_transform,
               Q);
    shader->SetUniformMatrix4fv("MVMatrix", (GLfloat *)X);

    shader->SetAttributePointerf("position", workBuf);

    // Perform the actual drawing.
    glDrawArrays(GL_TRIANGLE_FAN, 0, workBufIndex / 2);

    // Border color
    float bcolorv[4];
    bcolorv[0] = m_pen.GetColour().Red() / float(256);
    bcolorv[1] = m_pen.GetColour().Green() / float(256);
    bcolorv[2] = m_pen.GetColour().Blue() / float(256);
    bcolorv[3] = m_pen.GetColour().Alpha() / float(256);

    shader->SetUniform4fv("color", bcolorv);

    // Perform the actual drawing.
    glDrawArrays(GL_LINE_LOOP, 0, workBufIndex / 2);

    shader->UnBind();
  }
#endif
}

void ocpnDC::DrawLine(wxCoord x1, wxCoord y1, wxCoord x2, wxCoord y2,
                      bool b_hiqual) {
  if (dc) dc->DrawLine(x1, y1, x2, y2);
#ifdef ocpnUSE_GL
  else if (ConfigurePen()) {
    bool b_draw_thick = false;

    float pen_width = wxMax(g_GLMinSymbolLineWidth, m_pen.GetWidth());

    //      Enable anti-aliased lines, at best quality
    if (b_hiqual) {
      SetGLStipple();

#ifndef __WXQT__
      glEnable(GL_BLEND);
      if (g_GLOptions.m_GLLineSmoothing) glEnable(GL_LINE_SMOOTH);
#endif

      if (pen_width > 1.0) {
        GLint parms[2];
        glGetIntegerv(GL_SMOOTH_LINE_WIDTH_RANGE, &parms[0]);
        if (glGetError()) glGetIntegerv(GL_ALIASED_LINE_WIDTH_RANGE, &parms[0]);
        if (pen_width > parms[1])
          b_draw_thick = true;
        else
          glLineWidth(pen_width);
      } else
        glLineWidth(pen_width);
    } else {
      if (pen_width > 1) {
        GLint parms[2];
        glGetIntegerv(GL_ALIASED_LINE_WIDTH_RANGE, &parms[0]);
        if (pen_width > parms[1])
          b_draw_thick = true;
        else
          glLineWidth(pen_width);
      } else
        glLineWidth(pen_width);
    }

#if defined(USE_ANDROID_GLES2) || defined(ocpnUSE_GLSL)
    if (b_draw_thick)
      DrawGLThickLine(x1, y1, x2, y2, m_pen, b_hiqual);
    else {
#if 0  // Use AA lines

      GLShaderProgram *shader = pAALine_shader_program[m_canvasIndex];
      shader->Bind();

      // Assuming here that transform matrix for this shader is preset for canvas.
      shader->SetUniformMatrix4fv("MVMatrix", (GLfloat *)m_glchartCanvas->m_pParentCanvas->GetpVP()->vp_matrix_transform);

      float vpx[2];
      int width = 0;
      int height = 0;
      GetSize(&width, &height);
      vpx[0] = width;
      vpx[1] = height;

      shader->SetUniform2fv("uViewPort", vpx);

      float colorv[4];
      colorv[0] = m_pen.GetColour().Red() / float(256);
      colorv[1] = m_pen.GetColour().Green() / float(256);
      colorv[2] = m_pen.GetColour().Blue() / float(256);
      colorv[3] = 1.0;

      shader->SetUniform4fv("color", colorv);

      float fBuf[4];
      shader->SetAttributePointerf("position", fBuf);


      wxDash *dashes;
      int n_dashes = m_pen.GetDashes(&dashes);
      if (n_dashes) {
        float angle = atan2f((float)(y2 - y1), (float)(x2 - x1));
        float cosa = cosf(angle);
        float sina = sinf(angle);
        float t1 = m_pen.GetWidth();

        float lpix = sqrtf(powf(x1 - x2, 2) + powf(y1 - y2, 2));
        float lrun = 0.;
        float xa = x1;
        float ya = y1;
        float ldraw = t1 * dashes[0];
        float lspace = t1 * dashes[1];

        ldraw = wxMax(ldraw, 4.0);
        lspace = wxMax(lspace, 4.0);
        lpix = wxMin(lpix, 2000.0);

        while (lrun < lpix) {
          //    Dash
          float xb = xa + ldraw * cosa;
          float yb = ya + ldraw * sina;

          if ((lrun + ldraw) >= lpix)  // last segment is partial draw
          {
            xb = x2;
            yb = y2;
          }

          fBuf[0] = xa;
          fBuf[1] = ya;
          fBuf[2] = xb;
          fBuf[3] = yb;

          glDrawArrays(GL_LINES, 0, 2);

          xa = xa + (lspace + ldraw) * cosa;
          ya = ya + (lspace + ldraw) * sina;
          lrun += lspace + ldraw;
        }
      } else  // not dashed
      {
        fBuf[0] = x1;
        fBuf[1] = y1;
        fBuf[2] = x2;
        fBuf[3] = y2;

        glDrawArrays(GL_LINES, 0, 2);
      }
      shader->UnBind();
#else

      GLShaderProgram *shader;
      if (m_glchartCanvas) {
        shader = pcolor_tri_shader_program[m_canvasIndex];
        shader->Bind();
        // Assuming here that transform matrix for this shader is preset for
        // canvas.
        // shader->SetUniformMatrix4fv("MVMatrix", (GLfloat
        // *)m_glchartCanvas->m_pParentCanvas->GetpVP()->vp_matrix_transform);
      } else {
        shader = m_pcolor_tri_shader_program;
        shader->Bind();
        shader->SetUniformMatrix4fv("MVMatrix",
                                    (float *)&(m_vp.vp_matrix_transform));
      }

      float colorv[4];
      colorv[0] = m_pen.GetColour().Red() / float(256);
      colorv[1] = m_pen.GetColour().Green() / float(256);
      colorv[2] = m_pen.GetColour().Blue() / float(256);
      colorv[3] = 1.0;

      shader->SetUniform4fv("color", colorv);

      glLineWidth(wxMax(g_GLMinSymbolLineWidth, m_pen.GetWidth()));

      float fBuf[4];
      shader->SetAttributePointerf("position", fBuf);

      wxDash *dashes;
      int n_dashes = m_pen.GetDashes(&dashes);
      if (n_dashes) {
        float angle = atan2f((float)(y2 - y1), (float)(x2 - x1));
        float cosa = cosf(angle);
        float sina = sinf(angle);
        float t1 = m_pen.GetWidth();

        float lpix = sqrtf(powf(x1 - x2, 2) + powf(y1 - y2, 2));
        float lrun = 0.;
        float xa = x1;
        float ya = y1;
        float ldraw = t1 * dashes[0];
        float lspace = t1 * dashes[1];

        ldraw = wxMax(ldraw, 4.0);
        lspace = wxMax(lspace, 4.0);
        // lpix = wxMin(lpix, 2000.0);

        while (lrun < lpix) {
          //    Dash
          float xb = xa + ldraw * cosa;
          float yb = ya + ldraw * sina;

          if ((lrun + ldraw) >= lpix)  // last segment is partial draw
          {
            xb = x2;
            yb = y2;
          }

          fBuf[0] = xa;
          fBuf[1] = ya;
          fBuf[2] = xb;
          fBuf[3] = yb;

          glDrawArrays(GL_LINES, 0, 2);

          xa = xa + (lspace + ldraw) * cosa;
          ya = ya + (lspace + ldraw) * sina;
          lrun += lspace + ldraw;
        }
      } else  // not dashed
      {
        fBuf[0] = x1;
        fBuf[1] = y1;
        fBuf[2] = x2;
        fBuf[3] = y2;

        glDrawArrays(GL_LINES, 0, 2);
      }
      shader->UnBind();

#endif
    }

#else
#endif
    glDisable(GL_LINE_STIPPLE);

    if (b_hiqual) {
      glDisable(GL_LINE_SMOOTH);
      glDisable(GL_BLEND);
    }
  }
#endif
}

void ocpnDC::DrawText(const wxString &text, wxCoord x, wxCoord y, float angle) {
  if (dc) dc->DrawText(text, x, y);
#ifdef ocpnUSE_GL
  else {
    wxCoord w = 0;
    wxCoord h = 0;

    // FIXME Dave Re-enable, and fix rotation logic.
    if (0 /*m_buseTex*/) {
      m_texfont.Build(m_font, 1.0,
                      m_dpi_factor);  // make sure the font is ready
      m_texfont.GetTextExtent(text, &w, &h);
      m_texfont.SetColor(m_textforegroundcolour);

      if (w && h) {
        glEnable(GL_BLEND);
        glEnable(GL_TEXTURE_2D);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        m_texfont.RenderString(text, x, y, angle);

        glDisable(GL_TEXTURE_2D);
        glDisable(GL_BLEND);
      }
    } else {
      wxScreenDC sdc;
      sdc.SetFont(m_font);
      sdc.GetTextExtent(text, &w, &h, NULL, NULL, &m_font);
      if (w && h) {
        /* create bitmap of appropriate size and select it */
        wxBitmap bmp(w, h);
        wxMemoryDC temp_dc;
        temp_dc.SelectObject(bmp);

        /* fill bitmap with black */
        temp_dc.SetBackground(wxBrush(wxColour(0, 0, 0)));
        temp_dc.Clear();

        /* draw the text white */
        temp_dc.SetFont(m_font);
        temp_dc.SetTextForeground(wxColour(255, 255, 255));
        temp_dc.DrawText(text, 0, 0);
        temp_dc.SelectObject(wxNullBitmap);

        /* use the data in the bitmap for alpha channel,
        and set the color to text foreground */
        wxImage image = bmp.ConvertToImage();
        if (x < 0 ||
            y < 0) {  // Allow Drawing text which is offset to start off screen
          int dx = (x < 0 ? -x : 0);
          int dy = (y < 0 ? -y : 0);
          w = bmp.GetWidth() - dx;
          h = bmp.GetHeight() - dy;
          /* picture is out of viewport */
          if (w <= 0 || h <= 0) return;
          image = image.GetSubImage(wxRect(dx, dy, w, h));
          x += dx;
          y += dy;
        }

        unsigned char *data = new unsigned char[w * h * 4];
        unsigned char *im = image.GetData();

        if (im) {
          unsigned int r = m_textforegroundcolour.Red();
          unsigned int g = m_textforegroundcolour.Green();
          unsigned int b = m_textforegroundcolour.Blue();
          for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
              unsigned int index = ((i * w) + j) * 4;
              data[index] = r;
              data[index + 1] = g;
              data[index + 2] = b;
              data[index + 3] = im[((i * w) + j) * 3];
            }
          }
        }

        unsigned int texobj;

        glGenTextures(1, &texobj);
        glBindTexture(GL_TEXTURE_2D, texobj);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

        int TextureWidth = NextPow2(w);
        int TextureHeight = NextPow2(h);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, TextureWidth, TextureHeight, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE,
                        data);

        glEnable(GL_TEXTURE_2D);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        float u = (float)w / TextureWidth, v = (float)h / TextureHeight;

        float uv[8];
        float coords[8];

        // normal uv
        uv[0] = 0;
        uv[1] = 0;
        uv[2] = u;
        uv[3] = 0;
        uv[4] = u;
        uv[5] = v;
        uv[6] = 0;
        uv[7] = v;

        // pixels
        coords[0] = 0;
        coords[1] = 0;
        coords[2] = w;
        coords[3] = 0;
        coords[4] = w;
        coords[5] = h;
        coords[6] = 0;
        coords[7] = h;

        // Set up the shader
        GLShaderProgram *shader;
        if (m_glchartCanvas) {
          shader = ptexture_2D_shader_program[m_canvasIndex];
          shader->Bind();
          // Assuming here that transform matrix for this shader is preset for
          // canvas.
          // shader->SetUniformMatrix4fv("MVMatrix", (GLfloat
          // *)m_glchartCanvas->m_pParentCanvas->GetpVP()->vp_matrix_transform);
        } else {
          shader = m_ptexture_2D_shader_program;
          shader->Bind();
          shader->SetUniformMatrix4fv("MVMatrix",
                                      (float *)&(m_vp.vp_matrix_transform));
        }

        // Set up the texture sampler to texture unit 0
        shader->SetUniform1i("uTex", 0);

        // Rotate
        mat4x4 I, Q;
        mat4x4_identity(I);
        mat4x4_rotate_Z(Q, I, 0);

        // Translate
        Q[3][0] = x;
        Q[3][1] = y;

        shader->SetUniformMatrix4fv("TransformMatrix", (GLfloat *)Q);

        float co1[8];
        float tco1[8];

        // Perform the actual drawing.

        // For some reason, glDrawElements is busted on Android
        // So we do this a hard ugly way, drawing two triangles...
        co1[0] = coords[0];
        co1[1] = coords[1];
        co1[2] = coords[2];
        co1[3] = coords[3];
        co1[4] = coords[6];
        co1[5] = coords[7];
        co1[6] = coords[4];
        co1[7] = coords[5];

        tco1[0] = uv[0];
        tco1[1] = uv[1];
        tco1[2] = uv[2];
        tco1[3] = uv[3];
        tco1[4] = uv[6];
        tco1[5] = uv[7];
        tco1[6] = uv[4];
        tco1[7] = uv[5];

        shader->SetAttributePointerf("aPos", co1);
        shader->SetAttributePointerf("aUV", tco1);

        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

        shader->UnBind();

        glDisable(GL_BLEND);
        glDisable(GL_TEXTURE_2D);

        glDeleteTextures(1, &texobj);
        delete[] data;
      }
    }
  }
#endif
}
