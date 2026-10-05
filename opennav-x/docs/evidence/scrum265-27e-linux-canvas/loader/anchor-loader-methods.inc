ChartSymbols::ChartSymbols(void) {}
ChartSymbols::~ChartSymbols(void) {}
void ChartSymbols::InitializeTables(void) {
  rasterSymbolsLoadedColorMapNumber = -1;
  ColorTableIndex = 0;
  rasterSymbolsTexture = 0;
}
void ChartSymbols::DeleteGlobals(void) {
  m_symbolGraphicLocations.clear();

  for (unsigned int i = 0; i < m_colorTables.GetCount(); i++) {
    colTable *ct = (colTable *)m_colorTables.Item(i);
    delete ct->tableName;
    ct->colors.clear();
    ct->wxColors.clear();
    delete ct;
  }

  m_colorTables.Clear();
}
void ChartSymbols::ProcessColorTables(pugi::xml_node &node) {
  for (pugi::xml_node child = node.first_child(); child != 0;
       child = child.next_sibling()) {
    const char *pcn = child.name();

    if (!strcmp(pcn, "color-table")) {
      colTable *colortable = new colTable;
      colortable->tableName =
          new wxString(child.first_attribute().value(), wxConvUTF8);

      pugi::xml_node colorNode = child.first_child();
      while (colorNode) {
        if (!strcmp(colorNode.name(), "graphics-file")) {
          colortable->rasterFileName =
              wxString(colorNode.first_attribute().value(), wxConvUTF8);
        }

        if (!strcmp(colorNode.name(), "color")) {
          wxString key;
          S52color color;

          for (pugi::xml_attribute attr = colorNode.first_attribute(); attr;
               attr = attr.next_attribute()) {
            const char *pca = attr.name();
            if (!strcmp(pca, "name")) {
              strncpy(color.colName, attr.value(), 5);
              color.colName[5] = 0;
              key = wxString(attr.value(), wxConvUTF8);

            } else if (!strcmp(pca, "r")) {
              color.R = attr.as_int();
            } else if (!strcmp(pca, "g")) {
              color.G = attr.as_int();
            } else if (!strcmp(pca, "b")) {
              color.B = attr.as_int();
            }
          }

          colortable->colors[key] = color;
          wxColour wxcolor(color.R, color.G, color.B);
          colortable->wxColors[key] = wxcolor;
        }

        colorNode = colorNode.next_sibling();
      }

      m_colorTables.Add((void *)colortable);
    }
  }
}
void ChartSymbols::ProcessSymbols(pugi::xml_node &node) {
  ChartSymbol symbol;

  for (pugi::xml_node child = node.first_child(); child != 0;
       child = child.next_sibling()) {
    symbol.RCID = child.attribute("RCID").as_int();

    symbol.hasVector = false;
    symbol.hasBitmap = false;
    symbol.preferBitmap = true;

    pugi::xml_node symbolNode = child.first_child();
    while (symbolNode) {
      const char *nodeText = symbolNode.first_child().value();
      const char *nodeType = symbolNode.name();

      if (!strcmp(nodeType, "description"))
        symbol.description = nodeText;
      else if (!strcmp(nodeType, "name"))
        symbol.name = nodeText;
      else if (!strcmp(nodeType, "definition"))
        symbol.hasVector = !strcmp(nodeText, "V");
      else if (!strcmp(nodeType, "color-ref"))
        symbol.colorRef = nodeText;

      else if (!strcmp(nodeType, "prefer-bitmap")) {
        if (!strcmp(nodeText, "no"))
          symbol.preferBitmap = false;
        else if (!strcmp(nodeText, "false"))
          symbol.preferBitmap = false;
      }

      else if (!strcmp(nodeType, "bitmap")) {
        symbol.bitmapSize.size.x = symbolNode.attribute("width").as_int();
        symbol.bitmapSize.size.y = symbolNode.attribute("height").as_int();
        symbol.hasBitmap = true;

        for (pugi::xml_node child = symbolNode.first_child(); child != 0;
             child = child.next_sibling()) {
          const char *nodeType = child.name();

          if (!strcmp(nodeType, "distance")) {
            symbol.bitmapSize.minDistance = child.attribute("min").as_int();
            symbol.bitmapSize.maxDistance = child.attribute("max").as_int();
          } else if (!strcmp(nodeType, "origin")) {
            symbol.bitmapSize.origin.x = child.attribute("x").as_int();
            symbol.bitmapSize.origin.y = child.attribute("y").as_int();
          } else if (!strcmp(nodeType, "pivot")) {
            symbol.bitmapSize.pivot.x = child.attribute("x").as_int();
            symbol.bitmapSize.pivot.y = child.attribute("y").as_int();
          } else if (!strcmp(nodeType, "graphics-location")) {
            symbol.bitmapSize.graphics.x = child.attribute("x").as_int();
            symbol.bitmapSize.graphics.y = child.attribute("y").as_int();
          }
        }
      }

      else if (!strcmp(nodeType, "vector")) {
        symbol.vectorSize.size.x = symbolNode.attribute("width").as_int();
        symbol.vectorSize.size.y = symbolNode.attribute("height").as_int();
        symbol.hasVector = true;

        for (pugi::xml_node child = symbolNode.first_child(); child != 0;
             child = child.next_sibling()) {
          const char *nodeType = child.name();

          if (!strcmp(nodeType, "distance")) {
            symbol.vectorSize.minDistance = child.attribute("min").as_int();
            symbol.vectorSize.maxDistance = child.attribute("max").as_int();
          } else if (!strcmp(nodeType, "origin")) {
            symbol.vectorSize.origin.x = child.attribute("x").as_int();
            symbol.vectorSize.origin.y = child.attribute("y").as_int();
          } else if (!strcmp(nodeType, "pivot")) {
            symbol.vectorSize.pivot.x = child.attribute("x").as_int();
            symbol.vectorSize.pivot.y = child.attribute("y").as_int();
          } else if (!strcmp(nodeType, "HPGL")) {
            symbol.HPGL = wxString(child.first_child().value(), wxConvUTF8);
          }
        }
      }

      symbolNode = symbolNode.next_sibling();
    }

    BuildSymbol(symbol);
  }
}
void ChartSymbols::BuildSymbol(ChartSymbol &symbol) {
  Rule *symb = (Rule *)calloc(1, sizeof(Rule));
  plib->pAlloc->Add(symb);

  wxString SVCT;
  wxString SCRF;

  symb->RCID = symbol.RCID;
  memcpy(symb->name.SYNM, symbol.name.char_str(), 8);

  symb->exposition.SXPO = new wxString(symbol.description);

  symb->vector.SVCT = (char *)malloc(symbol.HPGL.Len() + 1);
  strcpy(symb->vector.SVCT, symbol.HPGL.mb_str());

  symb->colRef.SCRF = (char *)malloc(symbol.colorRef.Len() + 1);
  strcpy(symb->colRef.SCRF, symbol.colorRef.mb_str());

  symb->bitmap.SBTM = NULL;

  SymbolSizeInfo_t symbolSize;

  if (symbol.hasVector && !(symbol.preferBitmap && symbol.hasBitmap)) {
    symb->definition.SYDF = 'V';
    symbolSize = symbol.vectorSize;
  } else {
    symb->definition.SYDF = 'R';
    symbolSize = symbol.bitmapSize;
  }

  symb->pos.symb.minDist.PAMI = symbolSize.minDistance;
  symb->pos.symb.maxDist.PAMA = symbolSize.maxDistance;

  symb->pos.symb.pivot_x.SYCL = symbolSize.pivot.x;
  symb->pos.symb.pivot_y.SYRW = symbolSize.pivot.y;

  symb->pos.symb.bnbox_w.SYHL = symbolSize.size.x;
  symb->pos.symb.bnbox_h.SYVL = symbolSize.size.y;

  symb->pos.symb.bnbox_x.SBXC = symbolSize.origin.x;
  symb->pos.symb.bnbox_y.SBXR = symbolSize.origin.y;

  wxRect graphicsLocation(symbol.bitmapSize.graphics, symbol.bitmapSize.size);
  m_symbolGraphicLocations[symbol.name] = graphicsLocation;

  // Already something here with same key? Then free its strings, otherwise they
  // leak.
  Rule *symbtmp = (*plib->_symb_sym)[symbol.name];
  if (symbtmp) {
    free(symbtmp->colRef.SCRF);
    free(symbtmp->vector.SVCT);
    delete symbtmp->exposition.SXPO;
  }

  (*plib->_symb_sym)[symbol.name] = symb;
}
int ChartSymbols::LoadRasterFileForColorTable(int tableNo, bool flush,
                                              const ChartCtx& ctx) {
  if (tableNo == rasterSymbolsLoadedColorMapNumber && !flush) {
    if (ctx.m_use_opengl) {
      if (rasterSymbolsTexture) return true;
#ifdef ocpnUSE_GL
      if (!ctx.m_texture_rectangle_format && rasterSymbols.IsOk())
        return true;
#endif
    }
    if (rasterSymbols.IsOk()) return true;
  }

  colTable *coltab = (colTable *)m_colorTables.Item(tableNo);

  wxString filename = configFileDirectory + wxFileName::GetPathSeparator() +
                      coltab->rasterFileName;

  wxImage rasterFileImg;
  if (rasterFileImg.LoadFile(filename, wxBITMAP_TYPE_PNG)) {
#ifdef ocpnUSE_GL
    /* for opengl mode, load the symbols into a texture */
    if (ctx.m_use_opengl && ctx.m_texture_rectangle_format) {
      int w = rasterFileImg.GetWidth();
      int h = rasterFileImg.GetHeight();

      //    Get the glRGBA format data from the wxImage
      unsigned char *d = rasterFileImg.GetData();
      unsigned char *a = rasterFileImg.GetAlpha();

      /* combine rgb with alpha */
      unsigned char *e = (unsigned char *)malloc(w * h * 4);
      if (d && a) {
        for (int y = 0; y < h; y++)
          for (int x = 0; x < w; x++) {
            int off = (y * w + x);

            e[off * 4 + 0] = d[off * 3 + 0];
            e[off * 4 + 1] = d[off * 3 + 1];
            e[off * 4 + 2] = d[off * 3 + 2];
            e[off * 4 + 3] = a[off];
          }
      }
      if (!rasterSymbolsTexture) glGenTextures(1, &rasterSymbolsTexture);

      glBindTexture(ctx.m_texture_rectangle_format, rasterSymbolsTexture);

      /* unfortunately this texture looks terrible with compression */
      GLuint format = GL_RGBA;
      glTexImage2D(ctx.m_texture_rectangle_format, 0, format, w, h, 0, GL_RGBA,
                   GL_UNSIGNED_BYTE, e);

      //             glTexParameteri( g_texture_rectangle_format,
      //             GL_TEXTURE_MAG_FILTER, GL_NEAREST ); glTexParameteri(
      //             g_texture_rectangle_format, GL_TEXTURE_MIN_FILTER,
      //             GL_NEAREST );

      glTexParameteri(ctx.m_texture_rectangle_format, GL_TEXTURE_WRAP_S,
                      GL_CLAMP_TO_EDGE);
      glTexParameteri(ctx.m_texture_rectangle_format, GL_TEXTURE_WRAP_T,
                      GL_CLAMP_TO_EDGE);

      glTexParameteri(ctx.m_texture_rectangle_format, GL_TEXTURE_MAG_FILTER,
                      GL_NEAREST);  // No mipmapping
      glTexParameteri(ctx.m_texture_rectangle_format, GL_TEXTURE_MIN_FILTER,
                      GL_NEAREST);

      rasterSymbolsTextureSize = wxSize(w, h);

      glDisable(GL_TEXTURE_2D);

      free(e);
    }
#endif
    { rasterSymbols = wxBitmap(rasterFileImg, -1 /*32*/); }

    rasterSymbolsLoadedColorMapNumber = tableNo;
    return true;
  }

  wxString msg(_T("ChartSymbols...Failed to load raster symbols file "));
  msg += filename;
  wxLogMessage(msg);
  return false;
}
int ChartSymbols::FindColorTable(const wxString &tableName) {
  for (unsigned int i = 0; i < m_colorTables.GetCount(); i++) {
    colTable *ct = (colTable *)m_colorTables.Item(i);
    if (tableName.IsSameAs(*ct->tableName)) {
      return i;
    }
  }
  return 0;
}
wxString ChartSymbols::HashKey(const char *symbolName) {
  char key[9];
  key[8] = 0;
  strncpy(key, symbolName, 8);
  return wxString(key, wxConvUTF8);
}
wxImage ChartSymbols::GetImage(const char *symbolName) {
  wxRect bmArea = m_symbolGraphicLocations[HashKey(symbolName)];
  if (rasterSymbols.IsOk()) {
    wxBitmap bitmap = rasterSymbols.GetSubBitmap(bmArea);
    return bitmap.ConvertToImage();
  } else
    return wxImage(1, 1);
}
unsigned int ChartSymbols::GetGLTextureRect(wxRect &rect,
                                            const char *symbolName) {
  rect = m_symbolGraphicLocations[HashKey(symbolName)];
  return rasterSymbolsTexture;
}
