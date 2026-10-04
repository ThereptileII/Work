emboss_data *ChartCanvas::EmbossOverzoomIndicator(ocpnDC &dc) {
  double zoom_factor = GetVP().ref_scale / GetVP().chart_scale;

  if (GetQuiltMode()) {
    // disable Overzoom indicator for MBTiles
    int refIndex = GetQuiltRefChartdbIndex();
    if (refIndex >= 0) {
      const ChartTableEntry &cte = ChartData->GetChartTableEntry(refIndex);
      ChartTypeEnum current_type = (ChartTypeEnum)cte.GetChartType();
      if (current_type == CHART_TYPE_MBTILES) {
        ChartBase *pChart = m_pQuilt->GetRefChart();
        ChartMbTiles *ptc = dynamic_cast<ChartMbTiles *>(pChart);
        if (ptc) {
          zoom_factor = ptc->GetZoomFactor();
        }
      }
    }

    if (zoom_factor <= 3.9) return NULL;
  } else {
    if (m_singleChart) {
      if (zoom_factor <= 3.9) return NULL;
    } else
      return NULL;
  }

  if (m_pEM_OverZoom) {
    m_pEM_OverZoom->x = 4;
    m_pEM_OverZoom->y = 0;
    if (g_MainToolbar && IsPrimaryCanvas()) {
      wxRect masterToolbarRect = g_MainToolbar->GetToolbarRect();
      m_pEM_OverZoom->x = masterToolbarRect.width + 4;
    }
  }
  return m_pEM_OverZoom;
}
