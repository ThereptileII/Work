  auto *overzoom = m_pParentCanvas->EmbossOverzoomIndicator(gldc);
  if (!overzoom || !opennav::integration::DrawChartOverzoomWarning(
          m_gldc, *m_pParentCanvas, overzoom->x, overzoom->y))
    DrawEmboss(m_gldc, overzoom);