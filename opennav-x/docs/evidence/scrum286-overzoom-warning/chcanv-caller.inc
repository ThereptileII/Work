  auto *overzoom = EmbossOverzoomIndicator(dc);
  if (!overzoom || !opennav::integration::DrawChartOverzoomWarning(
          dc, *this, overzoom->x, overzoom->y))
    DrawEmboss(dc, overzoom);