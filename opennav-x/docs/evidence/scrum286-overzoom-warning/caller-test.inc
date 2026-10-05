  auto *overzoom = Invoke();
  if (!overzoom || !opennav::integration::DrawChartOverzoomWarning(
          dc, canvas, overzoom->x, overzoom->y))
    DrawEmboss(dc, overzoom);