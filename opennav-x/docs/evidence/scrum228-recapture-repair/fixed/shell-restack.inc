void Shell::RestackChartControls() {
  for (auto *overlay : chart_overlays_)
    static_cast<XNavFloatingSurface *>(overlay)->RestackAboveOwner();
}
