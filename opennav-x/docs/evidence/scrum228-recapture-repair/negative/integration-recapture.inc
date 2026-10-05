void AfterFrameRecapture() {
#ifdef __WXGTK__
  if (shell && IsXNav()) shell->RestackChartControls();
#endif
}
