void MyFrame::OnRecaptureTimer(wxTimerEvent &event) {
#ifdef OPENNAV_X
  // A late resize recapture must not cover an active owned sheet or card.
  if (opennav::HasXNavTransientSurface()) return;
#endif
  Raise();
#if defined(OPENNAV_X) && defined(__WXGTK__)
  // GTK's owner Raise can cover mapped owned chart controls until the next
  // shell tick. Restore their order within this same recapture callback.
  /* negative control: omit hook */
#endif
}
