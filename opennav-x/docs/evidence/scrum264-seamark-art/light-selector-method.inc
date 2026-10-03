wxString _selSYcol(char *buf, bool bsectr, double valnmr) {
  wxString sym;

  if (!bsectr) {
    sym = _T(";SY(LITDEF11");  // default

    // max 1 color
    if ('\0' == buf[1]) {
      if (strpbrk(buf, "\003"))
        sym = _T(";SY(LIGHTS11");
      else if (strpbrk(buf, "\004"))
        sym = _T(";SY(LIGHTS12");
      else if (strpbrk(buf, "\001\006\011"))
        sym = _T(";SY(LIGHTS13");
    } else {
      // max 2 color
      if ('\0' == buf[2]) {
        if (strpbrk(buf, "\001") && strpbrk(buf, "\003"))
          sym = _T(";SY(LIGHTS11");
        else if (strpbrk(buf, "\001") && strpbrk(buf, "\004"))
          sym = _T(";SY(LIGHTS12");
      }
    }
  }

  else  // all-round fixed light symbolized as a circle, radius depends on color
        // This treatment is seen on SeeMyDenc by SevenCs
        // This may not be S-52 compliant....
  {
    //      Another non-standard extension....
    //      All round light circle diameter is scaled if the light has a
    //      reasonable VALNMR attribute
    int radius = 3;
    if (valnmr > 0) {
      if (valnmr < 7.0)
        radius = 3;
      else if (valnmr < 15.0)
        radius = 10;
      else if (valnmr < 30.0)
        radius = 15;
      else
        radius = 20;
    }

    // max 1 color
    if ('\0' == buf[1]) {
      if (strpbrk(buf, "\003"))
        sym.Printf(_T(",LITRD, 2,0,360,%d,0"), radius + 1);
      else if (strpbrk(buf, "\004"))
        sym.Printf(_T(",LITGN, 2,0,360,%d,0"), radius);
      else if (strpbrk(buf, "\001\006\011"))
        sym.Printf(_T(",LITYW, 2,0,360,%d,0"), radius + 2);
      else if (strpbrk(buf, "\014"))
        sym.Printf(_T(",CHMGD, 2,0,360,%d,0"), radius + 3);
      else
        sym.Printf(_T(",CHMGD, 2,0,360,%d,0"), radius + 5);  // default

    } else if ('\0' == buf[2])  // or 2 color
    {
      if (strpbrk(buf, "\001") && strpbrk(buf, "\003"))
        sym.Printf(_T(",LITRD, 2,0,360,%d,0"), radius + 1);
      else if (strpbrk(buf, "\001") && strpbrk(buf, "\004"))
        sym.Printf(_T(",LITGN, 2,0,360,%d,0"), radius);
      else
        sym.Printf(_T(",CHMGD, 2,0,360,%d,0"), radius + 5);  // default

    } else
      sym.Printf(_T(",CHMGD, 2,0,360,%d,0"), radius + 5);

    if (sym.Len()) sym.Prepend(_T(";CA(OUTLW, 4"));
  }

  return sym;
}
