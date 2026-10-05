namespace opennav::integration {
class OriginalFanTile {
 public:
  // At most 4 MiB RGBA plus bounded wx backing/copy storage. No per-Rule or
  // cross-frame cache, no full-canvas image and no cropped-away sector legs.
  static constexpr size_t kMaxPixels=1024*1024;
  wxImage image;
  wxRect bounds;
  bool Build(const CaFanGeometry& g,const CaFanPaint& paint) {
    image.Destroy();bounds={};
    if(!std::isfinite(g.radius) || !std::isfinite(g.start) || !std::isfinite(g.end) ||
       !std::isfinite(g.pixelScale) || g.radius<=0 || g.radius>32768 ||
       g.pixelScale<.5 || g.pixelScale>8 ||
       std::abs(g.start)>1440 || std::abs(g.end)>1440)return false;
    const auto coordinate=[](int n){return n>=-10000000 && n<=10000000;};
    if(!coordinate(g.center.x)||!coordinate(g.center.y)||
       !coordinate(g.leg1.x)||!coordinate(g.leg1.y)||
       !coordinate(g.leg2.x)||!coordinate(g.leg2.y))return false;
    double start=g.start,end=g.end;
    if(end<=start)end+=360;
    if(end-start<1 || end-start>=360)return false;
    const double padding=std::ceil(1.3*g.pixelScale)+2;
    const int left=int(std::floor(std::min({double(g.center.x)-g.radius,double(g.leg1.x),double(g.leg2.x)})-padding));
    const int top=int(std::floor(std::min({double(g.center.y)-g.radius,double(g.leg1.y),double(g.leg2.y)})-padding));
    const int right=int(std::ceil(std::max({double(g.center.x)+g.radius,double(g.leg1.x),double(g.leg2.x)})+padding));
    const int bottom=int(std::ceil(std::max({double(g.center.y)+g.radius,double(g.leg1.y),double(g.leg2.y)})+padding));
    const int w=right-left+1,h=bottom-top+1;
    if(w<=0 || h<=0 || w>2048 || h>2048 || size_t(w)*h>kMaxPixels)return false;
    try {
      wxImage built(w,h,true);if(!built.IsOk())return false;
      built.InitAlpha();std::memset(built.GetAlpha(),0,size_t(w)*h);
      // Obtain native antialias coverage with opaque white masks, then compose
      // straight RGBA ourselves. Drawing low-alpha RGB directly into wxImage
      // loses several RGB levels during Cairo's premultiply/unpremultiply.
      for(unsigned layer=0;layer<3;++layer) {
        wxImage mask(w,h,true);if(!mask.IsOk())return false;
        mask.InitAlpha();std::memset(mask.GetAlpha(),0,size_t(w)*h);
        {
          std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::Create(mask));
          if(!gc || !gc->SetCompositionMode(wxCOMPOSITION_OVER))return false;
          gc->SetAntialiasMode(wxANTIALIAS_DEFAULT);gc->Translate(-left,-top);
          const double pi=3.14159265358979323846;
          const double a=(start-90)*pi/180,b=(end-90)*pi/180;
          auto path=gc->CreatePath();
          if(layer==0) {
            path.MoveToPoint(g.center.x,g.center.y);
            path.AddLineToPoint(g.center.x+g.radius*std::cos(a),g.center.y+g.radius*std::sin(a));
            path.AddArc(g.center.x,g.center.y,g.radius,a,b,true);path.CloseSubpath();
            gc->SetBrush(*wxWHITE_BRUSH);gc->FillPath(path);
          } else {
            if(layer==1) {
              // One compound fill is the union of both butt-ended segments and
              // the SVG default miter join (limit 4). Native defaults differ.
              if(!Boundary(path,g))return false;
              gc->SetBrush(*wxWHITE_BRUSH);gc->FillPath(path,wxWINDING_RULE);
            } else {
              path.AddArc(g.center.x,g.center.y,g.radius,a,b,true);
              gc->SetPen(gc->CreatePen(wxGraphicsPenInfo(*wxWHITE).Width(1.2*g.pixelScale).Cap(wxCAP_BUTT)));
              gc->StrokePath(path);
            }
          }
        } // Native coverage is committed to mask alpha here.
        const wxColour color=layer?paint.stroke:paint.fill;
        const unsigned ink[]={color.Red(),color.Green(),color.Blue()};
        const double opacity=layer==0?.17:layer==1?.6:.8;
        auto* rgb=built.GetData();auto* alpha=built.GetAlpha();const auto* coverage=mask.GetAlpha();
        for(size_t i=0;i<size_t(w)*h;++i) {
          if(!coverage[i])continue;
          const double source=coverage[i]/255.0*opacity,behind=alpha[i]/255.0*(1-source),combined=source+behind;
          for(unsigned channel=0;channel<3;++channel)
            rgb[i*3+channel]=static_cast<unsigned char>(std::lround((ink[channel]*source+rgb[i*3+channel]*behind)/combined));
          alpha[i]=static_cast<unsigned char>(std::lround(combined*255));
        }
      }
      image=built;bounds=wxRect(left,top,w,h);return true;
    } catch(const std::bad_alloc&) {return false;}
  }
  static bool Boundary(wxGraphicsPath& path,const CaFanGeometry& g) {
    struct P {double x,y;};
    const double x1=g.center.x-g.leg1.x,y1=g.center.y-g.leg1.y;
    const double x2=g.leg2.x-g.center.x,y2=g.leg2.y-g.center.y;
    const double len1=std::hypot(x1,y1),len2=std::hypot(x2,y2),r=.325*g.pixelScale;
    if(len1<=0 || len2<=0)return false;
    const P n1{-y1/len1*r,x1/len1*r},n2{-y2/len2*r,x2/len2*r};
    const auto polygon=[&](std::vector<P> points) {
      double area=0;for(size_t i=0;i<points.size();++i){const auto& a=points[i];const auto& b=points[(i+1)%points.size()];area+=a.x*b.y-a.y*b.x;}
      if(area<0)std::reverse(points.begin(),points.end());
      path.MoveToPoint(points[0].x,points[0].y);
      for(size_t i=1;i<points.size();++i)path.AddLineToPoint(points[i].x,points[i].y);
      path.CloseSubpath();
    };
    polygon({{g.leg1.x+n1.x,g.leg1.y+n1.y},{g.center.x+n1.x,g.center.y+n1.y},
             {g.center.x-n1.x,g.center.y-n1.y},{g.leg1.x-n1.x,g.leg1.y-n1.y}});
    polygon({{g.center.x+n2.x,g.center.y+n2.y},{g.leg2.x+n2.x,g.leg2.y+n2.y},
             {g.leg2.x-n2.x,g.leg2.y-n2.y},{g.center.x-n2.x,g.center.y-n2.y}});
    const double cross=x1*y2-y1*x2;
    if(cross==0)return true;
    const double side=cross>0?-1:1;
    const P a{g.center.x+side*n1.x,g.center.y+side*n1.y};
    const P b{g.center.x+side*n2.x,g.center.y+side*n2.y};
    const double denominator=1+(x1*x2+y1*y2)/(len1*len2);
    if(denominator>0 && 2/denominator<=16)
      polygon({{double(g.center.x),double(g.center.y)},a,
               {g.center.x+side*(n1.x+n2.x)/denominator,g.center.y+side*(n1.y+n2.y)/denominator},b});
    else polygon({{double(g.center.x),double(g.center.y)},a,b});
    return true;
  }
  bool Draw(wxDC& dc) const {
    if(!image.IsOk())return false;
    wxBitmap bitmap(image);if(!bitmap.IsOk())return false;
    dc.DrawBitmap(bitmap,bounds.x,bounds.y,true);return true;
  }
}
;
}
