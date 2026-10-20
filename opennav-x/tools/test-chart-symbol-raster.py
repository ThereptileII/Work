#!/usr/bin/env python3
"""Native wx raster proof through actual prepared core/private renderer methods.

No application/plugin is built or launched. The generated resource header and
prepared source must be supplied explicitly; native Windows/GL remain separate.
"""
import argparse
from pathlib import Path
import os
import shlex
import subprocess


def method(text, signature):
    start = text.index(signature)
    brace = text.index("{", start)
    depth = 0
    for end in range(brace, len(text)):
        depth += (text[end] == "{") - (text[end] == "}")
        if not depth:
            return text[start:end + 1]
    raise ValueError("Unclosed method " + signature)


STUB = r'''
#include <cstring>
#include <cmath>
#define GEO_POINT 1
#define GEO_AREA 2
double g_scaminScale=1;
double GetOCPNChartScaleFactor_Plugin() { return 1.; }
struct Box {
 double x=0,y=0,width=0,height=0;
 void Set(double a,double b,double c,double d) { x=b;y=-c;width=d-b;height=c-a; }
 bool IntersectIn(const Box&) const { return true; }
 void Expand(const Box& other) { *this=other; }
};
using LLBBox=Box;
struct Object { char FeatureName[7]{}; int Primitive_type=GEO_POINT; Box BBObj; };
struct ObjRazRules { Object* obj; };
struct Rule {
 struct { char SYNM[9]{}; } name;
 struct { struct { struct { int SYCL=0; } pivot_x;
                         struct { int SYRW=0; } pivot_y; } line; } pos;
};
struct VPointCompat {
 int pix_width=200,pix_height=200;
 double rotation=0,view_scale_ppm=1;
};
void PrepareS52ShaderUniforms(VPointCompat*) {}
struct FontCache { void PrepareShader(int,int,double) {} };
constexpr unsigned TXF_CACHE=1;
struct { FontCache* cache=nullptr; } s_txf[TXF_CACHE];
struct S52color { unsigned char R,G,B; };
struct ChartSymbols {
 unsigned GetGLTextureRect(wxRect& rect,const char* name) {
   const auto* svg=opennav::integration::ChartSymbolSvg(true,name,"DAY");
   rect=svg?wxRect(0,0,svg->width,svg->height):wxRect(0,0,24,28);return 0;
 }
 wxImage GetImage(const char*) { return wxImage(); }
};
class s52plib {
 public:
 bool m_presentationLightSymbols=true,m_presentationRasterSubmitted=false;
 bool m_presentationVeryLight=false,m_benableGLLS=false;
 bool m_useGLSL=false;
 double m_presentationDisplayScale=1,m_ChartScaleFactorExp=1;
 double m_SoundingsScaleFactor=1,m_TextScaleFactor=1,m_dipfactor=1;
 double m_display_size_mm=1000,m_ContentScaleFactor=1,lastLightLat=0,lastLightLon=0;
 int m_nSoundingFactor=0,m_nTextFactor=0,hashChanges=0;
 wxString m_ColorScheme="DAY";
 opennav::integration::ChartSymbolRasterCache m_presentationSymbolRasters;
 ChartSymbols m_chartSymbols;
 VPointCompat vp_plib;
 wxDC* m_pdc=nullptr;
 void ClearTextList() {}
 void GenerateStateHash() { ++hashChanges; }
 double GetPPMM() { return 4.; }
 S52color* getColor(const char*) { static S52color c{54,66,65};return &c; }
 void GetPixPointSingle(double x,double y,double* lat,double* lon) {
   *lat=-y;*lon=x;
 }
 void SetScaleFactorExp(double);
 void ConfigurePresentationDisplay(bool,int);
 void PrepareForRender(VPointCompat*);
 bool RenderPresentationSymbol(ObjRazRules*,Rule*,const wxPoint&,double);
 bool RenderRasterSymbol(ObjRazRules*,Rule*,wxPoint&,float);
};
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--generated", type=Path, required=True)
    parser.add_argument("--wx-prefix", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--private", action="store_true")
    parser.add_argument("--cxx", default="c++")
    parser.add_argument("--negative-control", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source = (args.source / "libs/s52plib/src/s52plib.cpp").read_text()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    methods = [
        method(source, "void s52plib::SetScaleFactorExp("),
        method(source, "void s52plib::ConfigurePresentationDisplay("),
        method(source, "void s52plib::PrepareForRender(VPointCompat *vp)"),
        method(source, "bool s52plib::RenderPresentationSymbol("),
    ]
    # Execute the actual entry through native size/SCAMIN/DIP logic and the new
    # owned dispatch. The untouched stock fallback is outside this component.
    raster = method(source, "bool s52plib::RenderRasterSymbol(")
    prefix = raster[:raster.index("  int pivot_x =")]
    assert "RenderPresentationSymbol(rzRules, prule, r, scale_factor)" in prefix
    methods.append(prefix + "  return false;\n}")
    snippet = STUB + "\n" + "\n".join(methods) + "\n"
    if args.negative_control:
        old = "m_ChartScaleFactorExp = GetOCPNChartScaleFactor_Plugin() * m_presentationDisplayScale;"
        assert snippet.count(old) == 1
        snippet = snippet.replace(old, "m_ChartScaleFactorExp = GetOCPNChartScaleFactor_Plugin();")
    (output / "actual-chart-symbol-display.inc").write_text(snippet)
    wx = [str(args.wx_prefix / "bin/wx-config"),
          "--prefix=" + str(args.wx_prefix)]
    flags = shlex.split(subprocess.check_output(wx + ["--cxxflags", "--libs"], text=True))
    exe = output / "chart-symbol-raster"
    command = [args.cxx, "-std=c++17", "-Wall", "-Wextra",
               "-DSKAGER_OCHARTS_ADAPTER" if args.private else "-DOPENNAV_X",
               "-I" + str(root / "src"), "-I" + str(args.generated.resolve()),
               "-I" + str(output), str(root / "tests/chart_symbol_raster_test.cpp"),
               *flags, "-Wl,-rpath," + str(args.wx_prefix / "lib"), "-o", str(exe)]
    compiled = subprocess.run(command, capture_output=True, text=True)
    (output / "compile.log").write_text(compiled.stdout + compiled.stderr)
    if compiled.returncode:
        print(compiled.stderr)
        compiled.check_returncode()
    result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=30,
        env={**os.environ, "LD_LIBRARY_PATH": str(args.wx_prefix / "lib")})
    (output / "checks.log").write_text(result.stdout + result.stderr)
    print(result.stdout + result.stderr, end="")
    if args.negative_control:
        assert result.returncode == 1 and "actual Prepare retains display scale" in result.stderr
        print("Negative control: Prepare dropping display percentage rejected")
    else:
        result.check_returncode()


if __name__ == "__main__":
    main()
