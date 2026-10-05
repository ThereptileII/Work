#include "integration/OnboardAisPresentation.h"
#include "integration/OnboardAisPaint.h"
#include "integration/ChartPresentation.h"
#include "ui/Controls.h"
#include "model/ais_target_data.h"
#include "chcanv.h"
#include "ocpndc.h"
#include <wx/thread.h>
#ifdef ocpnUSE_GL
#include "shaders.h"
#endif

namespace opennav::integration {
static_assert(AIS_CLASS_A == 0 && AIS_CLASS_B == 1 && AIS_NO_ALERT == 0 &&
              UNDERWAY_USING_ENGINE == 0 && UNDERWAY_SAILING == 8 && UNDEFINED == 15,
              "Review copied AIS appearance policy when upstream identities change");
bool DrawChartOnboardAis(ocpnDC &dc, ChartCanvas &canvas,
                        const OnboardAisAppearance &appearance,
                        double x, double y, double north_angle,
                        double user_scale, int attenuation) {
  wxColour land, water;
  if (!wxIsMainThread() || !UseOnboardAisBody(appearance) ||
      !ChartBackground(canvas.GetColorScheme(),land,water) ||
      !std::isfinite(user_scale) || user_scale <= 0 ||
      attenuation < 50 || attenuation > 100) return false;
  auto mesh=OnboardAisBodyMesh(appearance.target_class==1,x,y,north_angle,
      canvas.FromDIP(100)/100.0*user_scale*attenuation/100.0);
  if (mesh.fill.empty()) return false;
  const auto mode=canvas.GetColorScheme()==GLOBAL_COLOR_SCHEME_NIGHT
      ? ui::LightMode::Night : canvas.GetColorScheme()==GLOBAL_COLOR_SCHEME_DUSK
      ? ui::LightMode::Dusk : ui::LightMode::Day;
  const auto palette=ui::OnlineChartTheme(mode);
  // Night's prototype chart ancestor applies brightness(.78), to this body only.
  const auto fill=OnboardAisInk(palette.fill,mode), outline=OnboardAisInk(palette.stroke,mode);
  if (auto *native=dc.GetDC()) {
    if (!PaintOnboardAisBody(*native,mesh,fill,outline)) return false;
  } else {
#ifdef ocpnUSE_GL
    if (dc.m_canvasIndex<0 || dc.m_canvasIndex>=2) return false;
    auto *shader=pcolor_tri_shader_program[dc.m_canvasIndex];
    if (!shader) return false;
    GLint previous_program=0;glGetIntegerv(GL_CURRENT_PROGRAM,&previous_program);
    const auto blended=glIsEnabled(GL_BLEND);glDisable(GL_BLEND);
    shader->Bind();
    shader->SetUniformMatrix4fv("MVMatrix",
        reinterpret_cast<GLfloat *>(canvas.GetpVP()->vp_matrix_transform));
    const auto paint=[&](std::vector<float> &triangles,const wxColour &ink) {
      float color[]={ink.Red()/255.f,ink.Green()/255.f,ink.Blue()/255.f,1.f};
      shader->SetUniform4fv("color",color);
      shader->SetAttributePointerf("position",triangles.data());
      glDrawArrays(GL_TRIANGLES,0,triangles.size()/2);
    };
    paint(mesh.fill,fill);paint(mesh.outline,outline);
    shader->UnBind();glUseProgram(previous_program);
    if (blended) glEnable(GL_BLEND);
#else
    return false;
#endif
  }
  for (const auto *triangles : {&mesh.fill,&mesh.outline})
    for (std::size_t i=0;i<triangles->size();i+=2) {
      dc.CalcBoundingBox(std::floor((*triangles)[i]),std::floor((*triangles)[i+1]));
      dc.CalcBoundingBox(std::ceil((*triangles)[i]),std::ceil((*triangles)[i+1]));
    }
  return true;
}
} // namespace opennav::integration
