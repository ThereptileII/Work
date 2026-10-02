#include "integration/ChartRouteUnderlay.h"
#include "ui/Controls.h" // Before GL/X11 headers defining None.
#include "chcanv.h"
#include "integration/ChartPresentation.h"
#include "ocpndc.h"
#include <cmath>
#include <memory>
#include <wx/graphics.h>
#ifdef ocpnUSE_GL
#include "shaders.h"
#endif
namespace opennav::integration {
bool ChartRouteUnderlay::Draw(ocpnDC &dc, ChartCanvas &canvas) const {
  wxColour verified_ink;
  if (!ChartActiveRouteInk(canvas, verified_ink))
    return false;
  int width = 0, height = 0;
  dc.GetSize(&width, &height);
  auto mesh = Mesh(canvas.FromDIP(100) / 100.0, width, height);
  if (!mesh.valid)
    return false;
  if (mesh.triangles.empty())
    return true;
  const auto mode = canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_NIGHT
                        ? ui::LightMode::Night
                    : canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_DUSK
                        ? ui::LightMode::Dusk
                        : ui::LightMode::Day;
  const auto ink = ui::Colour(ui::FloatingTheme(mode).surface);
  if (auto *native = dc.GetDC()) {
    std::unique_ptr<wxGraphicsContext> gc(
        wxGraphicsContext::CreateFromUnknownDC(*native));
    if (!gc)
      return false;
    auto path = gc->CreatePath();
    // One compound union mesh, one alpha fill. Normalize triangle winding so
    // shared edges cannot cancel a neighboring triangle in the software path.
    for (std::size_t i = 0; i < mesh.triangles.size(); i += 6) {
      auto *t = mesh.triangles.data() + i;
      const bool reverse =
          (t[2] - t[0]) * (t[5] - t[1]) - (t[3] - t[1]) * (t[4] - t[0]) < 0;
      const int b = reverse ? 4 : 2, c = reverse ? 2 : 4;
      path.MoveToPoint(t[0], t[1]);
      path.AddLineToPoint(t[b], t[b + 1]);
      path.AddLineToPoint(t[c], t[c + 1]);
      path.CloseSubpath();
    }
    gc->SetBrush(wxBrush(wxColour(ink.Red(), ink.Green(), ink.Blue(), 153)));
    gc->FillPath(path, wxWINDING_RULE); // .6 is exactly 153/255.
  } else {
#ifdef ocpnUSE_GL
    if (dc.m_canvasIndex < 0 || dc.m_canvasIndex >= 2)
      return false;
    auto *shader = pcolor_tri_shader_program[dc.m_canvasIndex];
    if (!shader)
      return false;
    GLint program = 0, src_rgb = 0, dst_rgb = 0, src_alpha = 0, dst_alpha = 0,
          eq_rgb = 0, eq_alpha = 0;
    glGetIntegerv(GL_CURRENT_PROGRAM, &program);
    glGetIntegerv(GL_BLEND_SRC_RGB, &src_rgb);
    glGetIntegerv(GL_BLEND_DST_RGB, &dst_rgb);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &src_alpha);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &dst_alpha);
    glGetIntegerv(GL_BLEND_EQUATION_RGB, &eq_rgb);
    glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &eq_alpha);
    const auto blended = glIsEnabled(GL_BLEND);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
    shader->Bind();
    shader->SetUniformMatrix4fv(
        "MVMatrix",
        reinterpret_cast<GLfloat *>(canvas.GetpVP()->vp_matrix_transform));
    float color[]{ink.Red() / 256.f, ink.Green() / 256.f, ink.Blue() / 256.f,
                  .6f};
    shader->SetUniform4fv("color", color);
    shader->SetAttributePointerf("position", mesh.triangles.data());
    glDrawArrays(GL_TRIANGLES, 0, mesh.triangles.size() / 2);
    shader->UnBind();
    glUseProgram(program);
    glBlendFuncSeparate(src_rgb, dst_rgb, src_alpha, dst_alpha);
    glBlendEquationSeparate(eq_rgb, eq_alpha);
    if (!blended)
      glDisable(GL_BLEND);
#else
    return false;
#endif
  }
  for (std::size_t i = 0; i < mesh.triangles.size(); i += 2) {
    dc.CalcBoundingBox(std::floor(mesh.triangles[i]),
                       std::floor(mesh.triangles[i + 1]));
    dc.CalcBoundingBox(std::ceil(mesh.triangles[i]),
                       std::ceil(mesh.triangles[i + 1]));
  }
  return true;
}
} // namespace opennav::integration
