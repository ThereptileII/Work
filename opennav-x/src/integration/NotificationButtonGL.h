#pragma once

// Include after upstream's platform GL headers. Only a successfully generated
// SKAGER notification bitmap opts in; Legacy and stock fallback do no GL work.
namespace opennav::integration {
class NotificationButtonBlend {
 public:
  explicit NotificationButtonBlend(bool active) : active_(active) {
    if (!active_) return;
    enabled_ = glIsEnabled(GL_BLEND);
    glGetIntegerv(GL_BLEND_SRC_RGB, &src_rgb_);
    glGetIntegerv(GL_BLEND_DST_RGB, &dst_rgb_);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &src_alpha_);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &dst_alpha_);
    glGetIntegerv(GL_BLEND_EQUATION_RGB, &eq_rgb_);
    glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &eq_alpha_);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
  }
  ~NotificationButtonBlend() {
    if (!active_) return;
    glBlendFuncSeparate(src_rgb_, dst_rgb_, src_alpha_, dst_alpha_);
    glBlendEquationSeparate(eq_rgb_, eq_alpha_);
    if (!enabled_) glDisable(GL_BLEND);
  }
  NotificationButtonBlend(const NotificationButtonBlend &) = delete;
  NotificationButtonBlend &operator=(const NotificationButtonBlend &) = delete;
 private:
  bool active_;
  GLboolean enabled_ = false;
  GLint src_rgb_ = 0, dst_rgb_ = 0, src_alpha_ = 0, dst_alpha_ = 0,
        eq_rgb_ = 0, eq_alpha_ = 0;
};
} // namespace opennav::integration
