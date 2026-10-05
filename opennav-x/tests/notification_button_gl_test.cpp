#include <array>
#include <cassert>
#include <iostream>
using GLint = int;
using GLboolean = bool;
enum { GL_BLEND, GL_BLEND_SRC_RGB, GL_BLEND_DST_RGB, GL_BLEND_SRC_ALPHA,
       GL_BLEND_DST_ALPHA, GL_BLEND_EQUATION_RGB, GL_BLEND_EQUATION_ALPHA,
       GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_FUNC_ADD };
std::array<GLint,7> state;
int calls=0;
GLboolean glIsEnabled(int key) { ++calls; return state[key]; }
void glGetIntegerv(int key, GLint *value) { ++calls; *value=state[key]; }
void glEnable(int key) { ++calls; state[key]=1; }
void glDisable(int key) { ++calls; state[key]=0; }
void glBlendFuncSeparate(int sr,int dr,int sa,int da) {
  ++calls;state[1]=sr;state[2]=dr;state[3]=sa;state[4]=da;
}
void glBlendFunc(int s,int d) { glBlendFuncSeparate(s,d,s,d); }
void glBlendEquationSeparate(int r,int a) { ++calls;state[5]=r;state[6]=a; }
#include "integration/NotificationButtonGL.h"
int main() {
  for(int enabled : {0,1}) {
    state={enabled,21,22,23,24,25,26};const auto before=state;
    calls=0;{opennav::integration::NotificationButtonBlend guard(false);}
    assert(calls==0 && state==before);
    {
      opennav::integration::NotificationButtonBlend guard(true);
      assert(state[0]==1 && state[1]==GL_SRC_ALPHA && state[2]==GL_ONE_MINUS_SRC_ALPHA);
      assert(state[3]==GL_SRC_ALPHA && state[4]==GL_ONE_MINUS_SRC_ALPHA);
      assert(state[5]==GL_FUNC_ADD && state[6]==GL_FUNC_ADD);
    }
    assert(state==before);
  }
  std::cout << "GL blend opt-out is inert; enabled/disabled incoming state fully restored\n";
}
