#pragma once
#include "application/NavigationObjects.h"
class MyFrame;
namespace opennav::integration {
application::NavigationActions
MakeNavigationActions(MyFrame &frame,
                      std::function<vessel::Navigation()> position,
                      std::function<application::AnchorState()> anchor);
}
