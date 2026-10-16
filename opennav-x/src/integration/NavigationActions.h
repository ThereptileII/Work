#pragma once
#include "application/NavigationObjects.h"
class MyFrame;
class ChartCanvas;
namespace opennav::integration {
// Prepare and persist a new draft before the native canvas clears its state.
// Failure leaves the draft editable; this never calls FinishRoute itself.
application::CommandResult PrepareNativeRouteFinish(MyFrame &frame,
                                                    ChartCanvas &canvas);
application::NavigationActions
MakeNavigationActions(MyFrame &frame,
                      std::function<vessel::Navigation()> position,
                      std::function<application::AnchorState(vessel::Time)> anchor);
}
