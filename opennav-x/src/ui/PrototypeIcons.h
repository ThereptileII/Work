#pragma once
#include "ui/Controls.h"

// Derived unchanged SVG path data from the supplied immutable v8 app.js icons.
// HTML SHA-256 b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447.
namespace opennav::ui {
inline const char *PrototypeIconPath(XNavIcon icon) {
  switch (icon) {
    case XNavIcon::Speed: return "M4 17a9 9 0 1 1 16 0m-8-4 5-6M8 20h8";
    case XNavIcon::Depth: return "M12 3v13m-4-4 4 4 4-4M3 21l3-2 3 2 3-2 3 2 3-2 3 2";
    case XNavIcon::Wind: return "M3 7h12a3 3 0 1 0-3-3M3 12h16a3 3 0 1 1-3 3M3 17h6a3 3 0 1 1-3 3";
    case XNavIcon::Battery: return "M3 6h16v12H3Zm18 4v4M6 9v6m4-6v6m4-6v6";
    case XNavIcon::Spark: return "m12 2 2.5 7.5L22 12l-7.5 2.5L12 22l-2.5-7.5L2 12l7.5-2.5Z";
    case XNavIcon::Plus: return "M12 5v14M5 12h14";
    case XNavIcon::Minus: return "M5 12h14";
    case XNavIcon::Ownship: return "M12 2v4m0 12v4M2 12h4m12 0h4M19 12a7 7 0 1 0-14 0 7 7 0 0 0 14 0Zm-7-2v4m-2-2h4";
    case XNavIcon::Back: return "M19 12H5m7-7-7 7 7 7";
    case XNavIcon::Close: return "m6 6 12 12M6 18 18 6";
    case XNavIcon::Route: return "M6 7a2 2 0 1 0 0-4 2 2 0 0 0 0 4Zm12 14a2 2 0 1 0 0-4 2 2 0 0 0 0 4ZM6 7v5a4 4 0 0 0 4 4h4a4 4 0 0 0 0-8h-1m5 8v1";
    case XNavIcon::Compass: return "M12 2v4m0 12v4M2 12h4m12 0h4M19 12a7 7 0 1 0-14 0 7 7 0 0 0 14 0Zm-7-2v4m-2-2h4";
    case XNavIcon::Settings: return "m9 3-1 3-3 1-2 3 2 3v3l3 2 3-1 3 1 3-2v-3l2-3-2-3-3-1-1-3Zm6 8a3 3 0 1 1-6 0 3 3 0 0 1 6 0Z";
    case XNavIcon::Chart: return "M3 5 9 3l6 2 6-2v16l-6 2-6-2-6 2Zm6-2v16m6-14v16";
    case XNavIcon::Traffic: return "m7 3 4 13-4-3-4 3Zm10 5 4 13-4-3-4 3Z";
    case XNavIcon::Energy: return "m13 2-9 12h7l-1 8 10-13h-7Z";
    case XNavIcon::Instruments: return "M4 18a9 9 0 1 1 16 0M6 8l2 2m10-2-2 2M12 4v3m-9 7h3m12 0h3m-9 2 4-5M9 20h6";
    case XNavIcon::Anchor: return "M12 8v13m-5-9H3a9 9 0 0 0 18 0h-4M8 9h8M15 5a3 3 0 1 0-6 0 3 3 0 0 0 6 0Z";
    case XNavIcon::Radar: return "M12 3a9 9 0 1 0 9 9M12 7a5 5 0 1 0 5 5M12 12 21 3M12 3v9h9";
    case XNavIcon::Sun: return "M16 12a4 4 0 1 1-8 0 4 4 0 0 1 8 0ZM12 2v2m0 16v2M2 12h2m16 0h2M5 5l1.5 1.5m11 11L19 19M5 19l1.5-1.5m11-11L19 5";
    case XNavIcon::Dusk: return "M3 17h18M5 14a7 7 0 0 1 14 0M12 3v3M3 7l2 2m14 0 2-2M7 21h10";
    case XNavIcon::Moon: return "M20 14A9 9 0 0 1 10 3a9 9 0 1 0 10 11Z";
    case XNavIcon::Bell: return "M6 9a6 6 0 0 1 12 0v6l2 3H4l2-3ZM10 21h4";
    case XNavIcon::Search: return "M10 17a7 7 0 1 0 0-14 7 7 0 0 0 0 14Zm5-2 6 6";
    case XNavIcon::Layers: return "m12 3 10 5-10 5L2 8Zm-10 9 10 5 10-5M2 16l10 5 10-5";
    case XNavIcon::Ruler: return "m3 16 13-13 5 5L8 21Zm9-9 3 3M8 11l3 3m-7 1 3 3";
    case XNavIcon::Edit: return "m4 15 12-12 5 5L9 20l-6 1Zm9-9 5 5";
    case XNavIcon::Pin: return "M18 9c0 5-6 12-6 12S6 14 6 9a6 6 0 1 1 12 0ZM14 9a2 2 0 1 0-4 0 2 2 0 0 0 4 0Z";
    case XNavIcon::Sliders: return "M4 6h7m4 0h5M4 12h2m4 0h10M4 18h10m4 0h2M11 3v6m-5 0v6m8 0v6";
    case XNavIcon::Chevron: return "m9 5 7 7-7 7";
    case XNavIcon::Menu: return "M3 6h18M3 12h18M3 18h18";
    case XNavIcon::Shield: return "M12 2 3 6v6c0 5 9 10 9 10s9-5 9-10V6Zm-5 9 3 3 7-7";
    case XNavIcon::Download: return "M12 3v12m-5-5 5 5 5-5M4 17v4h16v-4";
    case XNavIcon::Refresh: return "M20 8a9 9 0 1 0 1 8M20 3v5h-5";
    case XNavIcon::Info: return "M12 11v6m0-10h.01M21 12a9 9 0 1 0-18 0 9 9 0 0 0 18 0Z";
    case XNavIcon::Boat: return "M8 3h8v7l5 3-3 7H6l-3-7 5-3Zm0 7 4-2 4 2M12 8v12";
    default: return "";
  }
}
} // namespace opennav::ui
