#include "integration/ChartNameTypography.h"
#include "integration/ChartNameSpacing.h"
#include <iostream>
#include <limits>

int main() {
  using namespace opennav::integration;
  int checked = 0;
  const auto expect = [&checked](const char* feature, const char* rule,
                                bool tx, ChartNameRole role) {
    ++checked;
    if (GeographicChartName(feature, rule, tx) != role) {
      std::cerr << "Unexpected chart-name role at case " << checked << '\n';
      return false;
    }
    return true;
  };
  bool ok = true;
  for (const char* feature : {"BUAARE", "LNDARE", "LNDRGN"})
    ok &= expect(feature, "OBJNAM,1,2,3,'15120',0,0,XNGEO,26", true,
                 ChartNameRole::Land);
  ok &= expect("SEAARE", "OBJNAM,1,2,3,'15110',0,0,XNGEO,26", true,
               ChartNameRole::Water);
  for (const char* feature : {"LIGHTS", "BOYLAT", "BCNCAR", "WRECKS", "OBSTRN",
                              "SOUNDG", "DEPARE", "DEPCNT", "RESARE", "ACHARE"})
    ok &= expect(feature, "OBJNAM,1,2,3,'15120',0,0,CHBLK,26", true,
                 ChartNameRole::Unchanged);
  for (const char* rule : {"", "OBJNAM", "NOBJNM,1", "INFORM,OBJNAM,1",
                           "OBJNAMS,1", "'OBJNAM',1", "DEPTH,1"})
    ok &= expect("SEAARE", rule, true, ChartNameRole::Unchanged);
  ok &= expect("SEAARE", "OBJNAM,1", false, ChartNameRole::Unchanged);
  ok &= expect(nullptr, "OBJNAM,1", true, ChartNameRole::Unchanged);
  ok &= expect("SEAARE", nullptr, true, ChartNameRole::Unchanged);
  // S-57 class names are fixed-width: no zero terminator is required.
  const char feature[6] = {'L','N','D','A','R','E'};
  ok &= expect(feature, "OBJNAM,1", true, ChartNameRole::Land);
  const auto check = [&](bool condition) { ++checked; ok &= condition; };
  // Real Scandinavian names retain their letters. Canonical decompositions
  // keep native whole-string shaping rather than guessing glyph boundaries.
  check(ChartNameClusterStarts(U"ÖSTERSJÖN").size() == 9);
  check(ChartNameClusterStarts(U"Arko\u0308sund").empty());
  check(ChartNameClusterStarts(U"A\u0308\u0301 B").empty());
  for (const auto* name : {U"\u0308A", U"A \u0308", U"\u0627\u0644\u0628\u062d\u0631",
                          U"\u6d77", U"A\u200dB", U"A\nB", U"\U0001f30a", U""})
    check(ChartNameClusterStarts(name).empty());
  check(ChartNameClusterStarts(std::u32string(513, U'A')).empty());
  check(ChartNameTrackingWidth(3, 5) == 15); // Chromium SVG measurement
  check(ChartNameTrackingWidth(3, 6.25) == 18.75);
  for (const double spacing : {0.0, -1.0, 65.0,
         std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    check(ChartNameTrackingWidth(3, spacing) == 0);
  check(ChartNameTrackingWidth(513, 1) == 0);
  check(ChartNameTrackingWidth(512, 64) == 0);
  std::cout << checked << " chart-name typography boundary checks\n";
  return ok ? 0 : 1;
}
