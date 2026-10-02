#include "integration/ChartNameTypography.h"
#include <iostream>

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
  std::cout << checked << " chart-name typography boundary checks\n";
  return ok ? 0 : 1;
}
