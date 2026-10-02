#pragma once

#include <cstring>
#include <initializer_list>

namespace opennav::integration {

enum class ChartNameRole { Unchanged, Land, Water };

// Only six-byte S-57 feature names and the parsed TX attribute are inspected.
// Soundings, navigational features, formatted TE text and secondary attributes
// retain their complete upstream typography and presentation semantics.
inline ChartNameRole GeographicChartName(const char* feature,
                                         const char* instruction, bool tx) {
  if (!feature || !instruction || !tx ||
      std::strncmp(instruction, "OBJNAM,", 7) != 0)
    return ChartNameRole::Unchanged;
  if (std::strncmp(feature, "SEAARE", 6) == 0)
    return ChartNameRole::Water;
  for (const char* name : {"BUAARE", "LNDARE", "LNDRGN"})
    if (std::strncmp(feature, name, 6) == 0)
      return ChartNameRole::Land;
  return ChartNameRole::Unchanged;
}

}  // namespace opennav::integration
