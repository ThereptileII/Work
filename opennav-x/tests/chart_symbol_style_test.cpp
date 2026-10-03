#include <stdexcept>
#include <iostream>
#include "actual-symbol-style.inc"
int main() try {
  int checks = 0;
  const auto check = [&](bool value) {
    ++checks;
    if (!value) throw std::runtime_error("Effective symbol style check " + std::to_string(checks));
  };
  s52plib standard;
  check(standard.GetEffectiveSymbolStyle() == PAPER_CHART);
  standard.UpdateMarinerParams();
  check(parameters[S52_MAR_SYMPLIFIED_PNT] == 0);
  check(parameters[S52_MAR_SYMBOLIZED_BND] == 0);
  standard.m_nSymbolStyle = SIMPLIFIED;
  check(standard.GetEffectiveSymbolStyle() == SIMPLIFIED);
  standard.UpdateMarinerParams();
  check(parameters[S52_MAR_SYMPLIFIED_PNT] == 1);
  s52plib styled;
  const auto saved = styled.m_nSymbolStyle;
  styled.EnablePresentationSimplifiedSymbols();
  check(styled.m_nSymbolStyle == saved);
  check(styled.GetEffectiveSymbolStyle() == SIMPLIFIED);
  styled.UpdateMarinerParams();
  check(parameters[S52_MAR_SYMPLIFIED_PNT] == 1);
  check(styled.m_nSymbolStyle == saved);
  styled.m_nBoundaryStyle = SYMBOLIZED_BOUNDARIES;
  styled.UpdateMarinerParams();
  check(parameters[S52_MAR_SYMBOLIZED_BND] == 1);
  check(styled.m_nSymbolStyle == PAPER_CHART);
  // An upstream preference change remains stored but cannot replace verified
  // presentation-local selection. New Standard/Legacy libraries use that value.
  for (auto preference : {SIMPLIFIED, PAPER_CHART, SIMPLIFIED, PAPER_CHART}) {
    styled.m_nSymbolStyle = preference;
    styled.UpdateMarinerParams();
    check(styled.m_nSymbolStyle == preference);
    check(styled.GetEffectiveSymbolStyle() == SIMPLIFIED);
    check(parameters[S52_MAR_SYMPLIFIED_PNT] == 1);
    s52plib fallback;
    fallback.m_nSymbolStyle = styled.m_nSymbolStyle;
    check(fallback.GetEffectiveSymbolStyle() == preference);
    fallback.UpdateMarinerParams();
    check(parameters[S52_MAR_SYMPLIFIED_PNT] == (preference == SIMPLIFIED ? 1 : 0));
  }
  std::cout << checks << " actual effective-style/mariner-parameter checks passed\n";
}
catch (const std::exception& e) { std::cerr << e.what() << "\n"; return 1; }
