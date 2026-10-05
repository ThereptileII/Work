#include <iostream>
#include <stdexcept>
#include "actual-symbol-style.inc"

int main() {
  int checks = 0;
  auto check = [&](bool result, const char* reason) {
    ++checks;
    if (!result) throw std::runtime_error(reason);
  };
  try {
    for (auto saved : {PAPER_CHART, SIMPLIFIED}) {
      for (auto boundary : {PLAIN_BOUNDARIES, SYMBOLIZED_BOUNDARIES}) {
        s52plib stock;
        stock.m_nSymbolStyle = saved;
        stock.m_nBoundaryStyle = boundary;
        stock.UpdateMarinerParams();
        check(stock.GetEffectiveSymbolStyle() == saved, "unbound renderer must follow host/saved preference");
        check(S52_getMarinerParam(S52_MAR_SYMPLIFIED_PNT) == (saved == SIMPLIFIED ? 1.0 : 0.0), "stock mariner parameter must follow stock table");
        s52plib owned;
        owned.m_nSymbolStyle = saved;
        owned.m_nBoundaryStyle = boundary;
        owned.EnablePresentationSimplifiedSymbols();
        owned.UpdateMarinerParams();
        check(owned.GetEffectiveSymbolStyle() == SIMPLIFIED, "verified style must select Simplified");
        check(owned.m_nSymbolStyle == saved, "enabling must preserve stored preference");
        check(S52_getMarinerParam(S52_MAR_SYMPLIFIED_PNT) == 1.0, "conditional/cache parameter must follow effective table");
        check(S52_getMarinerParam(S52_MAR_SYMBOLIZED_BND) == (boundary == SYMBOLIZED_BOUNDARIES ? 1.0 : 0.0), "area boundary preference must remain unchanged");
        // Host/config synchronization can write Paper again: owned display remains
        // Simplified, while the field remains exactly the value the host supplied.
        owned.m_nSymbolStyle = PAPER_CHART;
        owned.EnablePresentationSimplifiedSymbols();
        owned.UpdateMarinerParams();
        check(owned.GetEffectiveSymbolStyle() == SIMPLIFIED && owned.m_nSymbolStyle == PAPER_CHART, "host updates must not overwrite owned policy or saved field");
        check(S52_getMarinerParam(S52_MAR_SYMPLIFIED_PNT) == 1.0, "repeated updates must retain effective parameter");
        s52plib next_stock;
        next_stock.m_nSymbolStyle = saved;
        next_stock.UpdateMarinerParams();
        check(next_stock.GetEffectiveSymbolStyle() == saved, "new stock/fallback instance must not inherit owned policy");
        check(S52_getMarinerParam(S52_MAR_SYMPLIFIED_PNT) == (saved == SIMPLIFIED ? 1.0 : 0.0), "new stock instance must restore its own mariner parameter");
      }
    }
    std::cout << checks << " actual private getter/mariner checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }
}
