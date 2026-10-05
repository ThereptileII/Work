typedef enum _LUPname {
  SIMPLIFIED = 'L',             // points
  PAPER_CHART = 'R',            // points
  LINES = 'S',                  // lines
  PLAIN_BOUNDARIES = 'N',       // areas
  SYMBOLIZED_BOUNDARIES = 'O',  // areas
  LUPNAME_NUM = 5
} LUPname;
enum { S52_MAR_SYMPLIFIED_PNT, S52_MAR_SYMBOLIZED_BND };
static double parameters[2]{};
void S52_setMarinerParam(int key, double value) { parameters[key]=value; }
class s52plib {
 public:
  LUPname m_nSymbolStyle=PAPER_CHART, m_nBoundaryStyle=PLAIN_BOUNDARIES;
  void UpdateMarinerParams(void);
void EnablePresentationSimplifiedSymbols() { m_presentationSimpleSymbols = true; }
LUPname GetEffectiveSymbolStyle() const {
    return m_presentationSimpleSymbols ? SIMPLIFIED : m_nSymbolStyle;
  }
 private:
bool m_presentationSimpleSymbols = false;
};
void s52plib::UpdateMarinerParams(void) {
  //      Symbol Style
  if (SIMPLIFIED == GetEffectiveSymbolStyle())
    S52_setMarinerParam(S52_MAR_SYMPLIFIED_PNT, 1.0);
  else
    S52_setMarinerParam(S52_MAR_SYMPLIFIED_PNT, 0.0);

  //      Boundary Style
  if (SYMBOLIZED_BOUNDARIES == m_nBoundaryStyle)
    S52_setMarinerParam(S52_MAR_SYMBOLIZED_BND, 1.0);
  else
    S52_setMarinerParam(S52_MAR_SYMBOLIZED_BND, 0.0);
}
