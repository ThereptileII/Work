#include "application/ChartInfo.h"
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace opennav::application;
namespace {
void Check(bool value, const char *why) {
  if (!value) throw std::runtime_error(why);
}
std::string Row(const std::string &key, const std::string &value) {
  return "<tr><td valign=top><font size=-2>" + key +
      "</font></td><td>&nbsp;&nbsp;</td><td valign=top><font size=-1>" +
      value + "</font></td></tr>\n";
}
bool SummaryContains(const ChartInfoObject &object, const std::string &needle) {
  for (const auto &line : object.summary)
    if (line.find(needle) != std::string::npos) return true;
  return false;
}
void OverlappingObjects() {
  // Same section/table structure emitted by the pinned s57chart query; the
  // values are explicitly offline fixtures, not real navigation observations.
  const auto html = "<html><body><b>Lateral buoy</b> <font size=-2>(BOYLAT)</font><br>"
      "<table>" + Row("OBJNAM", "Fixture &amp; Test") +
      Row("COLOUR", "red (3)") + Row("ZZTEST", "unknown source value") +
      "</table><hr noshade><b>Depth area</b> <font size=-2>(DEPARE)</font><br>"
      "<table><tr><td><font size=-1>4.0 m - 8.0 m</font></td></tr></table>"
      "<hr noshade><b>AIS Area Notice:</b> Restricted area<br>expires: supplied date"
      "<hr noshade><b>Lateral buoy</b><br>Second overlapping object</body></html>";
  const auto info = ParseChartInfo(html, 57.2, 16.3);
  Check(info.position_valid && !info.truncated && info.objects.size() == 4,
        "All native, overlay and notice sections must survive without deduplication");
  Check(info.objects[0].title == "Fixture & Test" && info.objects[0].kind == "Lateral buoy",
        "Observed object name and human-readable class lead the presentation");
  Check(SummaryContains(info.objects[0], "Colour: red (3)"),
        "Readable labels preserve upstream-decoded values and enum provenance");
  Check(!SummaryContains(info.objects[0], "ZZTEST") &&
        info.objects[0].details.find("ZZTEST") != std::string::npos &&
        info.objects[0].details.find("unknown source value") != std::string::npos,
        "Unknown attributes remain accessible in complete details");
  Check(SummaryContains(info.objects[1], "4.0 m - 8.0 m"),
        "Depth ranges retain upstream units");
  Check(info.objects[2].details.find("expires: supplied date") != std::string::npos,
        "AIS notice expiry is retained");
}
void GroupedLights() {
  const auto info = ParseChartInfo(
      "<b>Light</b> <font size=-2>(LIGHTS)</font><br>"
      "<font size=-2>57° N 16° E</font><br>"
      "<font size=-2>(Sector angles are True Bearings from Seaward)</font><br>"
      "<table><tr><td></td><td><b>Fl (2) R 6 s 12 m 8 NM</b> (120&#176 - 180&#176;)"
      "<br>Fixture red sector</td></tr>"
      "<tr><td></td><td><b>Fl (2) W 6 s 12 m 10 NM</b> (180° - 220°)"
      "<br>Fixture white sector</td></tr></table><hr>", 0, 0);
  Check(info.objects.size() == 1 && info.objects[0].title == "Light",
        "Upstream grouped light sections remain grouped");
  Check(SummaryContains(info.objects[0], "True Bearings from Seaward") &&
        SummaryContains(info.objects[0], "Fl (2) R") &&
        SummaryContains(info.objects[0], "Fl (2) W"),
        "Every light sector and bearing convention remains visible");
  Check(info.objects[0].details.find("120° - 180°") != std::string::npos,
        "Pinned semicolonless and regular numeric entities decode correctly");
}
void InertMarkup() {
  const auto info = ParseChartInfo(
      "<HTML><HEAD><style>hidden styling</style></HEAD><BODY>"
      "<B>Plugin fixture</B><BR>Supplied &lt;word&gt; &#x00c5; &unknown;"
      "<script>do-not-display-or-execute()</script><BR>"
      "<a HREF = '/Charts/CaseSensitive.TXT'>Document &amp; notes</a>"
      "<img src='https://example.invalid/never-fetch' alt='supplied image description'>"
      "</BODY></HTML>", 10, 20);
  Check(info.objects.size() == 1, "Plugin text remains readable");
  const auto &details = info.objects[0].details;
  Check(details.find("<word> Å &unknown;") != std::string::npos,
        "Text entities decode without interpreting the result as markup");
  Check(details.find("hidden styling") == std::string::npos &&
        details.find("do-not-display") == std::string::npos,
        "Script/style content is not presented as chart information");
  Check(details.find("/Charts/CaseSensitive.TXT") != std::string::npos &&
        details.find("Document & notes") != std::string::npos,
        "Attachment labels and exact case-sensitive references are retained as inert text");
  Check(details.find("supplied image description") != std::string::npos &&
        details.find("https://example.invalid/never-fetch") != std::string::npos,
        "Images remain inert descriptive references without remote loading");
}
void EmptyMalformedAndLimits() {
  const auto empty = ParseChartInfo("", std::numeric_limits<double>::quiet_NaN(), 0);
  Check(empty.objects.empty() && !empty.notice.empty() && !empty.position_valid,
        "Empty query and missing location are explicit");
  const auto malformed = ParseChartInfo("<>Plain text<br>value <incomplete", 91, 180);
  Check(!malformed.position_valid && malformed.objects.size() == 1 &&
        malformed.objects[0].details.find("<incomplete") != std::string::npos,
        "Unterminated markup does not discard the remaining supplied text");
  std::string sections;
  for (int i = 0; i < 513; ++i) sections += "<b>Object</b><br>value<hr>";
  const auto many = ParseChartInfo(sections, 0, 0);
  Check(many.truncated && many.objects.size() == 512 &&
        many.notice.find("omitted") != std::string::npos,
        "The bounded section limit is explicitly reported, never silent");
  const auto large = ParseChartInfo(std::string(1024 * 1024 + 1, 'x'), 0, 0);
  Check(large.truncated && large.notice.find("Legacy") != std::string::npos,
        "Oversized input tells users where the complete source query remains available");
}
}
int main() {
  try {
    OverlappingObjects(); GroupedLights(); InertMarkup(); EmptyMalformedAndLimits();
    std::cout << "Chart information presentation tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
