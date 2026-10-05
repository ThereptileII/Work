#include "application/ChartInfo.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <string_view>

namespace opennav::application {
namespace {
constexpr std::size_t max_input = 1024 * 1024, max_sections = 512;
std::string Trim(std::string text) {
  const auto whitespace = [](unsigned char c) { return std::isspace(c); };
  const auto first = std::find_if_not(text.begin(), text.end(), whitespace);
  const auto last = std::find_if_not(text.rbegin(), text.rend(), whitespace).base();
  return first < last ? std::string(first, last) : std::string{};
}
std::string Lower(std::string text) {
  for (auto &c : text) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return text;
}
std::string Utf8(unsigned cp) {
  if (!cp || cp > 0x10ffff || (cp >= 0xd800 && cp <= 0xdfff)) return "\xef\xbf\xbd";
  std::string text;
  if (cp < 0x80) text += static_cast<char>(cp);
  else if (cp < 0x800) {
    text += static_cast<char>(0xc0 | (cp >> 6));
    text += static_cast<char>(0x80 | (cp & 63));
  } else if (cp < 0x10000) {
    text += static_cast<char>(0xe0 | (cp >> 12));
    text += static_cast<char>(0x80 | ((cp >> 6) & 63));
    text += static_cast<char>(0x80 | (cp & 63));
  } else {
    text += static_cast<char>(0xf0 | (cp >> 18));
    text += static_cast<char>(0x80 | ((cp >> 12) & 63));
    text += static_cast<char>(0x80 | ((cp >> 6) & 63));
    text += static_cast<char>(0x80 | (cp & 63));
  }
  return text;
}
std::string Decode(std::string_view text) {
  static const std::map<std::string, std::string> entities = {
      {"amp", "&"}, {"lt", "<"}, {"gt", ">"}, {"quot", "\""},
      {"apos", "'"}, {"nbsp", " "}, {"deg", "°"}, {"ndash", "–"},
      {"mdash", "—"}, {"hellip", "…"}, {"plusmn", "±"}, {"copy", "©"},
      {"le", "≤"}, {"ge", "≥"}};
  std::string result;
  for (std::size_t i = 0; i < text.size(); ++i) {
    if (text[i] == '&') {
      auto end = i + 1;
      while (end < text.size() && end - i <= 16 &&
             (std::isalnum(static_cast<unsigned char>(text[end])) || text[end] == '#')) ++end;
      const bool semicolon = end < text.size() && text[end] == ';';
      const auto token = std::string(text.substr(i + 1, end - i - 1));
      const auto named = entities.find(token);
      std::string decoded;
      if (semicolon && named != entities.end()) decoded = named->second;
      else if (token.size() > 1 && token[0] == '#') {
        unsigned cp = 0;
        std::size_t begin = 1;
        const bool hex = token[1] == 'x' || token[1] == 'X';
        if (hex) begin = 2;
        bool valid = begin < token.size();
        for (auto j = begin; j < token.size(); ++j) {
          const auto c = static_cast<unsigned char>(token[j]);
          const unsigned digit = c >= '0' && c <= '9' ? c - '0'
                               : c >= 'a' && c <= 'f' ? c - 'a' + 10
                               : c >= 'A' && c <= 'F' ? c - 'A' + 10 : 99;
          if (digit >= (hex ? 16u : 10u) || cp > 0x10ffff) { valid = false; break; }
          cp = cp * (hex ? 16 : 10) + digit;
        }
        if (valid) decoded = Utf8(cp);
      }
      if (!decoded.empty()) { result += decoded; i = end - (semicolon ? 0 : 1); continue; }
    }
    const auto c = static_cast<unsigned char>(text[i]);
    if (c >= 32 && c != 127) result += static_cast<char>(c);
    else if (std::isspace(c)) result += ' ';
  }
  return result;
}
std::string Normalize(const std::string &text) {
  std::string result;
  bool space = false;
  for (const char c : text) {
    if (c == '\n' || c == '\t') {
      while (!result.empty() && result.back() == ' ') result.pop_back();
      if (!result.empty() && result.back() != c) result += c;
      space = false;
    } else if (c == ' ') space = !result.empty();
    else {
      if (space && result.back() != '\n' && result.back() != '\t') result += ' ';
      result += c;
      space = false;
    }
  }
  return Trim(result);
}
std::string Attribute(const std::string &tag, const std::string &attribute) {
  const auto lower = Lower(tag);
  auto key = lower.find(attribute);
  while (key != std::string::npos) {
    const auto after = key + attribute.size();
    if ((key == 0 || std::isspace(static_cast<unsigned char>(tag[key - 1]))) &&
        after < tag.size() && (tag[after] == '=' || std::isspace(static_cast<unsigned char>(tag[after])))) {
      auto value = after;
      while (value < tag.size() && std::isspace(static_cast<unsigned char>(tag[value]))) ++value;
      if (value == tag.size() || tag[value++] != '=') return {};
      while (value < tag.size() && std::isspace(static_cast<unsigned char>(tag[value]))) ++value;
      const char quote = value < tag.size() && (tag[value] == '\'' || tag[value] == '"') ? tag[value++] : 0;
      auto stop = value;
      while (stop < tag.size() && (quote ? tag[stop] != quote : !std::isspace(static_cast<unsigned char>(tag[stop])))) ++stop;
      return Decode(std::string_view(tag).substr(value, stop - value));
    }
    key = lower.find(attribute, after);
  }
  return {};
}
const std::map<std::string, std::string> labels = {
    {"OBJNAM", "Name"}, {"NOBJNM", "Local name"}, {"INFORM", "Information"},
    {"NINFOM", "Local information"}, {"CATLIT", "Light type"},
    {"COLOUR", "Colour"}, {"HEIGHT", "Height"}, {"ELEVAT", "Elevation"},
    {"VALNMR", "Nominal range"}, {"LITCHR", "Light characteristic"},
    {"SIGGRP", "Signal group"}, {"SIGPER", "Signal period"},
    {"SECTR1", "Sector start"}, {"SECTR2", "Sector end"}, {"STATUS", "Status"},
    {"QUASOU", "Sounding quality"}, {"QUAPOS", "Position quality"},
    {"VALSOU", "Sounding"}, {"DRVAL1", "Minimum depth"},
    {"DRVAL2", "Maximum depth"}, {"WATLEV", "Water level"},
    {"CATWRK", "Wreck category"}, {"CATOBS", "Obstruction category"},
    {"RESTRN", "Restrictions"}, {"CATSPM", "Mark category"},
    {"CATLAM", "Lateral mark"}, {"CATCAM", "Cardinal mark"},
    {"CONVIS", "Visual conspicuousness"}, {"NATCON", "Construction"}};
struct Section { std::string text, title; };
ChartInfoObject Present(Section section) {
  ChartInfoObject object;
  object.details = Normalize(section.text);
  object.title = Normalize(section.title);
  if (object.title.empty()) object.title = "Chart information";
  std::vector<std::string> prose;
  std::size_t begin = 0;
  while (begin < object.details.size()) {
    const auto end = object.details.find('\n', begin);
    const auto line = Trim(object.details.substr(begin, end - begin));
    const auto tab = line.find('\t');
    const auto label = labels.find(Trim(line.substr(0, tab)));
    if (tab != std::string::npos && label != labels.end()) {
      auto value = Trim(line.substr(tab + 1));
      std::replace(value.begin(), value.end(), '\t', ' ');
      value = Normalize(value);
      if (!value.empty()) {
        object.summary.push_back(label->second + ": " + value);
        if (label->first == "OBJNAM") { object.kind = object.title; object.title = value; }
      }
    } else if (!line.empty() && tab == std::string::npos && line != object.title) {
      // Retain prose, notices and the upstream's combined light-sector text.
      // An isolated S-57 class code is only useful in the complete details.
      const auto code = line.find('(');
      const auto acronym = code != std::string::npos && line.back() == ')'
          ? line.substr(code + 1, line.size() - code - 2) : std::string{};
      const bool class_code = !acronym.empty() && acronym.size() <= 10 &&
          std::all_of(acronym.begin(), acronym.end(), [](unsigned char c) {
            return std::isupper(c) || std::isdigit(c) || c == '_';
          });
      if (!(class_code && (line.substr(0, code) == object.title + " " || code == 0)))
        prose.push_back(line);
    } else if (tab != std::string::npos && label == labels.end()) {
      const auto key = Trim(line.substr(0, tab));
      const bool technical = key.size() == 6 &&
          std::all_of(key.begin(), key.end(), [](unsigned char c) { return std::isupper(c) || c == '_'; });
      if (!technical) {
        auto readable = line;
        std::replace(readable.begin(), readable.end(), '\t', ' ');
        prose.push_back(Normalize(readable));
      }
    }
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  object.summary.insert(object.summary.begin(), prose.begin(), prose.end());
  return object;
}
} // namespace
ChartInfo ParseChartInfo(const std::string &html, double latitude, double longitude) {
  ChartInfo info;
  info.latitude = latitude;
  info.longitude = longitude;
  info.position_valid = std::isfinite(latitude) && std::isfinite(longitude) &&
                        std::abs(latitude) <= 90 && std::abs(longitude) <= 180;
  info.truncated = html.size() > max_input;
  auto length = (std::min)(html.size(), max_input);
  if (length < html.size())
    while (length && (static_cast<unsigned char>(html[length]) & 0xc0) == 0x80) --length;
  const std::string_view input(html.data(), length);
  Section section;
  bool bold = false, first_bold = true;
  std::string hidden;
  const auto finish = [&] {
    auto object = Present(std::move(section));
    if (!object.details.empty()) {
      if (info.objects.size() < max_sections) info.objects.push_back(std::move(object));
      else info.truncated = true;
    }
    section = {};
    bold = false;
    first_bold = true;
  };
  for (std::size_t pos = 0; pos < input.size();) {
    if (input[pos] != '<') {
      const auto end = input.find('<', pos);
      const auto text = Decode(input.substr(pos, end == std::string_view::npos ? input.size() - pos : end - pos));
      if (hidden.empty()) { section.text += text; if (bold) section.title += text; }
      pos = end == std::string_view::npos ? input.size() : end;
      continue;
    }
    auto end = pos + 1;
    char quote = 0;
    for (; end < input.size(); ++end) {
      const auto c = input[end];
      if (quote) { if (c == quote) quote = 0; }
      else if (c == '\'' || c == '"') quote = c;
      else if (c == '>') break;
    }
    if (end == input.size()) {
      if (hidden.empty()) section.text += Decode(input.substr(pos));
      break;
    }
    const auto original = std::string(input.substr(pos + 1, end - pos - 1));
    const auto tag = Lower(Trim(original));
    if (tag.empty()) { pos = end + 1; continue; }
    const auto name = tag.substr(0, tag.find_first_of(" \t\r\n/", tag[0] == '/' ? 1 : 0));
    pos = end + 1;
    if (!hidden.empty()) { if (name == "/" + hidden) hidden.clear(); continue; }
    if (name == "script" || name == "style" || name == "head") { hidden = name; continue; }
    if (name == "hr") { finish(); continue; }
    if ((name == "b" || name == "strong") && first_bold) { bold = true; first_bold = false; }
    if (name == "/b" || name == "/strong") bold = false;
    if (name == "br" || name == "/p" || name == "/div" || name == "/tr" ||
        name == "/table" || name == "/li" || name == "h1" || name == "/h1" ||
        name == "h2" || name == "/h2") section.text += '\n';
    if (name == "/td" || name == "/th") section.text += '\t';
    if (name == "a") {
      // Keep an attachment's target available as inert technical text. Never
      // create navigation/file actions from markup supplied by a chart/plugin.
      const auto target = Attribute(original, "href");
      if (!target.empty()) section.text += " [Attachment reference: " + target + "] ";
    }
    if (name == "img") {
      const auto description = Attribute(original, "alt");
      const auto source = Attribute(original, "src");
      section.text += " [Image reference: " + description +
          (description.empty() || source.empty() ? "" : " / ") + source + "] ";
    }
  }
  finish();
  if (info.truncated)
    info.notice = "This query exceeds the display limit. Some information is omitted; use Legacy Object Query for the complete result.";
  else if (info.objects.empty())
    info.notice = "No chart object information was supplied for this position.";
  return info;
}
} // namespace opennav::application
