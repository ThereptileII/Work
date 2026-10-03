# Untyped -D inputs from PowerShell retain Windows backslashes. Normalize before
# source-list expansion, find_package, or private-preparation verification.
# A literal separator replacement preserves drive letters and UNC paths even in
# host-side regression checks (TO_CMAKE_PATH interprets host path-list syntax).
foreach(trust_path IN ITEMS OPENNAV_SOURCE_DIR OPENNAV_TOOLS_DIR CURL_ROOT
    wxWidgets_ROOT_DIR wxWidgets_LIB_DIR SKAGER_OCHARTS_PREPARED)
  if(DEFINED ${trust_path})
    string(REPLACE "\\" "/" ${trust_path} "${${trust_path}}")
  endif()
endforeach()
unset(trust_path)
