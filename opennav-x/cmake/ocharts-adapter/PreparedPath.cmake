# Normalize before verification, globbing and generated source-list expansion.
# An untyped -D value can otherwise retain native backslash escape sequences.
file(TO_CMAKE_PATH "${SKAGER_PREPARED}" SKAGER_PREPARED)
get_filename_component(SKAGER_PREPARED "${SKAGER_PREPARED}" ABSOLUTE)
