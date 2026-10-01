#ifndef OPENNAV_DOWNLOADER_TEST_WX_LOG_H
#define OPENNAV_DOWNLOADER_TEST_WX_LOG_H

template <typename... Args>
void wxLogWarning(const char*, Args&&...) {}

template <typename... Args>
void wxLogMessage(const char*, Args&&...) {}

#endif
