#ifndef WEBFSEDITOR_H
#define WEBFSEDITOR_H

#include <Arduino.h>
#include "FS.h"
#include <ESPAsyncWebServer.h>
#include "WebFSEditorPages.h"

class WebFSEditor {
protected:

public:
  fs::FS &_fs;
  boolean &_doLoop;
  String httpuser;           // username to access web admin
  String httppassword;       // password to access web admin
  SemaphoreHandle_t xSPISemaphore;

  WebFSEditor(fs::FS &fs, boolean &doLoop);

  String listFiles(bool ishtml);

  String humanReadableSize(const size_t bytes);

  static String processor(const String& var);

  void configureWebServer(AsyncWebServer &server);

  void notFound(AsyncWebServerRequest *request);

  bool checkUserWebAuth(AsyncWebServerRequest * request);

  void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);

};

#endif
