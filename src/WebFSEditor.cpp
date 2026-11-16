
#include "WebFSEditor.h"
#include "WebFSEditorCompressed.h"
#include "BootRecovery.h"
#include "ConfigTrialRuntime.h"
#include <Arduino.h>
#include <ArduinoLog.h>

WebFSEditor::WebFSEditor(fs::FS &fs, boolean &doLoop):
  _fs(fs),
  _doLoop(doLoop) {

  useSemaphore = true;
}


// list all of the files, if ishtml=true, return html else json
String WebFSEditor::listFiles(bool ishtml) {
  _doLoop = false;
  if (useSemaphore) xSemaphoreTake( xSPISemaphore, portMAX_DELAY );
  String returnText = "";
  Log.noticeln("Listing files stored on _fs");
  File root = _fs.open("/");
  File foundfile = root.openNextFile();
  if (ishtml) {
    returnText += "<table><tr><th align='left'>Name</th><th align='left'>Size</th><th></th><th></th></tr>";
  } else {
    returnText += "{ \"files\":[\n";
  }
  uint8_t i=0;
  while (foundfile) {
    if (ishtml) {
      returnText += "<tr align='left'><td>" + String(foundfile.name()) + "</td><td>" + humanReadableSize(foundfile.size()) + "</td>";
      returnText += "<td><button onclick=\"downloadDeleteButton(\'" + String(foundfile.name()) + "\', \'download\')\">Edit</button>";
      returnText += "<td><button class=danger onclick=\"downloadDeleteButton(\'" + String(foundfile.name()) + "\', \'delete\')\">Delete</button></tr>";
    } else {
      if (i>0) returnText += ",";
      returnText += "{\"name\":\"" + String(foundfile.name()) + "\", \"size\":" + foundfile.size() + "}\n";
    }
    i++;
    foundfile = root.openNextFile();
    yield();
  }
  if (ishtml) {
    returnText += "</table>";
  } else {
    returnText += "]}";
  }
  root.close();
  foundfile.close();
  //xSemaphoreGive( xSPISemaphore );
  _doLoop = true;
  return returnText;
}


// Make size of files human readable
// source: https://github.com/CelliesProjects/minimalUploadAuthESP32
String WebFSEditor::humanReadableSize(const size_t bytes) {
  if (bytes < 1024) return String(bytes) + " B";
  else if (bytes < (1024 * 1024)) return String(bytes / 1024.0) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) return String(bytes / 1024.0 / 1024.0) + " MB";
  else return String(bytes / 1024.0 / 1024.0 / 1024.0) + " GB";
}


namespace {
void sendEditorPage(AsyncWebServerRequest* request, int status, const uint8_t* data, size_t size) {
  auto* response = request->beginResponse_P(status, "text/html", data, size);
  response->addHeader("Content-Encoding", "gzip");
  request->send(response);
}
}

void WebFSEditor::configureWebServer(AsyncWebServer &server) {
  // configure web server

  // if url isn't found
  server.onNotFound([&](AsyncWebServerRequest * request) {
    notFound(request);
  });

  // run handleUpload function when any file is uploaded
  server.onFileUpload([&](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    handleUpload(request, filename, index, data, len, final);
  });

  // visiting this page will cause you to be logged out
  server.on("/logout", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->requestAuthentication();
    request->send(401);
  });

  // presents a "you are now logged out webpage
  server.on("/logged-out", HTTP_GET, [](AsyncWebServerRequest * request) {
    sendEditorPage(request, 401, logout_html, sizeof logout_html);
  });

  server.on("/", HTTP_GET, [&](AsyncWebServerRequest * request) {
    if (checkUserWebAuth(request)) {
      sendEditorPage(request, 200, index_html, sizeof index_html);
    } else {
      return request->requestAuthentication();
    }

  });

  server.on("/reboot", HTTP_GET, [&](AsyncWebServerRequest * request) {
    if (checkUserWebAuth(request)) {
      sendEditorPage(request, 200, reboot_html, sizeof reboot_html);
      //shouldReboot = true;
      bootRecovery::requestRestart();
      ESP.restart();
    } else {
      return request->requestAuthentication();
    }
  });

  server.on("/listfiles", HTTP_GET, [&](AsyncWebServerRequest * request)
  {
    if (checkUserWebAuth(request)) {
      boolean doHTML = !request->hasParam("json");
      request->send(200, doHTML ? "text/plain" : "application/json", listFiles(doHTML) );
    } else {
      return request->requestAuthentication();
    }
  });

  server.on("/file", HTTP_GET, [&](AsyncWebServerRequest * request) {
    if (checkUserWebAuth(request)) {
      _doLoop = false;
      if (useSemaphore) xSemaphoreTake( xSPISemaphore, portMAX_DELAY );

      if (request->hasParam("name") && request->hasParam("action")) {
        const char *fileName = request->getParam("name")->value().c_str();
        const char *fileAction = request->getParam("action")->value().c_str();

        String path;
        if (path.reserve(strlen(fileName) + 1)) {
          path = '/';
          path += fileName;
        }

        if (path.isEmpty()) {
          request->send(503, "text/plain", "ERROR: unable to allocate file path");
        } else if (!_fs.exists(path)) {
          request->send(400, "text/plain", "ERROR: file does not exist");
        } else {
          if (strcmp(fileAction, "download") == 0) {
            request->send(_fs, path, "text/plain");
          } else if (strcmp(fileAction, "delete") == 0) {
            if (!configTrial::writable(path.c_str())) {
              request->send(409, "text/plain", "Configuration trial file is protected");
            } else {
              if (bootRecovery::active()) bootRecovery::deferAutomaticRetry();
              _fs.remove(path);
              request->send(200, "text/plain", "Deleted File: " + String(fileName));
            }
          } else {
            request->send(400, "text/plain", "ERROR: invalid action param supplied");
          }
        }
      } else {
        request->send(400, "text/plain", "ERROR: name and action params required");
      }
      _doLoop = true;
      //xSemaphoreGive( xSPISemaphore );
    } else {
      return request->requestAuthentication();
    }
  });
}

void WebFSEditor::notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

// used by server.on functions to discern whether a user has the correct httpapitoken OR is authenticated by username and password
bool WebFSEditor::checkUserWebAuth(AsyncWebServerRequest * request) {
  bool isAuthenticated = false;
  //override
  return true;

  if (request->authenticate(httpuser.c_str(), httppassword.c_str())) {
    Log.noticeln("is authenticated via username and password");
    isAuthenticated = true;
  }
  return isAuthenticated;
}

// handles uploads to the filserver
void WebFSEditor::handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  // make sure authenticated before allowing upload
  if (checkUserWebAuth(request)) {
    const String path = "/" + filename;
    if (!configTrial::writable(path.c_str())) {
      if (final) request->send(409, "text/plain", "Configuration trial file is protected");
      return;
    }
    _doLoop = false;
    //Serial.println("take 2");
    if (useSemaphore) xSemaphoreTake( xSPISemaphore, portMAX_DELAY );
    //Serial.println("taken 2");

    if (!index) {
      if (bootRecovery::active()) bootRecovery::deferAutomaticRetry();
      // open the file on first call and store the file handle in the request object
      request->_tempFile = _fs.open(path, "w");
    }

    if (len) {
      // stream the incoming chunk to the opened file
      request->_tempFile.write(data, len);
    }

    if (final) {
      // close the file handle as the upload is now done
      request->_tempFile.close();
      request->redirect("/");
    }

    _doLoop = true;
    //Serial.println("give 2");
    //xSemaphoreGive( xSPISemaphore );
  } else {
    Log.noticeln("Auth: Failed");
    return request->requestAuthentication();
  }
}
