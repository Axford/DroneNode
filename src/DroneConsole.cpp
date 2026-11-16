#include "DroneConsole.h"
DiagnosticConsole DroneConsole;
void DiagnosticConsole::begin() {
  if(!mutex) mutex=xSemaphoreCreateMutexStatic(&mutexStorage);
}
void DiagnosticConsole::reportContention() {
  if(contended.exchange(false)) writer.noteLoss();
}
void DiagnosticConsole::telemetry(bool enabled) {
  begin();
  if(xSemaphoreTake(mutex,portMAX_DELAY)==pdTRUE) {
    if(framed!=enabled) {
      Serial.flush();
      writer.bind(enabled? &wire:nullptr);
      framed=enabled;
      contended=false;
    }
    xSemaphoreGive(mutex);
  }
}
size_t DiagnosticConsole::write(const uint8_t* bytes,size_t size) {
  if(!mutex) return Serial.write(bytes,size); // Before system initialization only.
  if(xSemaphoreTake(mutex,framed.load()?0:portMAX_DELAY)!=pdTRUE) {contended=true;return size;}
  reportContention();
  size_t result=size;
  if(framed) writer.log(bytes,size);
  else result=Serial.write(bytes,size);
  xSemaphoreGive(mutex);
  return result;
}
bool DiagnosticConsole::sendMesh(const uint8_t* bytes) {
  if(!mutex || xSemaphoreTake(mutex,portMAX_DELAY)!=pdTRUE) return false;
  reportContention();
  bool result=framed && writer.sendMesh(bytes);
  if(result) Serial.flush();
  xSemaphoreGive(mutex);
  return result;
}
void DiagnosticConsole::service() {
  if(!mutex || xSemaphoreTake(mutex,0)!=pdTRUE) return;
  reportContention();
  if(framed) writer.service();
  xSemaphoreGive(mutex);
}
