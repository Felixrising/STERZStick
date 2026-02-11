#include "app/App.h"

// Transitional app entry points while refactor proceeds.
extern void firmwareSetup();
extern void firmwareLoop();

App& App::instance() {
  static App app;
  return app;
}

void App::setup() {
  firmwareSetup();
}

void App::loop() {
  firmwareLoop();
}
