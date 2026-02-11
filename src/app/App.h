#pragma once

class App {
 public:
  static App& instance();
  void setup();
  void loop();

 private:
  App() = default;
};
