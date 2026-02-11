#pragma once

#include <stdint.h>

namespace board {

struct BoardConfig {
  int ledPin;
  int holdPin;
  int buttonAPin;
  int buttonBPin;
  int buttonCPin;
  int ext0WakeGpio;
  int ext1WakeGpio;
};

const BoardConfig& current();

}  // namespace board
