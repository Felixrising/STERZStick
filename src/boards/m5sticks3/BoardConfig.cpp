#include "boards/BoardConfig.h"

namespace board {

#ifdef BOARD_M5STICKS3
const BoardConfig& current() {
  // Scaffold values for M5StickS3 support.
  // TODO: validate all pins and wake GPIO assignments on hardware.
  static const BoardConfig cfg{
      19,  // ledPin
      4,   // holdPin
      14,  // buttonAPin
      0,   // buttonBPin
      46,  // buttonCPin
      14,  // ext0WakeGpio
      46   // ext1WakeGpio
  };
  return cfg;
}
#endif

}  // namespace board
