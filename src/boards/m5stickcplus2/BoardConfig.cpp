#include "boards/BoardConfig.h"

namespace board {

#ifdef BOARD_M5STICKCPLUS2
const BoardConfig& current() {
  static const BoardConfig cfg{
      19,    // ledPin        (red LED / IR on G19)
      4,     // holdPin       (GPIO4 keeps power on)
      37,    // buttonAPin    (front button)
      39,    // buttonBPin    (top button)
      35,    // buttonCPin    (power button, readable GPIO)
      37,    // ext0WakeGpio  (Button A for ext0 wake)
      35,    // ext1WakeGpio  (Button C for ext1 wake)
      3,     // buttonCount
      true,  // hasHoldPin
      false, // hasPmicPowerOff
      true,  // hasRtcWake
      128    // imuCalibStrength (MPU6886)
  };
  return cfg;
}
#endif

}  // namespace board
