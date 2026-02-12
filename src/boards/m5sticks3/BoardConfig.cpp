#include "boards/BoardConfig.h"

namespace board {

#ifdef BOARD_M5STICKS3
const BoardConfig& current() {
  static const BoardConfig cfg{
      -1,    // ledPin        (no user-addressable LED GPIO; M5PM1 controls LED)
      -1,    // holdPin       (PMIC manages power, no hold pin)
      11,    // buttonAPin    (KEY1 = G11)
      12,    // buttonBPin    (KEY2 = G12)
      -1,    // buttonCPin    (power button is PMIC-managed, not a GPIO)
      -1,    // ext0WakeGpio  (uses M5PM1 wake, not ESP32 ext wake)
      -1,    // ext1WakeGpio
      2,     // buttonCount
      false, // hasHoldPin
      true,  // hasPmicPowerOff
      false, // hasRtcWake
      128    // imuCalibStrength (BMI270 -- needs hardware validation)
  };
  return cfg;
}
#endif

}  // namespace board
