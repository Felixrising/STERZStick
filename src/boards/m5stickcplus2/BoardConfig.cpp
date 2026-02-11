#include "boards/BoardConfig.h"

namespace board {

#ifdef BOARD_M5STICKCPLUS2
const BoardConfig& current() {
  static const BoardConfig cfg{
      19,  // ledPin
      4,   // holdPin
      37,  // buttonAPin
      39,  // buttonBPin
      35,  // buttonCPin
      37,  // ext0WakeGpio
      35   // ext1WakeGpio
  };
  return cfg;
}
#endif

}  // namespace board
