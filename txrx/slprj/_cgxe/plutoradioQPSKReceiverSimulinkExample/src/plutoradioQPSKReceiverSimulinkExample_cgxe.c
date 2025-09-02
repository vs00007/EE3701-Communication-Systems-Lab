/* Include files */

#include "plutoradioQPSKReceiverSimulinkExample_cgxe.h"
#include "m_UIzktXqQH1UJHNoLKJcwx.h"

unsigned int cgxe_plutoradioQPSKReceiverSimulinkExample_method_dispatcher
  (SimStruct* S, int_T method, void* data)
{
  if (ssGetChecksum0(S) == 2136401846 &&
      ssGetChecksum1(S) == 1022578848 &&
      ssGetChecksum2(S) == 306712915 &&
      ssGetChecksum3(S) == 955780795) {
    method_dispatcher_UIzktXqQH1UJHNoLKJcwx(S, method, data);
    return 1;
  }

  return 0;
}
