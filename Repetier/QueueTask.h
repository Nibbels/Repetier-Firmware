#ifndef QUEUE_TASK_H
#define QUEUE_TASK_H

#include <stdint.h>

// Commands in a buffered PrintLine; these values are part of the executor dispatch.
enum class QueueTask : uint8_t {
    None = 0,
    EnableZCompensation = 1,
    DisableZCompensation = 2,
    Pause = 4,
    PauseAndPark = 5,
    EnableSenseOffset = 100,
};
static_assert(sizeof(QueueTask) == 1, "Queue command must fit the shared move byte");

#endif
