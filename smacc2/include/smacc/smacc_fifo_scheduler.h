#include <boost/statechart/fifo_scheduler.hpp>
#include <smacc/smacc_fifo_worker.h>

typedef boost::statechart::fifo_scheduler<SmaccFifoWorker, SmaccAllocator> SmaccFifoScheduler;