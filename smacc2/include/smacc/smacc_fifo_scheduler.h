#include <smacc/smacc_fifo_worker.h>
#include <boost/statechart/fifo_scheduler.hpp>

typedef boost::statechart::fifo_scheduler<SmaccFifoWorker, SmaccAllocator> SmaccFifoScheduler;