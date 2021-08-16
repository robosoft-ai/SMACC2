#pragma once
#include <boost/statechart/fifo_worker.hpp>

// typedef std::allocator<void> SmaccAllocator;
typedef std::allocator<boost::statechart::none> SmaccAllocator;
typedef boost::statechart::fifo_worker<SmaccAllocator> SmaccFifoWorker;
