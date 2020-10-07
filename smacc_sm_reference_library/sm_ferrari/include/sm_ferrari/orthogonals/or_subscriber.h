#pragma once

#include <sm_ferrari/clients/cl_subscriber/cl_subscriber.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_ferrari
{
using namespace sm_ferrari::cl_subscriber;

class OrSubscriber : public smacc::Orthogonal<OrSubscriber>
{
public:
    virtual void onInitialize() override
    {
        auto subscriber_client = this->createClient<ClSubscriber>("/temperature");
        subscriber_client->initialize();
    }
};
} // namespace sm_ferrari