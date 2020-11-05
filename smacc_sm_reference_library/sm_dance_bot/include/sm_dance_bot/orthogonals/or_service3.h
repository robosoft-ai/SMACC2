#pragma once

#include <smacc/smacc_orthogonal.h>
#include <sm_dance_bot/clients/cl_service3/cl_service3.h>

namespace sm_dance_bot
{
class OrService3 : public smacc::Orthogonal<OrService3>
{
public:
    virtual void onInitialize() override
    {
        auto serviceClient = this->createClient<ClService3>();
        serviceClient->serviceName_ = "/service_node3";
        serviceClient->initialize();
    }
};
} // namespace sm_dance_bot