/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <smacc/smacc_client_behavior_base.h>

namespace smacc
{
    class SmaccClientBehavior : public ISmaccClientBehavior
    {
    public:
        virtual void onEntry() override;
        virtual void onExit() override;
    };
} // namespace smacc

#include <smacc/impl/smacc_client_behavior_impl.h>