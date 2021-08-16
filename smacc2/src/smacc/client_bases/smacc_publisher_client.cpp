/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <smacc/client_bases/smacc_publisher_client.h>

namespace smacc
{
namespace client_bases
{
SmaccPublisherClient::SmaccPublisherClient() { initialized_ = false; }

SmaccPublisherClient::~SmaccPublisherClient() {}
}  // namespace client_bases
}  // namespace smacc