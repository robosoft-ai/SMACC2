#pragma once

#include <smacc2/smacc.hpp>
#include <cl_px4_mr/cl_px4_mr.hpp>

namespace sm_cl_px4_mr_test_1
{

class OrPx4 : public smacc2::Orthogonal<OrPx4>
{
public:
  void onInitialize() override
  {
    auto client = this->createClient<cl_px4_mr::ClPx4Mr>();
  }
};

}  // namespace sm_cl_px4_mr_test_1
