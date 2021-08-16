#pragma once

#include <keyboard_client/cl_keyboard.h>
#include <smacc/smacc_orthogonal.h>
namespace sm_ferrari
{
class OrKeyboard : public smacc::Orthogonal<OrKeyboard>
{
public:
  void onInitialize() override { auto clKeyboard = this->createClient<cl_keyboard::ClKeyboard>(); }
};
}  // namespace sm_ferrari
