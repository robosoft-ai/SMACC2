#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.h>

namespace cl_keyboard
{
void CbDefaultKeyboardBehavior::onEntry()
{
  this->requiresClient(ClKeyboard_);
  this->ClKeyboard_->OnKeyPress(&CbDefaultKeyboardBehavior::OnKeyPress, this);
}

void CbDefaultKeyboardBehavior::OnKeyPress(char character) { postEventKeyPress(character); }
}  // namespace cl_keyboard