#include <keyboard_client/cl_keyboard.h>

namespace cl_keyboard {
ClKeyboard::ClKeyboard() {
  initialized_ = false;
  topicName = "/keyboard_unicode";
}

ClKeyboard::~ClKeyboard() 
{
  
}

void ClKeyboard::initialize() {

  SmaccSubscriberClient<std_msgs::msg::UInt16>::initialize();

  if (!this->initialized_) {
    this->onMessageReceived(&ClKeyboard::onKeyboardMessage, this);
    this->initialized_ = true;
  }
}

void ClKeyboard::onKeyboardMessage(const std_msgs::msg::UInt16 &unicode_keychar) {
  
  postEventKeyPress(unicode_keychar);
}
} // namespace cl_keyboard