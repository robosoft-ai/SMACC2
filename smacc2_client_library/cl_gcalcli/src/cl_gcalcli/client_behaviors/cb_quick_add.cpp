// Copyright 2024 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cl_gcalcli/client_behaviors/cb_quick_add.hpp>

namespace cl_gcalcli
{

CbQuickAdd::CbQuickAdd(const std::string & text, const std::string & calendar)
: text_(text), calendar_(calendar), client_(nullptr)
{
}

void CbQuickAdd::onEntry()
{
  RCLCPP_INFO(getLogger(), "[CbQuickAdd] Adding event: %s", text_.c_str());

  this->requiresClient(client_);

  if (!client_)
  {
    RCLCPP_ERROR(getLogger(), "[CbQuickAdd] ClGcalcli client not available");
    this->postFailureEvent();
    return;
  }

  auto * connection = client_->getConnection();
  if (!connection)
  {
    RCLCPP_ERROR(getLogger(), "[CbQuickAdd] CpGcalcliConnection not available");
    this->postFailureEvent();
    return;
  }

  // Build the quick add command
  std::string args = "quick";

  if (!calendar_.empty())
  {
    args += " --calendar \"" + calendar_ + "\"";
  }

  // Escape the text for shell
  std::string escaped_text = text_;
  // Replace double quotes with escaped quotes
  size_t pos = 0;
  while ((pos = escaped_text.find('"', pos)) != std::string::npos)
  {
    escaped_text.replace(pos, 1, "\\\"");
    pos += 2;
  }

  args += " \"" + escaped_text + "\"";

  auto result = connection->executeGcalcli(args, 30000);

  if (result.exit_code == 0 && !result.timed_out)
  {
    RCLCPP_INFO(getLogger(), "[CbQuickAdd] Event added successfully");
    this->postSuccessEvent();
  }
  else
  {
    RCLCPP_ERROR(getLogger(), "[CbQuickAdd] Failed to add event: %s", result.stdout_output.c_str());
    this->postFailureEvent();
  }
}

}  // namespace cl_gcalcli
