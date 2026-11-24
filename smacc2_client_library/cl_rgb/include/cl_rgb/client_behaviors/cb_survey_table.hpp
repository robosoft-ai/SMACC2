// Copyright 2026 RobosoftAI Inc.
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

#pragma once

#include <cl_rgb/client_behaviors/cb_rgb_client_behavior_base.hpp>
#include <cl_rgb/table_survey.hpp>

#include <string>

namespace cl_rgb
{

struct SurveyTableParams
{
  std::string camera = "cam_head";
  BlobParams blobs;
  int minBlobs = 0;    // fewer than this is a failure
  double waitS = 5.0;  // wall time to wait for a first image, its calibration and its pose
};

// Looks at the table once and records what lies on it: the blobs of the latest image, each
// with its place on the table, its size, its direction and its colour. The result is kept
// by CpSceneSurvey. Success when at least minBlobs were found, failure otherwise.
class CbSurveyTable : public CbRgbClientBehaviorBase
{
public:
  explicit CbSurveyTable(const SurveyTableParams & params = SurveyTableParams());

  void setParams(const SurveyTableParams & params) { params_ = params; }

  void onEntry() override;
  void onExit() override {}

private:
  SurveyTableParams params_;
};

}  // namespace cl_rgb
