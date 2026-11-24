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

#include <smacc2/smacc.hpp>

#include <cl_rgb/components/cp_camera.hpp>
#include <cl_rgb/components/cp_scene_survey.hpp>
#include <cl_rgb/components/cp_table_plane.hpp>
#include <cl_rgb/components/cp_table_segmenter.hpp>
#include <cl_rgb/components/cp_transforms.hpp>

#include <vector>

namespace cl_rgb
{

struct RgbParams
{
  std::vector<CameraParams> cameras;  // one CpCamera is created per entry, under its name
  TableParams table;
  SegmenterParams segmenter;
};

// Vision from colour images alone. The client only assembles the components: the cameras,
// the table they look at, the segmenter and the place where a survey's result is kept.
class ClRgb : public smacc2::ISmaccClient
{
public:
  explicit ClRgb(const RgbParams & params);
  ~ClRgb() override;

  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    // the transform tree first: every camera resolves it in onInitialize
    this->createComponent<CpTransforms, TOrthogonal, TClient>();
    for (const auto & camera : params_.cameras)
    {
      this->createNamedComponent<CpCamera, TOrthogonal, TClient>(camera.name, camera);
    }
    this->createComponent<CpTablePlane, TOrthogonal, TClient>(params_.table);
    this->createComponent<CpTableSegmenter, TOrthogonal, TClient>(params_.segmenter);
    this->createComponent<CpSceneSurvey, TOrthogonal, TClient>();
  }

private:
  RgbParams params_;
};

}  // namespace cl_rgb
