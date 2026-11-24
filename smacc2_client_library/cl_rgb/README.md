# cl_rgb

Vision from color images alone, for SMACC2: no depth image, no object model. A calibrated camera and a
table of known height turn a pixel into a point in the world; what does not look like table is grouped
into blobs, each with a place, a size, a direction and a color.

## Components

| Component | What it does |
|--|--|
| `CpTransforms` | the transform tree, shared by the cameras |
| `CpCamera` | one per camera, created under the camera's name: the latest image, the calibration, and the camera's pose at the time of that image. Signal `onFrame_` |
| `CpTablePlane` | the table's height and extent: the world point under a pixel, the table's region of an image |
| `CpTableSegmenter` | the pixels of the table's region that are not table |
| `CpSceneSurvey` | the result of the last survey. Signal `onSurvey_` |

## Client behaviors

| Behavior | Succeeds | Fails |
|--|--|--|
| `CbWaitForFrame` | when the camera has delivered new images since the state was entered | on its time-out, if one was set |
| `CbSurveyTable` | when the latest image shows at least `minBlobs` blobs; the blobs are stored in `CpSceneSurvey` | with fewer blobs, or with no usable image |

Both derive from `CbRgbClientBehaviorBase`, which resolves the components on the state machine thread.

## Use

```cpp
class OrRgb : public smacc2::Orthogonal<OrRgb>
{
public:
  void onInitialize() override
  {
    cl_rgb::RgbParams params;
    cl_rgb::CameraParams head;
    head.name = "cam_head";
    head.imageTopic = "/robodojo/cam_head/image_raw";
    head.infoTopic = "/robodojo/cam_head/camera_info";
    params.cameras.push_back(head);
    params.table.height = 0.765;  // m, in the cameras' world frame
    params.table.xMin = -0.7;
    params.table.xMax = 0.7;
    params.table.yMin = -0.6;
    params.table.yMax = 0.5;
    this->createClient<cl_rgb::ClRgb>(params);
  }
};
```

The cameras' poses are read from `/tf` at the time of each image, so a camera on a moving arm needs no
special handling. A complete machine: `sm_robodojo_survey_1` in `robodojo_sm_library`.

## What a blob's position means

`Blob::tablePoint` is where the ray through the blob's center meets the table. An object with height
is seen in front of the table behind it, so this point lies beyond the object, by up to half its
height times the tangent of the viewing angle. With the object's height known,
`CpTablePlane::pointUnderPixel(camera, u, v, height / 2)` gives its middle.

## Functions for what the blobs are

Plain functions, usable from a state or a behavior once a survey exists.

| Function | What it does |
|--|--|
| `colourName(lab)` | the everyday word for a colour, by hue |
| `colourCost(lab, name)` | how far a colour is from a colour word |
| `matchColours(seen, names)` | gives each named colour one of the seen colours, none twice, at the least total cost: for a scene whose colours are known by name beforehand |
| `fitBoxOnTable(blob, camera, tableZ, params)` | the place and the turn of a box of known size whose outline, drawn through the camera, covers the blob best. Falls back to the top face alone when the sides are missing from the blob |

Switches that are off unless set, each for a case met in use:

| Setting | For |
|--|--|
| `SegmenterParams::shadowDarkest` | against a reference picture, the table dimmed alike in all channels is shadow, not object |
| `BlobParams::colourEdgeHueDeg` | objects of different hue that touch in the image are cut apart |
| `BoxFitParams::minChromaFraction` | a box of one strong colour: dark, weakly coloured pixels (its tinted shadow) are left out of the fit |

## Tests

`colcon test --packages-select cl_rgb`: the geometry against a recorded scene, the survey on drawn
images.
