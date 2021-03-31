/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include "cb_circular_pivot_motion.h"

namespace cl_move_group_interface
{
    class CbEndEffectorRotate : public CbCircularPivotMotion
    {
    public:
        CbEndEffectorRotate(double deltaRadians, std::string tipLink = "");
        virtual ~CbEndEffectorRotate();

        virtual void onEntry() override;
    };

} // namespace cl_move_group_interface