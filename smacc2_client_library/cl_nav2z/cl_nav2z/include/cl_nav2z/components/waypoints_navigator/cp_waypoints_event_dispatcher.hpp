// Copyright 2021 RobosoftAI Inc.
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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <cl_nav2z/cl_nav2z.hpp>
#include <smacc2/smacc.hpp>

namespace cl_nav2z

/// @cond
// The purpose of the cond and endcond tags seen above and below in this file are there to prevent doxygen from indexing the 256 events
// below which makes the documentation produced unuseable.

{
template <typename TSource, typename TOrthogonal>
struct EvWaypoint0 : sc::event<EvWaypoint0<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint1 : sc::event<EvWaypoint1<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint2 : sc::event<EvWaypoint2<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint3 : sc::event<EvWaypoint3<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint4 : sc::event<EvWaypoint4<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint5 : sc::event<EvWaypoint5<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint6 : sc::event<EvWaypoint6<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint7 : sc::event<EvWaypoint7<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint8 : sc::event<EvWaypoint8<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint9 : sc::event<EvWaypoint9<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint10 : sc::event<EvWaypoint10<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint11 : sc::event<EvWaypoint11<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint12 : sc::event<EvWaypoint12<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint13 : sc::event<EvWaypoint13<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint14 : sc::event<EvWaypoint14<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint15 : sc::event<EvWaypoint15<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint16 : sc::event<EvWaypoint16<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint17 : sc::event<EvWaypoint17<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint18 : sc::event<EvWaypoint18<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint19 : sc::event<EvWaypoint19<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint20 : sc::event<EvWaypoint20<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint21 : sc::event<EvWaypoint21<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint22 : sc::event<EvWaypoint22<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint23 : sc::event<EvWaypoint23<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint24 : sc::event<EvWaypoint24<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint25 : sc::event<EvWaypoint25<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint26 : sc::event<EvWaypoint26<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint27 : sc::event<EvWaypoint27<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint28 : sc::event<EvWaypoint28<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint29 : sc::event<EvWaypoint29<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint30 : sc::event<EvWaypoint30<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint31 : sc::event<EvWaypoint31<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint32 : sc::event<EvWaypoint32<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint33 : sc::event<EvWaypoint33<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint34 : sc::event<EvWaypoint34<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint35 : sc::event<EvWaypoint35<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint36 : sc::event<EvWaypoint36<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint37 : sc::event<EvWaypoint37<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint38 : sc::event<EvWaypoint38<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint39 : sc::event<EvWaypoint39<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint40 : sc::event<EvWaypoint40<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint41 : sc::event<EvWaypoint41<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint42 : sc::event<EvWaypoint42<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint43 : sc::event<EvWaypoint43<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint44 : sc::event<EvWaypoint44<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint45 : sc::event<EvWaypoint45<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint46 : sc::event<EvWaypoint46<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint47 : sc::event<EvWaypoint47<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint48 : sc::event<EvWaypoint48<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint49 : sc::event<EvWaypoint49<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint50 : sc::event<EvWaypoint50<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint51 : sc::event<EvWaypoint51<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint52 : sc::event<EvWaypoint52<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint53 : sc::event<EvWaypoint53<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint54 : sc::event<EvWaypoint54<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint55 : sc::event<EvWaypoint55<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint56 : sc::event<EvWaypoint56<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint57 : sc::event<EvWaypoint57<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint58 : sc::event<EvWaypoint58<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint59 : sc::event<EvWaypoint59<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint60 : sc::event<EvWaypoint60<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint61 : sc::event<EvWaypoint61<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint62 : sc::event<EvWaypoint62<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint63 : sc::event<EvWaypoint63<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint64 : sc::event<EvWaypoint64<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint65 : sc::event<EvWaypoint65<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint66 : sc::event<EvWaypoint66<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint67 : sc::event<EvWaypoint67<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint68 : sc::event<EvWaypoint68<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint69 : sc::event<EvWaypoint69<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint70 : sc::event<EvWaypoint70<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint71 : sc::event<EvWaypoint71<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint72 : sc::event<EvWaypoint72<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint73 : sc::event<EvWaypoint73<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint74 : sc::event<EvWaypoint74<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint75 : sc::event<EvWaypoint75<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint76 : sc::event<EvWaypoint76<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint77 : sc::event<EvWaypoint77<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint78 : sc::event<EvWaypoint78<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint79 : sc::event<EvWaypoint79<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint80 : sc::event<EvWaypoint80<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint81 : sc::event<EvWaypoint81<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint82 : sc::event<EvWaypoint82<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint83 : sc::event<EvWaypoint83<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint84 : sc::event<EvWaypoint84<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint85 : sc::event<EvWaypoint85<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint86 : sc::event<EvWaypoint86<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint87 : sc::event<EvWaypoint87<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint88 : sc::event<EvWaypoint88<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint89 : sc::event<EvWaypoint89<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint90 : sc::event<EvWaypoint90<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint91 : sc::event<EvWaypoint91<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint92 : sc::event<EvWaypoint92<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint93 : sc::event<EvWaypoint93<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint94 : sc::event<EvWaypoint94<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint95 : sc::event<EvWaypoint95<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint96 : sc::event<EvWaypoint96<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint97 : sc::event<EvWaypoint97<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint98 : sc::event<EvWaypoint98<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint99 : sc::event<EvWaypoint99<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint100 : sc::event<EvWaypoint100<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint101 : sc::event<EvWaypoint101<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint102 : sc::event<EvWaypoint102<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint103 : sc::event<EvWaypoint103<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint104 : sc::event<EvWaypoint104<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint105 : sc::event<EvWaypoint105<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint106 : sc::event<EvWaypoint106<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint107 : sc::event<EvWaypoint107<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint108 : sc::event<EvWaypoint108<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint109 : sc::event<EvWaypoint109<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint110 : sc::event<EvWaypoint110<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint111 : sc::event<EvWaypoint111<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint112 : sc::event<EvWaypoint112<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint113 : sc::event<EvWaypoint113<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint114 : sc::event<EvWaypoint114<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint115 : sc::event<EvWaypoint115<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint116 : sc::event<EvWaypoint116<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint117 : sc::event<EvWaypoint117<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint118 : sc::event<EvWaypoint118<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint119 : sc::event<EvWaypoint119<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint120 : sc::event<EvWaypoint120<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint121 : sc::event<EvWaypoint121<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint122 : sc::event<EvWaypoint122<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint123 : sc::event<EvWaypoint123<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint124 : sc::event<EvWaypoint124<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint125 : sc::event<EvWaypoint125<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint126 : sc::event<EvWaypoint126<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint127 : sc::event<EvWaypoint127<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint128 : sc::event<EvWaypoint128<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint129 : sc::event<EvWaypoint129<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint130 : sc::event<EvWaypoint130<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint131 : sc::event<EvWaypoint131<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint132 : sc::event<EvWaypoint132<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint133 : sc::event<EvWaypoint133<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint134 : sc::event<EvWaypoint134<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint135 : sc::event<EvWaypoint135<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint136 : sc::event<EvWaypoint136<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint137 : sc::event<EvWaypoint137<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint138 : sc::event<EvWaypoint138<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint139 : sc::event<EvWaypoint139<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint140 : sc::event<EvWaypoint140<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint141 : sc::event<EvWaypoint141<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint142 : sc::event<EvWaypoint142<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint143 : sc::event<EvWaypoint143<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint144 : sc::event<EvWaypoint144<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint145 : sc::event<EvWaypoint145<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint146 : sc::event<EvWaypoint146<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint147 : sc::event<EvWaypoint147<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint148 : sc::event<EvWaypoint148<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint149 : sc::event<EvWaypoint149<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint150 : sc::event<EvWaypoint150<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint151 : sc::event<EvWaypoint151<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint152 : sc::event<EvWaypoint152<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint153 : sc::event<EvWaypoint153<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint154 : sc::event<EvWaypoint154<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint155 : sc::event<EvWaypoint155<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint156 : sc::event<EvWaypoint156<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint157 : sc::event<EvWaypoint157<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint158 : sc::event<EvWaypoint158<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint159 : sc::event<EvWaypoint159<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint160 : sc::event<EvWaypoint160<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint161 : sc::event<EvWaypoint161<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint162 : sc::event<EvWaypoint162<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint163 : sc::event<EvWaypoint163<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint164 : sc::event<EvWaypoint164<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint165 : sc::event<EvWaypoint165<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint166 : sc::event<EvWaypoint166<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint167 : sc::event<EvWaypoint167<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint168 : sc::event<EvWaypoint168<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint169 : sc::event<EvWaypoint169<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint170 : sc::event<EvWaypoint170<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint171 : sc::event<EvWaypoint171<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint172 : sc::event<EvWaypoint172<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint173 : sc::event<EvWaypoint173<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint174 : sc::event<EvWaypoint174<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint175 : sc::event<EvWaypoint175<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint176 : sc::event<EvWaypoint176<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint177 : sc::event<EvWaypoint177<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint178 : sc::event<EvWaypoint178<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint179 : sc::event<EvWaypoint179<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint180 : sc::event<EvWaypoint180<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint181 : sc::event<EvWaypoint181<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint182 : sc::event<EvWaypoint182<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint183 : sc::event<EvWaypoint183<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint184 : sc::event<EvWaypoint184<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint185 : sc::event<EvWaypoint185<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint186 : sc::event<EvWaypoint186<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint187 : sc::event<EvWaypoint187<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint188 : sc::event<EvWaypoint188<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint189 : sc::event<EvWaypoint189<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint190 : sc::event<EvWaypoint190<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint191 : sc::event<EvWaypoint191<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint192 : sc::event<EvWaypoint192<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint193 : sc::event<EvWaypoint193<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint194 : sc::event<EvWaypoint194<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint195 : sc::event<EvWaypoint195<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint196 : sc::event<EvWaypoint196<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint197 : sc::event<EvWaypoint197<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint198 : sc::event<EvWaypoint198<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint199 : sc::event<EvWaypoint199<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint200 : sc::event<EvWaypoint200<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint201 : sc::event<EvWaypoint201<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint202 : sc::event<EvWaypoint202<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint203 : sc::event<EvWaypoint203<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint204 : sc::event<EvWaypoint204<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint205 : sc::event<EvWaypoint205<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint206 : sc::event<EvWaypoint206<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint207 : sc::event<EvWaypoint207<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint208 : sc::event<EvWaypoint208<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint209 : sc::event<EvWaypoint209<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint210 : sc::event<EvWaypoint210<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint211 : sc::event<EvWaypoint211<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint212 : sc::event<EvWaypoint212<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint213 : sc::event<EvWaypoint213<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint214 : sc::event<EvWaypoint214<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint215 : sc::event<EvWaypoint215<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint216 : sc::event<EvWaypoint216<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint217 : sc::event<EvWaypoint217<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint218 : sc::event<EvWaypoint218<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint219 : sc::event<EvWaypoint219<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint220 : sc::event<EvWaypoint220<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint221 : sc::event<EvWaypoint221<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint222 : sc::event<EvWaypoint222<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint223 : sc::event<EvWaypoint223<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint224 : sc::event<EvWaypoint224<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint225 : sc::event<EvWaypoint225<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint226 : sc::event<EvWaypoint226<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint227 : sc::event<EvWaypoint227<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint228 : sc::event<EvWaypoint228<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint229 : sc::event<EvWaypoint229<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint230 : sc::event<EvWaypoint230<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint231 : sc::event<EvWaypoint231<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint232 : sc::event<EvWaypoint232<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint233 : sc::event<EvWaypoint233<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint234 : sc::event<EvWaypoint234<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint235 : sc::event<EvWaypoint235<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint236 : sc::event<EvWaypoint236<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint237 : sc::event<EvWaypoint237<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint238 : sc::event<EvWaypoint238<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint239 : sc::event<EvWaypoint239<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint240 : sc::event<EvWaypoint240<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint241 : sc::event<EvWaypoint241<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint242 : sc::event<EvWaypoint242<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint243 : sc::event<EvWaypoint243<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint244 : sc::event<EvWaypoint244<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint245 : sc::event<EvWaypoint245<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint246 : sc::event<EvWaypoint246<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint247 : sc::event<EvWaypoint247<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint248 : sc::event<EvWaypoint248<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint249 : sc::event<EvWaypoint249<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint250 : sc::event<EvWaypoint250<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint251 : sc::event<EvWaypoint251<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint252 : sc::event<EvWaypoint252<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint253 : sc::event<EvWaypoint253<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint254 : sc::event<EvWaypoint254<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint255 : sc::event<EvWaypoint255<TSource, TOrthogonal>>
{
  int waypointIndex;
};

template <typename TSource, typename TOrthogonal>
struct EvWaypoint256 : sc::event<EvWaypoint256<TSource, TOrthogonal>>
{
  int waypointIndex;
};

// Doxygen endcond tag
/// @endcond

class ClNav2Z;

#define WAYPOINTS_EVENTCOUNT 257

class WaypointEventDispatcher
{
public:
  ClNav2Z * client_ = nullptr;
  std::function<void()> postWaypointFn[WAYPOINTS_EVENTCOUNT];

  template <typename TDerived, typename TOrthogonal>
  void initialize(ClNav2Z * client);

  void postWaypointEvent(int index);
};

template <typename TEv>
void configurePostEvWaypoint(
  std::function<void()> * fntarget, WaypointEventDispatcher * thisobj, int index)
{
  thisobj->postWaypointFn[index] = [thisobj]() { thisobj->client_->template postEvent<TEv>(); };
}

template <typename TDerived, typename TOrthogonal>
void WaypointEventDispatcher::initialize(ClNav2Z * client)
{
  // Doygen cond tag
  /// @cond
  this->client_ = client;
  configurePostEvWaypoint<EvWaypoint0<TDerived, TOrthogonal>>(postWaypointFn, this, 0);
  configurePostEvWaypoint<EvWaypoint1<TDerived, TOrthogonal>>(postWaypointFn, this, 1);
  configurePostEvWaypoint<EvWaypoint2<TDerived, TOrthogonal>>(postWaypointFn, this, 2);
  configurePostEvWaypoint<EvWaypoint3<TDerived, TOrthogonal>>(postWaypointFn, this, 3);
  configurePostEvWaypoint<EvWaypoint4<TDerived, TOrthogonal>>(postWaypointFn, this, 4);
  configurePostEvWaypoint<EvWaypoint5<TDerived, TOrthogonal>>(postWaypointFn, this, 5);
  configurePostEvWaypoint<EvWaypoint6<TDerived, TOrthogonal>>(postWaypointFn, this, 6);
  configurePostEvWaypoint<EvWaypoint7<TDerived, TOrthogonal>>(postWaypointFn, this, 7);
  configurePostEvWaypoint<EvWaypoint8<TDerived, TOrthogonal>>(postWaypointFn, this, 8);
  configurePostEvWaypoint<EvWaypoint9<TDerived, TOrthogonal>>(postWaypointFn, this, 9);
  configurePostEvWaypoint<EvWaypoint10<TDerived, TOrthogonal>>(postWaypointFn, this, 10);
  configurePostEvWaypoint<EvWaypoint11<TDerived, TOrthogonal>>(postWaypointFn, this, 11);
  configurePostEvWaypoint<EvWaypoint12<TDerived, TOrthogonal>>(postWaypointFn, this, 12);
  configurePostEvWaypoint<EvWaypoint13<TDerived, TOrthogonal>>(postWaypointFn, this, 13);
  configurePostEvWaypoint<EvWaypoint14<TDerived, TOrthogonal>>(postWaypointFn, this, 14);
  configurePostEvWaypoint<EvWaypoint15<TDerived, TOrthogonal>>(postWaypointFn, this, 15);
  configurePostEvWaypoint<EvWaypoint16<TDerived, TOrthogonal>>(postWaypointFn, this, 16);
  configurePostEvWaypoint<EvWaypoint17<TDerived, TOrthogonal>>(postWaypointFn, this, 17);
  configurePostEvWaypoint<EvWaypoint18<TDerived, TOrthogonal>>(postWaypointFn, this, 18);
  configurePostEvWaypoint<EvWaypoint19<TDerived, TOrthogonal>>(postWaypointFn, this, 19);
  configurePostEvWaypoint<EvWaypoint20<TDerived, TOrthogonal>>(postWaypointFn, this, 20);
  configurePostEvWaypoint<EvWaypoint21<TDerived, TOrthogonal>>(postWaypointFn, this, 21);
  configurePostEvWaypoint<EvWaypoint22<TDerived, TOrthogonal>>(postWaypointFn, this, 22);
  configurePostEvWaypoint<EvWaypoint23<TDerived, TOrthogonal>>(postWaypointFn, this, 23);
  configurePostEvWaypoint<EvWaypoint24<TDerived, TOrthogonal>>(postWaypointFn, this, 24);
  configurePostEvWaypoint<EvWaypoint25<TDerived, TOrthogonal>>(postWaypointFn, this, 25);
  configurePostEvWaypoint<EvWaypoint26<TDerived, TOrthogonal>>(postWaypointFn, this, 26);
  configurePostEvWaypoint<EvWaypoint27<TDerived, TOrthogonal>>(postWaypointFn, this, 27);
  configurePostEvWaypoint<EvWaypoint28<TDerived, TOrthogonal>>(postWaypointFn, this, 28);
  configurePostEvWaypoint<EvWaypoint29<TDerived, TOrthogonal>>(postWaypointFn, this, 29);
  configurePostEvWaypoint<EvWaypoint30<TDerived, TOrthogonal>>(postWaypointFn, this, 30);
  configurePostEvWaypoint<EvWaypoint31<TDerived, TOrthogonal>>(postWaypointFn, this, 31);
  configurePostEvWaypoint<EvWaypoint32<TDerived, TOrthogonal>>(postWaypointFn, this, 32);
  configurePostEvWaypoint<EvWaypoint33<TDerived, TOrthogonal>>(postWaypointFn, this, 33);
  configurePostEvWaypoint<EvWaypoint34<TDerived, TOrthogonal>>(postWaypointFn, this, 34);
  configurePostEvWaypoint<EvWaypoint35<TDerived, TOrthogonal>>(postWaypointFn, this, 35);
  configurePostEvWaypoint<EvWaypoint36<TDerived, TOrthogonal>>(postWaypointFn, this, 36);
  configurePostEvWaypoint<EvWaypoint37<TDerived, TOrthogonal>>(postWaypointFn, this, 37);
  configurePostEvWaypoint<EvWaypoint38<TDerived, TOrthogonal>>(postWaypointFn, this, 38);
  configurePostEvWaypoint<EvWaypoint39<TDerived, TOrthogonal>>(postWaypointFn, this, 39);
  configurePostEvWaypoint<EvWaypoint40<TDerived, TOrthogonal>>(postWaypointFn, this, 40);
  configurePostEvWaypoint<EvWaypoint41<TDerived, TOrthogonal>>(postWaypointFn, this, 41);
  configurePostEvWaypoint<EvWaypoint42<TDerived, TOrthogonal>>(postWaypointFn, this, 42);
  configurePostEvWaypoint<EvWaypoint43<TDerived, TOrthogonal>>(postWaypointFn, this, 43);
  configurePostEvWaypoint<EvWaypoint44<TDerived, TOrthogonal>>(postWaypointFn, this, 44);
  configurePostEvWaypoint<EvWaypoint45<TDerived, TOrthogonal>>(postWaypointFn, this, 45);
  configurePostEvWaypoint<EvWaypoint46<TDerived, TOrthogonal>>(postWaypointFn, this, 46);
  configurePostEvWaypoint<EvWaypoint47<TDerived, TOrthogonal>>(postWaypointFn, this, 47);
  configurePostEvWaypoint<EvWaypoint48<TDerived, TOrthogonal>>(postWaypointFn, this, 48);
  configurePostEvWaypoint<EvWaypoint49<TDerived, TOrthogonal>>(postWaypointFn, this, 49);
  configurePostEvWaypoint<EvWaypoint50<TDerived, TOrthogonal>>(postWaypointFn, this, 50);
  configurePostEvWaypoint<EvWaypoint51<TDerived, TOrthogonal>>(postWaypointFn, this, 51);
  configurePostEvWaypoint<EvWaypoint52<TDerived, TOrthogonal>>(postWaypointFn, this, 52);
  configurePostEvWaypoint<EvWaypoint53<TDerived, TOrthogonal>>(postWaypointFn, this, 53);
  configurePostEvWaypoint<EvWaypoint54<TDerived, TOrthogonal>>(postWaypointFn, this, 54);
  configurePostEvWaypoint<EvWaypoint55<TDerived, TOrthogonal>>(postWaypointFn, this, 55);
  configurePostEvWaypoint<EvWaypoint56<TDerived, TOrthogonal>>(postWaypointFn, this, 56);
  configurePostEvWaypoint<EvWaypoint57<TDerived, TOrthogonal>>(postWaypointFn, this, 57);
  configurePostEvWaypoint<EvWaypoint58<TDerived, TOrthogonal>>(postWaypointFn, this, 58);
  configurePostEvWaypoint<EvWaypoint59<TDerived, TOrthogonal>>(postWaypointFn, this, 59);
  configurePostEvWaypoint<EvWaypoint60<TDerived, TOrthogonal>>(postWaypointFn, this, 60);
  configurePostEvWaypoint<EvWaypoint61<TDerived, TOrthogonal>>(postWaypointFn, this, 61);
  configurePostEvWaypoint<EvWaypoint62<TDerived, TOrthogonal>>(postWaypointFn, this, 62);
  configurePostEvWaypoint<EvWaypoint63<TDerived, TOrthogonal>>(postWaypointFn, this, 63);
  configurePostEvWaypoint<EvWaypoint64<TDerived, TOrthogonal>>(postWaypointFn, this, 64);
  configurePostEvWaypoint<EvWaypoint65<TDerived, TOrthogonal>>(postWaypointFn, this, 65);
  configurePostEvWaypoint<EvWaypoint66<TDerived, TOrthogonal>>(postWaypointFn, this, 66);
  configurePostEvWaypoint<EvWaypoint67<TDerived, TOrthogonal>>(postWaypointFn, this, 67);
  configurePostEvWaypoint<EvWaypoint68<TDerived, TOrthogonal>>(postWaypointFn, this, 68);
  configurePostEvWaypoint<EvWaypoint69<TDerived, TOrthogonal>>(postWaypointFn, this, 69);
  configurePostEvWaypoint<EvWaypoint70<TDerived, TOrthogonal>>(postWaypointFn, this, 70);
  configurePostEvWaypoint<EvWaypoint71<TDerived, TOrthogonal>>(postWaypointFn, this, 71);
  configurePostEvWaypoint<EvWaypoint72<TDerived, TOrthogonal>>(postWaypointFn, this, 72);
  configurePostEvWaypoint<EvWaypoint73<TDerived, TOrthogonal>>(postWaypointFn, this, 73);
  configurePostEvWaypoint<EvWaypoint74<TDerived, TOrthogonal>>(postWaypointFn, this, 74);
  configurePostEvWaypoint<EvWaypoint75<TDerived, TOrthogonal>>(postWaypointFn, this, 75);
  configurePostEvWaypoint<EvWaypoint76<TDerived, TOrthogonal>>(postWaypointFn, this, 76);
  configurePostEvWaypoint<EvWaypoint77<TDerived, TOrthogonal>>(postWaypointFn, this, 77);
  configurePostEvWaypoint<EvWaypoint78<TDerived, TOrthogonal>>(postWaypointFn, this, 78);
  configurePostEvWaypoint<EvWaypoint79<TDerived, TOrthogonal>>(postWaypointFn, this, 79);
  configurePostEvWaypoint<EvWaypoint80<TDerived, TOrthogonal>>(postWaypointFn, this, 80);
  configurePostEvWaypoint<EvWaypoint81<TDerived, TOrthogonal>>(postWaypointFn, this, 81);
  configurePostEvWaypoint<EvWaypoint82<TDerived, TOrthogonal>>(postWaypointFn, this, 82);
  configurePostEvWaypoint<EvWaypoint83<TDerived, TOrthogonal>>(postWaypointFn, this, 83);
  configurePostEvWaypoint<EvWaypoint84<TDerived, TOrthogonal>>(postWaypointFn, this, 84);
  configurePostEvWaypoint<EvWaypoint85<TDerived, TOrthogonal>>(postWaypointFn, this, 85);
  configurePostEvWaypoint<EvWaypoint86<TDerived, TOrthogonal>>(postWaypointFn, this, 86);
  configurePostEvWaypoint<EvWaypoint87<TDerived, TOrthogonal>>(postWaypointFn, this, 87);
  configurePostEvWaypoint<EvWaypoint88<TDerived, TOrthogonal>>(postWaypointFn, this, 88);
  configurePostEvWaypoint<EvWaypoint89<TDerived, TOrthogonal>>(postWaypointFn, this, 89);
  configurePostEvWaypoint<EvWaypoint90<TDerived, TOrthogonal>>(postWaypointFn, this, 90);
  configurePostEvWaypoint<EvWaypoint91<TDerived, TOrthogonal>>(postWaypointFn, this, 91);
  configurePostEvWaypoint<EvWaypoint92<TDerived, TOrthogonal>>(postWaypointFn, this, 92);
  configurePostEvWaypoint<EvWaypoint93<TDerived, TOrthogonal>>(postWaypointFn, this, 93);
  configurePostEvWaypoint<EvWaypoint94<TDerived, TOrthogonal>>(postWaypointFn, this, 94);
  configurePostEvWaypoint<EvWaypoint95<TDerived, TOrthogonal>>(postWaypointFn, this, 95);
  configurePostEvWaypoint<EvWaypoint96<TDerived, TOrthogonal>>(postWaypointFn, this, 96);
  configurePostEvWaypoint<EvWaypoint97<TDerived, TOrthogonal>>(postWaypointFn, this, 97);
  configurePostEvWaypoint<EvWaypoint98<TDerived, TOrthogonal>>(postWaypointFn, this, 98);
  configurePostEvWaypoint<EvWaypoint99<TDerived, TOrthogonal>>(postWaypointFn, this, 99);
  configurePostEvWaypoint<EvWaypoint100<TDerived, TOrthogonal>>(postWaypointFn, this, 100);
  configurePostEvWaypoint<EvWaypoint101<TDerived, TOrthogonal>>(postWaypointFn, this, 101);
  configurePostEvWaypoint<EvWaypoint102<TDerived, TOrthogonal>>(postWaypointFn, this, 102);
  configurePostEvWaypoint<EvWaypoint103<TDerived, TOrthogonal>>(postWaypointFn, this, 103);
  configurePostEvWaypoint<EvWaypoint104<TDerived, TOrthogonal>>(postWaypointFn, this, 104);
  configurePostEvWaypoint<EvWaypoint105<TDerived, TOrthogonal>>(postWaypointFn, this, 105);
  configurePostEvWaypoint<EvWaypoint106<TDerived, TOrthogonal>>(postWaypointFn, this, 106);
  configurePostEvWaypoint<EvWaypoint107<TDerived, TOrthogonal>>(postWaypointFn, this, 107);
  configurePostEvWaypoint<EvWaypoint108<TDerived, TOrthogonal>>(postWaypointFn, this, 108);
  configurePostEvWaypoint<EvWaypoint109<TDerived, TOrthogonal>>(postWaypointFn, this, 109);
  configurePostEvWaypoint<EvWaypoint110<TDerived, TOrthogonal>>(postWaypointFn, this, 110);
  configurePostEvWaypoint<EvWaypoint111<TDerived, TOrthogonal>>(postWaypointFn, this, 111);
  configurePostEvWaypoint<EvWaypoint112<TDerived, TOrthogonal>>(postWaypointFn, this, 112);
  configurePostEvWaypoint<EvWaypoint113<TDerived, TOrthogonal>>(postWaypointFn, this, 113);
  configurePostEvWaypoint<EvWaypoint114<TDerived, TOrthogonal>>(postWaypointFn, this, 114);
  configurePostEvWaypoint<EvWaypoint115<TDerived, TOrthogonal>>(postWaypointFn, this, 115);
  configurePostEvWaypoint<EvWaypoint116<TDerived, TOrthogonal>>(postWaypointFn, this, 116);
  configurePostEvWaypoint<EvWaypoint117<TDerived, TOrthogonal>>(postWaypointFn, this, 117);
  configurePostEvWaypoint<EvWaypoint118<TDerived, TOrthogonal>>(postWaypointFn, this, 118);
  configurePostEvWaypoint<EvWaypoint119<TDerived, TOrthogonal>>(postWaypointFn, this, 119);
  configurePostEvWaypoint<EvWaypoint120<TDerived, TOrthogonal>>(postWaypointFn, this, 120);
  configurePostEvWaypoint<EvWaypoint121<TDerived, TOrthogonal>>(postWaypointFn, this, 121);
  configurePostEvWaypoint<EvWaypoint122<TDerived, TOrthogonal>>(postWaypointFn, this, 122);
  configurePostEvWaypoint<EvWaypoint123<TDerived, TOrthogonal>>(postWaypointFn, this, 123);
  configurePostEvWaypoint<EvWaypoint124<TDerived, TOrthogonal>>(postWaypointFn, this, 124);
  configurePostEvWaypoint<EvWaypoint125<TDerived, TOrthogonal>>(postWaypointFn, this, 125);
  configurePostEvWaypoint<EvWaypoint126<TDerived, TOrthogonal>>(postWaypointFn, this, 126);
  configurePostEvWaypoint<EvWaypoint127<TDerived, TOrthogonal>>(postWaypointFn, this, 127);
  configurePostEvWaypoint<EvWaypoint128<TDerived, TOrthogonal>>(postWaypointFn, this, 128);
  configurePostEvWaypoint<EvWaypoint129<TDerived, TOrthogonal>>(postWaypointFn, this, 129);
  configurePostEvWaypoint<EvWaypoint130<TDerived, TOrthogonal>>(postWaypointFn, this, 130);
  configurePostEvWaypoint<EvWaypoint131<TDerived, TOrthogonal>>(postWaypointFn, this, 131);
  configurePostEvWaypoint<EvWaypoint132<TDerived, TOrthogonal>>(postWaypointFn, this, 132);
  configurePostEvWaypoint<EvWaypoint133<TDerived, TOrthogonal>>(postWaypointFn, this, 133);
  configurePostEvWaypoint<EvWaypoint134<TDerived, TOrthogonal>>(postWaypointFn, this, 134);
  configurePostEvWaypoint<EvWaypoint135<TDerived, TOrthogonal>>(postWaypointFn, this, 135);
  configurePostEvWaypoint<EvWaypoint136<TDerived, TOrthogonal>>(postWaypointFn, this, 136);
  configurePostEvWaypoint<EvWaypoint137<TDerived, TOrthogonal>>(postWaypointFn, this, 137);
  configurePostEvWaypoint<EvWaypoint138<TDerived, TOrthogonal>>(postWaypointFn, this, 138);
  configurePostEvWaypoint<EvWaypoint139<TDerived, TOrthogonal>>(postWaypointFn, this, 139);
  configurePostEvWaypoint<EvWaypoint140<TDerived, TOrthogonal>>(postWaypointFn, this, 140);
  configurePostEvWaypoint<EvWaypoint141<TDerived, TOrthogonal>>(postWaypointFn, this, 141);
  configurePostEvWaypoint<EvWaypoint142<TDerived, TOrthogonal>>(postWaypointFn, this, 142);
  configurePostEvWaypoint<EvWaypoint143<TDerived, TOrthogonal>>(postWaypointFn, this, 143);
  configurePostEvWaypoint<EvWaypoint144<TDerived, TOrthogonal>>(postWaypointFn, this, 144);
  configurePostEvWaypoint<EvWaypoint145<TDerived, TOrthogonal>>(postWaypointFn, this, 145);
  configurePostEvWaypoint<EvWaypoint146<TDerived, TOrthogonal>>(postWaypointFn, this, 146);
  configurePostEvWaypoint<EvWaypoint147<TDerived, TOrthogonal>>(postWaypointFn, this, 147);
  configurePostEvWaypoint<EvWaypoint148<TDerived, TOrthogonal>>(postWaypointFn, this, 148);
  configurePostEvWaypoint<EvWaypoint149<TDerived, TOrthogonal>>(postWaypointFn, this, 149);
  configurePostEvWaypoint<EvWaypoint150<TDerived, TOrthogonal>>(postWaypointFn, this, 150);
  configurePostEvWaypoint<EvWaypoint151<TDerived, TOrthogonal>>(postWaypointFn, this, 151);
  configurePostEvWaypoint<EvWaypoint152<TDerived, TOrthogonal>>(postWaypointFn, this, 152);
  configurePostEvWaypoint<EvWaypoint153<TDerived, TOrthogonal>>(postWaypointFn, this, 153);
  configurePostEvWaypoint<EvWaypoint154<TDerived, TOrthogonal>>(postWaypointFn, this, 154);
  configurePostEvWaypoint<EvWaypoint155<TDerived, TOrthogonal>>(postWaypointFn, this, 155);
  configurePostEvWaypoint<EvWaypoint156<TDerived, TOrthogonal>>(postWaypointFn, this, 156);
  configurePostEvWaypoint<EvWaypoint157<TDerived, TOrthogonal>>(postWaypointFn, this, 157);
  configurePostEvWaypoint<EvWaypoint158<TDerived, TOrthogonal>>(postWaypointFn, this, 158);
  configurePostEvWaypoint<EvWaypoint159<TDerived, TOrthogonal>>(postWaypointFn, this, 159);
  configurePostEvWaypoint<EvWaypoint160<TDerived, TOrthogonal>>(postWaypointFn, this, 160);
  configurePostEvWaypoint<EvWaypoint161<TDerived, TOrthogonal>>(postWaypointFn, this, 161);
  configurePostEvWaypoint<EvWaypoint162<TDerived, TOrthogonal>>(postWaypointFn, this, 162);
  configurePostEvWaypoint<EvWaypoint163<TDerived, TOrthogonal>>(postWaypointFn, this, 163);
  configurePostEvWaypoint<EvWaypoint164<TDerived, TOrthogonal>>(postWaypointFn, this, 164);
  configurePostEvWaypoint<EvWaypoint165<TDerived, TOrthogonal>>(postWaypointFn, this, 165);
  configurePostEvWaypoint<EvWaypoint166<TDerived, TOrthogonal>>(postWaypointFn, this, 166);
  configurePostEvWaypoint<EvWaypoint167<TDerived, TOrthogonal>>(postWaypointFn, this, 167);
  configurePostEvWaypoint<EvWaypoint168<TDerived, TOrthogonal>>(postWaypointFn, this, 168);
  configurePostEvWaypoint<EvWaypoint169<TDerived, TOrthogonal>>(postWaypointFn, this, 169);
  configurePostEvWaypoint<EvWaypoint170<TDerived, TOrthogonal>>(postWaypointFn, this, 170);
  configurePostEvWaypoint<EvWaypoint171<TDerived, TOrthogonal>>(postWaypointFn, this, 171);
  configurePostEvWaypoint<EvWaypoint172<TDerived, TOrthogonal>>(postWaypointFn, this, 172);
  configurePostEvWaypoint<EvWaypoint173<TDerived, TOrthogonal>>(postWaypointFn, this, 173);
  configurePostEvWaypoint<EvWaypoint174<TDerived, TOrthogonal>>(postWaypointFn, this, 174);
  configurePostEvWaypoint<EvWaypoint175<TDerived, TOrthogonal>>(postWaypointFn, this, 175);
  configurePostEvWaypoint<EvWaypoint176<TDerived, TOrthogonal>>(postWaypointFn, this, 176);
  configurePostEvWaypoint<EvWaypoint177<TDerived, TOrthogonal>>(postWaypointFn, this, 177);
  configurePostEvWaypoint<EvWaypoint178<TDerived, TOrthogonal>>(postWaypointFn, this, 178);
  configurePostEvWaypoint<EvWaypoint179<TDerived, TOrthogonal>>(postWaypointFn, this, 179);
  configurePostEvWaypoint<EvWaypoint180<TDerived, TOrthogonal>>(postWaypointFn, this, 180);
  configurePostEvWaypoint<EvWaypoint181<TDerived, TOrthogonal>>(postWaypointFn, this, 181);
  configurePostEvWaypoint<EvWaypoint182<TDerived, TOrthogonal>>(postWaypointFn, this, 182);
  configurePostEvWaypoint<EvWaypoint183<TDerived, TOrthogonal>>(postWaypointFn, this, 183);
  configurePostEvWaypoint<EvWaypoint184<TDerived, TOrthogonal>>(postWaypointFn, this, 184);
  configurePostEvWaypoint<EvWaypoint185<TDerived, TOrthogonal>>(postWaypointFn, this, 185);
  configurePostEvWaypoint<EvWaypoint186<TDerived, TOrthogonal>>(postWaypointFn, this, 186);
  configurePostEvWaypoint<EvWaypoint187<TDerived, TOrthogonal>>(postWaypointFn, this, 187);
  configurePostEvWaypoint<EvWaypoint188<TDerived, TOrthogonal>>(postWaypointFn, this, 188);
  configurePostEvWaypoint<EvWaypoint189<TDerived, TOrthogonal>>(postWaypointFn, this, 189);
  configurePostEvWaypoint<EvWaypoint190<TDerived, TOrthogonal>>(postWaypointFn, this, 190);
  configurePostEvWaypoint<EvWaypoint191<TDerived, TOrthogonal>>(postWaypointFn, this, 191);
  configurePostEvWaypoint<EvWaypoint192<TDerived, TOrthogonal>>(postWaypointFn, this, 192);
  configurePostEvWaypoint<EvWaypoint193<TDerived, TOrthogonal>>(postWaypointFn, this, 193);
  configurePostEvWaypoint<EvWaypoint194<TDerived, TOrthogonal>>(postWaypointFn, this, 194);
  configurePostEvWaypoint<EvWaypoint195<TDerived, TOrthogonal>>(postWaypointFn, this, 195);
  configurePostEvWaypoint<EvWaypoint196<TDerived, TOrthogonal>>(postWaypointFn, this, 196);
  configurePostEvWaypoint<EvWaypoint197<TDerived, TOrthogonal>>(postWaypointFn, this, 197);
  configurePostEvWaypoint<EvWaypoint198<TDerived, TOrthogonal>>(postWaypointFn, this, 198);
  configurePostEvWaypoint<EvWaypoint199<TDerived, TOrthogonal>>(postWaypointFn, this, 199);
  configurePostEvWaypoint<EvWaypoint200<TDerived, TOrthogonal>>(postWaypointFn, this, 200);
  configurePostEvWaypoint<EvWaypoint201<TDerived, TOrthogonal>>(postWaypointFn, this, 201);
  configurePostEvWaypoint<EvWaypoint202<TDerived, TOrthogonal>>(postWaypointFn, this, 202);
  configurePostEvWaypoint<EvWaypoint203<TDerived, TOrthogonal>>(postWaypointFn, this, 203);
  configurePostEvWaypoint<EvWaypoint204<TDerived, TOrthogonal>>(postWaypointFn, this, 204);
  configurePostEvWaypoint<EvWaypoint205<TDerived, TOrthogonal>>(postWaypointFn, this, 205);
  configurePostEvWaypoint<EvWaypoint206<TDerived, TOrthogonal>>(postWaypointFn, this, 206);
  configurePostEvWaypoint<EvWaypoint207<TDerived, TOrthogonal>>(postWaypointFn, this, 207);
  configurePostEvWaypoint<EvWaypoint208<TDerived, TOrthogonal>>(postWaypointFn, this, 208);
  configurePostEvWaypoint<EvWaypoint209<TDerived, TOrthogonal>>(postWaypointFn, this, 209);
  configurePostEvWaypoint<EvWaypoint210<TDerived, TOrthogonal>>(postWaypointFn, this, 210);
  configurePostEvWaypoint<EvWaypoint211<TDerived, TOrthogonal>>(postWaypointFn, this, 211);
  configurePostEvWaypoint<EvWaypoint212<TDerived, TOrthogonal>>(postWaypointFn, this, 212);
  configurePostEvWaypoint<EvWaypoint213<TDerived, TOrthogonal>>(postWaypointFn, this, 213);
  configurePostEvWaypoint<EvWaypoint214<TDerived, TOrthogonal>>(postWaypointFn, this, 214);
  configurePostEvWaypoint<EvWaypoint215<TDerived, TOrthogonal>>(postWaypointFn, this, 215);
  configurePostEvWaypoint<EvWaypoint216<TDerived, TOrthogonal>>(postWaypointFn, this, 216);
  configurePostEvWaypoint<EvWaypoint217<TDerived, TOrthogonal>>(postWaypointFn, this, 217);
  configurePostEvWaypoint<EvWaypoint218<TDerived, TOrthogonal>>(postWaypointFn, this, 218);
  configurePostEvWaypoint<EvWaypoint219<TDerived, TOrthogonal>>(postWaypointFn, this, 219);
  configurePostEvWaypoint<EvWaypoint220<TDerived, TOrthogonal>>(postWaypointFn, this, 220);
  configurePostEvWaypoint<EvWaypoint221<TDerived, TOrthogonal>>(postWaypointFn, this, 221);
  configurePostEvWaypoint<EvWaypoint222<TDerived, TOrthogonal>>(postWaypointFn, this, 222);
  configurePostEvWaypoint<EvWaypoint223<TDerived, TOrthogonal>>(postWaypointFn, this, 223);
  configurePostEvWaypoint<EvWaypoint224<TDerived, TOrthogonal>>(postWaypointFn, this, 224);
  configurePostEvWaypoint<EvWaypoint225<TDerived, TOrthogonal>>(postWaypointFn, this, 225);
  configurePostEvWaypoint<EvWaypoint226<TDerived, TOrthogonal>>(postWaypointFn, this, 226);
  configurePostEvWaypoint<EvWaypoint227<TDerived, TOrthogonal>>(postWaypointFn, this, 227);
  configurePostEvWaypoint<EvWaypoint228<TDerived, TOrthogonal>>(postWaypointFn, this, 228);
  configurePostEvWaypoint<EvWaypoint229<TDerived, TOrthogonal>>(postWaypointFn, this, 229);
  configurePostEvWaypoint<EvWaypoint230<TDerived, TOrthogonal>>(postWaypointFn, this, 230);
  configurePostEvWaypoint<EvWaypoint231<TDerived, TOrthogonal>>(postWaypointFn, this, 231);
  configurePostEvWaypoint<EvWaypoint232<TDerived, TOrthogonal>>(postWaypointFn, this, 232);
  configurePostEvWaypoint<EvWaypoint233<TDerived, TOrthogonal>>(postWaypointFn, this, 233);
  configurePostEvWaypoint<EvWaypoint234<TDerived, TOrthogonal>>(postWaypointFn, this, 234);
  configurePostEvWaypoint<EvWaypoint235<TDerived, TOrthogonal>>(postWaypointFn, this, 235);
  configurePostEvWaypoint<EvWaypoint236<TDerived, TOrthogonal>>(postWaypointFn, this, 236);
  configurePostEvWaypoint<EvWaypoint237<TDerived, TOrthogonal>>(postWaypointFn, this, 237);
  configurePostEvWaypoint<EvWaypoint238<TDerived, TOrthogonal>>(postWaypointFn, this, 238);
  configurePostEvWaypoint<EvWaypoint239<TDerived, TOrthogonal>>(postWaypointFn, this, 239);
  configurePostEvWaypoint<EvWaypoint240<TDerived, TOrthogonal>>(postWaypointFn, this, 240);
  configurePostEvWaypoint<EvWaypoint241<TDerived, TOrthogonal>>(postWaypointFn, this, 241);
  configurePostEvWaypoint<EvWaypoint242<TDerived, TOrthogonal>>(postWaypointFn, this, 242);
  configurePostEvWaypoint<EvWaypoint243<TDerived, TOrthogonal>>(postWaypointFn, this, 243);
  configurePostEvWaypoint<EvWaypoint244<TDerived, TOrthogonal>>(postWaypointFn, this, 244);
  configurePostEvWaypoint<EvWaypoint245<TDerived, TOrthogonal>>(postWaypointFn, this, 245);
  configurePostEvWaypoint<EvWaypoint246<TDerived, TOrthogonal>>(postWaypointFn, this, 246);
  configurePostEvWaypoint<EvWaypoint247<TDerived, TOrthogonal>>(postWaypointFn, this, 247);
  configurePostEvWaypoint<EvWaypoint248<TDerived, TOrthogonal>>(postWaypointFn, this, 248);
  configurePostEvWaypoint<EvWaypoint249<TDerived, TOrthogonal>>(postWaypointFn, this, 249);
  configurePostEvWaypoint<EvWaypoint250<TDerived, TOrthogonal>>(postWaypointFn, this, 250);
  configurePostEvWaypoint<EvWaypoint251<TDerived, TOrthogonal>>(postWaypointFn, this, 251);
  configurePostEvWaypoint<EvWaypoint252<TDerived, TOrthogonal>>(postWaypointFn, this, 252);
  configurePostEvWaypoint<EvWaypoint253<TDerived, TOrthogonal>>(postWaypointFn, this, 253);
  configurePostEvWaypoint<EvWaypoint254<TDerived, TOrthogonal>>(postWaypointFn, this, 254);
  configurePostEvWaypoint<EvWaypoint255<TDerived, TOrthogonal>>(postWaypointFn, this, 255);
  configurePostEvWaypoint<EvWaypoint256<TDerived, TOrthogonal>>(postWaypointFn, this, 256);
  // Doxygen endcond tag
  /// @endcond
}

}  // namespace cl_nav2z
