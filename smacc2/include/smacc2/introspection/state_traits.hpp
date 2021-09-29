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

#pragma once

namespace smacc2
{
template <typename T, typename TransitionTagName>
class HasSpecificNamedOnExit
{
  template <typename U, void (U::*)(TransitionTagName)>
  struct Check;
  template <typename U>
  static char func(Check<U, &U::onExit> *);
  template <typename U>
  static int func(...);

public:
  typedef HasSpecificNamedOnExit type;
  enum
  {
    value = sizeof(func<T>(0)) == sizeof(char)
  };
};

template <typename TState, typename TTransitionTagName>
void specificNamedOnExit(TState & st, TTransitionTagName tn, std::true_type)
{
  st.onExit(tn);
}

template <typename TState, typename TTransitionTagName>
void specificNamedOnExit(TState &, TTransitionTagName, std::false_type)
{
}

template <typename TState, typename TTransitionTagName>
void specificNamedOnExit(TState & m, TTransitionTagName tn)
{
  specificNamedOnExit(
    m, tn,
    std::integral_constant<bool, HasSpecificNamedOnExit<TState, TTransitionTagName>::value>());
}

//-------------------------------------------------

template <typename T>
class HasStandardOnExit
{
  template <typename U, void (U::*)()>
  struct Check;
  template <typename U>
  static char func(Check<U, &U::onExit> *);
  template <typename U>
  static int func(...);

public:
  typedef HasStandardOnExit type;
  enum
  {
    value = sizeof(func<T>(0)) == sizeof(char)
  };
};

template <typename TState>
void standardOnExit(TState & st, std::true_type)
{
  st.onExit();
}

template <typename TState>
void standardOnExit(TState &, std::false_type)
{
}

template <typename TState>
void standardOnExit(TState & m)
{
  standardOnExit(m, std::integral_constant<bool, HasStandardOnExit<TState>::value>());
}

}  // namespace smacc2
