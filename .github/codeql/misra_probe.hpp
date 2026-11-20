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

// Self-test for the CodeQL workflow, header part: the shapes of misra_probe.cpp inside a header.
#ifndef GITHUB__CODEQL__MISRA_PROBE_HPP_
#define GITHUB__CODEQL__MISRA_PROBE_HPP_

#include <cstddef>
#include <memory>

inline int * headerPlain()
{
  return new int(1);  // EXPECT plain new in an inline function of a header
}

template <typename T>
int * headerNonDependent()
{
  return new int(2);  // EXPECT new of a fixed type in a template of a header
}

template <typename T>
struct HeaderHolder
{
  std::shared_ptr<T> shared;
  T * fill()
  {
    T * raw = new T();  // EXPECT new in a member function of a class template of a header
    shared = std::shared_ptr<T>(new T());  // EXPECT new handed to a shared_ptr in a class template
    return raw;
  }
};

// The shape of a Boost.Statechart event: the class brings its own operator new and operator delete.
struct PooledEvent
{
  static void * operator new(std::size_t size)
  {
    return ::operator new(size);  // EXPECT direct call of the global operator new
  }
  static void operator delete(void * p)
  {
    ::operator delete(p);  // EXPECT direct call of the global operator delete
  }
  int value = 0;
};

template <typename TEvent>
int headerPost()
{
  auto * ev = new TEvent();  // EXPECT new of a class with its own operator new, in a template
  int v = ev->value;
  delete ev;  // EXPECT delete of a class with its own operator delete, in a template
  return v;
}

#endif  // GITHUB__CODEQL__MISRA_PROBE_HPP_
