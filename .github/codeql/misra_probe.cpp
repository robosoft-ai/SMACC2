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

// Self-test for the CodeQL workflow: each marked line breaks MISRA C++:2023 rule 21.6.2 or 21.6.3 on purpose.
// The workflow compiles this file and codeql_misra_summary.py reports which marked lines the queries found.
// It is part of no package and is never linked or run.
#include <cstdlib>
#include <functional>

#include "misra_probe.hpp"

struct Probe
{
  int value = 0;
};

int plainNew()
{
  int * p = new int(3);  // EXPECT plain new of a built-in type
  int v = *p;
  delete p;  // EXPECT plain delete
  return v;
}

int classNew()
{
  Probe * q = new Probe;  // EXPECT plain new of a class type
  int v = q->value;
  delete q;  // EXPECT delete of a class type
  return v;
}

int cAllocation()
{
  int * p = static_cast<int *>(std::malloc(sizeof(int)));  // EXPECT malloc
  std::free(p);                                            // EXPECT free
  return 0;
}

template <typename T>
T * templateNew()
{
  return new T();  // EXPECT new of a dependent type in an instantiated template
}

template <typename T>
T * templateNeverUsed()
{
  return new T();  // EXPECT new in a template that is never instantiated
}

struct Poster
{
  std::function<int()> post;
  // The shape SMACC2 uses: a lambda inside a member template creates an event with new.
  template <typename TEvent>
  void configure()
  {
    post = []()
    {
      auto * ev = new TEvent();  // EXPECT new in a lambda inside an instantiated member template
      int v = ev->value;
      delete ev;  // EXPECT delete in a lambda inside an instantiated member template
      return v;
    };
  }
};

int useAll()
{
  Probe * p = templateNew<Probe>();
  int v = p->value;
  delete p;  // EXPECT delete of a pointer a template returned
  int * a = headerPlain();
  int * b = headerNonDependent<Probe>();
  v += *a + *b;
  delete a;  // EXPECT delete of a pointer a header function returned
  delete b;  // EXPECT delete of a pointer a header template returned
  HeaderHolder<Probe> holder;
  Probe * held = holder.fill();
  v += held->value + holder.shared->value + headerPost<PooledEvent>();
  delete held;  // EXPECT delete of a pointer a class template returned
  Poster poster;
  poster.configure<Probe>();
  return v + poster.post() + plainNew() + classNew() + cAllocation();
}
