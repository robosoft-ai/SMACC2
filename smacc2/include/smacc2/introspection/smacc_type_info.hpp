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
#include <map>
#include <memory>
#include <string>
#include <typeinfo>
#include <vector>

namespace smacc2
{
namespace introspection
{
class TypeInfo
{
public:
  typedef std::shared_ptr<TypeInfo> Ptr;

  //---- FACTORY STATIC METHODS -----------
  static TypeInfo::Ptr getTypeInfoFromString(std::string inputtext);
  static TypeInfo::Ptr getFromStdTypeInfo(const std::type_info & tid);

  template <typename T>
  static TypeInfo::Ptr getTypeInfoFromType()
  {
    return TypeInfo::getFromStdTypeInfo(typeid(T));
  }
  //---------------------------------------

  std::vector<Ptr> templateParameters;

  TypeInfo(std::string tkey, std::string codedtype, std::string finaltype)
  {
    this->tkey = tkey;
    this->codedtype = codedtype;
    this->finaltype = finaltype;
  }

  std::string toString() { return this->tkey + ":" + this->finaltype; }

  std::string getNonTemplatedTypeName()
  {
    auto index = this->finaltype.find("<");
    return this->finaltype.substr(0, index);
  }

  const std::string & getFullName() { return this->finaltype; }

  static std::map<std::string, Ptr> typeInfoDatabase;

private:
  std::string tkey;
  std::string codedtype;
  std::string finaltype;
};
}  // namespace introspection
}  // namespace smacc2
