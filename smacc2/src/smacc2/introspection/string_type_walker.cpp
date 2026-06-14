// Copyright 2025 Robosoft Inc.
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

#include <smacc2/introspection/smacc_type_info.hpp>

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <set>

#include <smacc2/common.hpp>

namespace smacc2
{
namespace introspection
{
bool replace(std::string & str, const std::string & from, const std::string & to)
{
  size_t start_pos = str.find(from);
  if (start_pos == std::string::npos) return false;
  str.replace(start_pos, from.length(), to);
  return true;
}

std::string replace_back(
  std::string roottype, std::vector<std::pair<std::string, std::string>> & orderedtypesdict)
{
  while (roottype.find("$") != std::string::npos)
  {
    for (auto & t : orderedtypesdict)
    {
      auto & tkey = t.first;
      auto & tval = t.second;

      replace(roottype, tkey, tval);
    }
  }

  return roottype;
}

std::map<std::string, TypeInfo::Ptr> TypeInfo::typeInfoDatabase;

TypeInfo::Ptr TypeInfo::getFromStdTypeInfo(const std::type_info & tid)
{
  return TypeInfo::getTypeInfoFromString(demangleSymbol(tid.name()));
}

TypeInfo::Ptr TypeInfo::getTypeInfoFromString(std::string inputtext)
{
  auto it = typeInfoDatabase.find(inputtext);

  if (it != typeInfoDatabase.end()) return typeInfoDatabase[inputtext];

  bool ok = false;
  int typecount = 0;
  std::string originalinputtext = inputtext;
  std::map<std::string, std::string> typesdict;
  std::map<std::string, std::string> typesdict_content;

  // ITERATE on the current string to replace internal template types with ids (ie $Tx) replacements
  // until plain template type

  while (!ok)
  {
    const char * simpletypeRE = "[^\\<\\>,\\s]+\\<[^\\<\\>]+\\>";

    // locate moste outer template
    std::smatch matches;
    std::regex regex(simpletypeRE);  // matches words beginning by "sub"

    std::regex_search(inputtext, matches, regex);

    // checking ending condition
    if (matches.size() == 0)
    {
      if (typesdict.size() == 0)
      {
        auto tkey = "$T" + std::to_string(typecount);
        typesdict[tkey] = inputtext;
      }
      break;
    }
    else
    {
      // find and replace all coincidences typestr to tokenid
      int i = 0;
      for (auto & m : matches)
      {
        std::string tstr = m;
        auto tkey = "$T" + std::to_string(typecount);
        replace(inputtext, tstr, tkey);

        typecount++;
        typesdict[tkey] = tstr;

        i++;
      }
    }
  }

  // ---------- SORT by token key to replace back in a single pass --------
  std::vector<std::pair<std::string, std::string>> orderedTypedict;
  for (const auto & item : typesdict)
  {
    orderedTypedict.emplace_back(item);
  }

  std::sort(
    orderedTypedict.begin(), orderedTypedict.end(),
    [](auto & a, auto & b) { return std::stoi(a.first.substr(2)) > std::stoi(b.first.substr(2)); });
  // -----------------------------------------------------------------

  std::set<std::string> allbasetypes;
  for (auto & typeentry : orderedTypedict /*typesdict*/)
  {
    auto flat = typeentry.second;

    size_t startindex = flat.find("<");
    size_t endindex = flat.find(">");
    if (startindex != std::string::npos)
    {
      flat = flat.substr(
        startindex + 1,
        endindex - startindex - 1);  // gets the list of template parameters in csv format

      typesdict_content[typeentry.first] = flat;
      std::vector<std::string> localbasetypes;

      std::istringstream iss(flat);
      std::string token;
      while (std::getline(iss, token, ','))
      {
        boost::trim(token);
        localbasetypes.push_back(token);
      }

      for (auto & b : localbasetypes)
      {
        size_t found = b.find("$");

        if (found == std::string::npos)
        {
          allbasetypes.insert(b);
        }
      }
    }

    //refresh
    for (auto & b : allbasetypes)
    {
      typesdict[b] = b;
    }
  }
  // append leaf types
  for (auto & b : allbasetypes)
  {
    orderedTypedict.push_back({b, b});
  }

  std::vector<TypeInfo::Ptr> types;
  std::vector<std::string> tokens;

  for (auto t : orderedTypedict)  // must be ordered. to avoid issue, ie: $11 to be replaced in $T14
  {
    auto & tkey = t.first;
    auto & tval = t.second;
    auto finaltype = replace_back(tval, orderedTypedict);
    auto tinfo = std::make_shared<TypeInfo>(tkey, tval, finaltype);
    types.push_back(tinfo);
    tokens.push_back(tkey);
  }

  // --------- FINDING THE CURRENT TYPE -------------
  TypeInfo::Ptr roottype = nullptr;
  for (auto & t : types)
  {
    if (t->finaltype == originalinputtext)
    {
      roottype = t;
      break;
    }
  }

  if (roottype == nullptr && globalNh_ != nullptr)
  {
    std::stringstream ss;
    ss << "---------------" << std::endl;
    ss << "TYPE STRING WALKER, type was not properly interpreted: " << std::endl
       << " - " << inputtext.c_str() << std::endl
       << " - " << originalinputtext << std::endl;

    auto strmsg = ss.str();
    RCLCPP_WARN_STREAM(globalNh_->get_logger(), strmsg);
  }

  // Replacing back tokens in strings to get the whole final name
  for (size_t i = 0; i < types.size(); i++)
  {
    auto t = types[i];
    auto ttoken = tokens[i];

    std::vector<std::pair<int, TypeInfo::Ptr>> unorderedTemplateParameters;

    auto codedtypecopy = typesdict_content[ttoken];

    for (auto & t2 : types)
    {
      if (t == t2) continue;

      auto index = codedtypecopy.find(t2->tkey);
      if (index != std::string::npos)
      {
        auto pair = std::make_pair(index, t2);  // this line is important for the order of templates
        unorderedTemplateParameters.push_back(pair);
        replace(codedtypecopy, t2->tkey, "");  // consume token
        //std::cout << "codedtypecopy: " << codedtypecopy << std::endl;
      }
    }

    std::sort(
      unorderedTemplateParameters.begin(), unorderedTemplateParameters.end(),
      [](auto & a, auto & b) -> bool { return a.first <= b.first; });

    //RCLCPP_DEBUG_STREAM(nh_->get_logger(),"------------------");
    //RCLCPP_DEBUG_STREAM(nh_->get_logger(),"CREATING TYPE:" << t->getFullName());

    for (auto & item : unorderedTemplateParameters)
    {
      //RCLCPP_DEBUG_STREAM(nh_->get_logger()," - template parameter: " << item.second->getFullName());
      t->templateParameters.push_back(item.second);
    }
    //RCLCPP_DEBUG_STREAM(nh_->get_logger(),"------------------");
  }

  //RCLCPP_DEBUG_STREAM(nh_->get_logger(),"ADDING TYPE TO DATABASE: " << inputtext);
  //RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Current Database");
  for (auto & en : typeInfoDatabase)
  {
    if (globalNh_ != nullptr) RCLCPP_DEBUG_STREAM(globalNh_->get_logger(), "- " << en.first);
  }

  typeInfoDatabase[originalinputtext] = roottype;

  if (roottype == nullptr && globalNh_ != nullptr)
  {
    std::stringstream ss;
    ss << "---------------" << std::endl;
    ss << "TYPE STRING WALKER, type was not properly interpreted: " << std::endl
       << " - " << inputtext.c_str() << std::endl
       << " - " << originalinputtext << std::endl;

    ss << "---- current type (ordered types)" << std::endl;

    for (auto & en : orderedTypedict)
    {
      ss << "- " << en.first << ": " << en.second << std::endl;
    }
    ss << "---------------" << std::endl;

    ss << "------current type types --------" << std::endl;

    for (auto & t : types)
    {
      ss << t->finaltype << std::endl;
    }

    ss << "----total typeinfo database" << std::endl;

    for (auto & en : typeInfoDatabase)
    {
      ss << "- " << en.first << ": " << en.second << std::endl;
    }

    auto strmsg = ss.str();
    RCLCPP_WARN_STREAM(globalNh_->get_logger(), strmsg);
  }

  return roottype;
}

}  // namespace introspection
}  // namespace smacc2
