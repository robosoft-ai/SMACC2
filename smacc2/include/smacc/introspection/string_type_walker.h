/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <string>
#include <memory>
#include <vector>
#include <typeinfo>
#include <map>

namespace smacc
{
namespace introspection
{
class TypeInfo
{
public:
    typedef std::shared_ptr<TypeInfo> Ptr;

    //---- FACTORY STATIC METHODS -----------
    static TypeInfo::Ptr getTypeInfoFromString(std::string inputtext);
    static TypeInfo::Ptr getFromStdTypeInfo(const std::type_info &tid);

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

    std::string toString()
    {
        return this->tkey + ":" + this->finaltype;
    }

    std::string getNonTemplatedTypeName()
    {
        auto index = this->finaltype.find("<");
        return this->finaltype.substr(0, index);
    }

    
    const std::string &getFullName()
    {
        return this->finaltype;
    }

    static std::map<std::string, Ptr> typeInfoDatabase;

private:
    std::string tkey;
    std::string codedtype;
    std::string finaltype;
};
} // namespace introspection
} // namespace smacc