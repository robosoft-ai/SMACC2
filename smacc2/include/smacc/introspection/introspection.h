/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <boost/statechart/state.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/event.hpp>

#include <rclcpp/rclcpp.hpp>
#include <typeinfo>
#include <boost/mpl/list.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/mpl/transform.hpp>

#include <smacc/smacc_types.h>
#include <smacc/introspection/string_type_walker.h>
#include <smacc/introspection/smacc_state_info.h>

#include "smacc_msgs/msg/smacc_transition.hpp"
#include <cxxabi.h>

namespace sc = boost::statechart;

namespace smacc
{
namespace introspection
{
using namespace boost;
using namespace smacc::default_transition_tags;

void transitionInfoToMsg(const SmaccTransitionInfo &transition, smacc_msgs::msg::SmaccTransition &transitionMsg);

typedef std::allocator<boost::statechart::none> SmaccAllocator;

// template <class T>
// auto optionalNodeHandle(boost::intrusive_ptr<T> &obj)
//     -> rclcpp::Node::SharedPtr
// {
//     return obj->getROSNode();
// }

// template <class T>
// auto optionalNodeHandle(T *) -> rclcpp::Node::SharedPtr
// {
//     return rclcpp::Node::make_shared("");
// }

inline std::string demangleSymbol(const std::string &name)
{
    return demangleSymbol(name.c_str());
}

inline std::string demangleSymbol(const char *name)
{
#if (__GNUC__ && __cplusplus && __GNUC__ >= 3)
    int status;
    char *res = abi::__cxa_demangle(name, 0, 0, &status);
    if (res)
    {
        const std::string demangled_name(res);
        std::free(res);
        return demangled_name;
    }
    // Demangling failed, fallback to mangled name
    return std::string(name);
#else
    return std::string(name);
#endif
}

template <typename T>
inline std::string demangleSymbol()
{
    return demangleSymbol(typeid(T).name());
}

template <class T>
inline std::string demangledTypeName()
{
    return demangleSymbol(typeid(T).name());
}

inline std::string demangleType(const std::type_info* tinfo)
{
    return demangleSymbol(tinfo->name());
}

inline std::string demangleType(const std::type_info &tinfo)
{
    return demangleSymbol(tinfo.name());
}

template <typename...>
struct typelist
{
};

//-------------------------------------------------------------------------
template <typename T>
class HasEventLabel
{
private:
    typedef char YesType[1];
    typedef char NoType[2];

    template <typename C>
    static YesType &test(decltype(&C::getEventLabel));
    template <typename C>
    static NoType &test(...);

public:
    enum
    {
        value = sizeof(test<T>(0)) == sizeof(YesType)
    };
};

template <typename T>
typename std::enable_if<HasEventLabel<T>::value, void>::type
EventLabel(std::string &label)
{
    label = T::getEventLabel();
}

template <typename T>
typename std::enable_if<!HasEventLabel<T>::value, void>::type
EventLabel(std::string &label)
{
    label = "";
}
//-----------------------------------------------------------------------

template <typename T>
class HasAutomaticTransitionTag
{
private:
    typedef char YesType[1];
    typedef char NoType[2];

    template <typename C>
    static YesType &test(decltype(&C::getDefaultTransitionTag));
    template <typename C>
    static NoType &test(...);

public:
    enum
    {
        value = sizeof(test<T>(0)) == sizeof(YesType)
    };
};

template <typename T>
typename std::enable_if<HasAutomaticTransitionTag<T>::value, void>::type
automaticTransitionTag(std::string &transition_name)
{
    transition_name = T::getDefaultTransitionTag();
}

template <typename T>
typename std::enable_if<!HasAutomaticTransitionTag<T>::value, void>::type
automaticTransitionTag(std::string &transition_name)
{
    transition_name = "";
}

//-------------------------------------------------
template <typename T>
class HasAutomaticTransitionType
{
private:
    typedef char YesType[1];
    typedef char NoType[2];

    template <typename C>
    static YesType &test(decltype(&C::getDefaultTransitionType));
    template <typename C>
    static NoType &test(...);

public:
    enum
    {
        value = sizeof(test<T>(0)) == sizeof(YesType)
    };
};

template <typename T>
typename std::enable_if<HasAutomaticTransitionType<T>::value, void>::type
automaticTransitionType(std::string &transition_type)
{
    transition_type = T::getDefaultTransitionType();
}

template <typename T>
typename std::enable_if<!HasAutomaticTransitionType<T>::value, void>::type
automaticTransitionType(std::string &transition_type)
{
    transition_type = demangledTypeName<DEFAULT>();
}

// there are many ways to implement this, for instance adding static methods to the types
typedef boost::mpl::list<SUCCESS, ABORT, CANCEL, /*PREEMPT,*/ CONTINUELOOP, ENDLOOP> DEFAULT_TRANSITION_TYPES;

//--------------------------------

template <typename T>
struct type_
{
    using type = T;
};

//---------------------------------------------
template <typename T>
struct add_type_wrapper
{
    using type = type_<T>;
};

template <typename TTransition>
struct CheckType
{
    CheckType(std::string *transitionTypeName)
    {
        this->transitionTypeName = transitionTypeName;
    }

    std::string *transitionTypeName;
    template <typename T>
    void operator()(T)
    {
        //RCLCPP_INFO_STREAM(nh_->get_logger(),"comparing.."<< demangleSymbol<T>() <<" vs " << demangleSymbol<TTransition>() );
        if (std::is_base_of<T, TTransition>::value || std::is_same<T, TTransition>::value)
        {
            *(this->transitionTypeName) = demangledTypeName<T>();
            //RCLCPP_INFO(nh_->get_logger(),"YESS!");
        }
    }
};

template <typename TTransition>
static std::string getTransitionType()
{
    std::string output;
    CheckType<TTransition> op(&output);
    using boost::mpl::_1;
    using wrappedList = typename boost::mpl::transform<DEFAULT_TRANSITION_TYPES, _1>::type;

    boost::mpl::for_each<wrappedList>(op);
    return output;
}

// // BASE CASE
// template <typename T>
// static void walkStateReactorsSources(SmaccStateReactorInfo &sbinfo, typelist<T>)
// {
//     auto sourceType = TypeInfo::getFromStdTypeInfo(typeid(T));
//     auto evinfo = std::make_shared<SmaccEventInfo>(sourceType);
//     EventLabel<T>(evinfo->label);
//     sbinfo.sourceEventTypes.push_back(evinfo);
//     RCLCPP_INFO_STREAM(nh_->get_logger(),"event: " << sourceType->getFullName());
//     RCLCPP_INFO_STREAM(nh_->get_logger(),"event parameters: " << sourceType->templateParameters.size());
// }

// // RECURSIVE CASE
// template <typename TEvHead, typename... TEvArgs>
// static void walkStateReactorsSources(SmaccStateReactorInfo &sbinfo, typelist<TEvHead, TEvArgs...>)
// {
//     auto sourceType = TypeInfo::getFromStdTypeInfo(typeid(TEvHead));
//     auto evinfo = std::make_shared<SmaccEventInfo>(sourceType);
//     EventLabel<TEvHead>(evinfo->label);
//     sbinfo.sourceEventTypes.push_back(evinfo);
//     RCLCPP_INFO_STREAM(nh_->get_logger(),"event: " << sourceType->getFullName());
//     RCLCPP_INFO_STREAM(nh_->get_logger(),"event parameters: " << sourceType->templateParameters.size());
//     walkStateReactorsSources(sbinfo, typelist<TEvArgs...>());
// }

} // namespace introspection
} // namespace smacc
#include <smacc/introspection/smacc_state_machine_info.h>
