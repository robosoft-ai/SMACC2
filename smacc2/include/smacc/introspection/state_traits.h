#pragma once

namespace smacc
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
    void specificNamedOnExit(TState &st, TTransitionTagName tn, std::true_type)
    {
        st.onExit(tn);
    }

    template <typename TState, typename TTransitionTagName>
    void specificNamedOnExit(TState &, TTransitionTagName tn, std::false_type)
    {
    }

    template <typename TState, typename TTransitionTagName>
    void specificNamedOnExit(TState &m, TTransitionTagName tn)
    {
        specificNamedOnExit(m, tn,
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
    void standardOnExit(TState &st, std::true_type)
    {
        st.onExit();
    }

    template <typename TState>
    void standardOnExit(TState &, std::false_type)
    {
    }

    template <typename TState>
    void standardOnExit(TState &m)
    {
        standardOnExit(m,
                       std::integral_constant<bool, HasStandardOnExit<TState>::value>());
    }

} // namespace smacc