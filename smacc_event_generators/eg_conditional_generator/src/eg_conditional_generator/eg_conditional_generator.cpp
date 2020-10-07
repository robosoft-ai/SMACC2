#include <eg_conditional_generator/eg_conditional_generator.h>

namespace smacc
{
    namespace event_generators
    {
        EgConditionalGenerator::EgConditionalGenerator(ConditionalGeneratorMode mode, std::function <bool()> updatePredicate)
            :mode_(mode),
            updatePredicate_(updatePredicate)
        {

        }

        void EgConditionalGenerator::checkPredicateAndPost()
        {
            if(this->updatePredicate_())
            {
                this->postEventTrue();
            }
            else
            {
                this->postEventFalse();
            }
        }

        void EgConditionalGenerator::onEntry()
        {
            if (mode_ == ConditionalGeneratorMode::ONE_SHOT)
            {
                this->checkPredicateAndPost();
            }
        }

        void EgConditionalGenerator::update()
        {
            if (mode_ == ConditionalGeneratorMode::ON_UPDATE)
            {
                this->checkPredicateAndPost();
            }
        }

        void EgConditionalGenerator::setPredicateFunction(std::function <bool()> updatePredicate)
        {
            updatePredicate_ = updatePredicate;
        }
        
    } // namespace state_reactors
} // namespace smacc