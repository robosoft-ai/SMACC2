#include <eg_random_generator/eg_random_generator.h>

namespace smacc
{
    namespace state_reactors
    {
        EgRandomGenerator::EgRandomGenerator(RandomGenerateReactorMode mode, double evAMin, double evAMax, double evBMin, double evBMax, double evCMin, double evCMax)
            : 
              mode_(mode),
              evAMin_(evAMin),
              evAMax_(evAMax),
              evBMin_(evBMin),
              evBMax_(evBMax),
              evCMin_(evCMin),
              evCMax_(evCMax)
        {

            auto values = {evAMin, evAMax, evBMin, evBMax, evCMin, evCMax};

            this->minValue = std::numeric_limits<double>::max();
            this->maxValue = std::numeric_limits<double>::min();
            for (auto &v : values)
            {
                if (v < minValue)
                {
                    minValue = v;
                }

                if (v > maxValue)
                {
                    maxValue = v;
                }
            }
        }

        void EgRandomGenerator::postRandomEvents()
        {
            int range = this->maxValue - this->minValue;
            int result = (rand() % range) + minValue;

            if (result >= evAMin_ && result <= evAMax_)
            {
                this->postEventA();
            }

            if (result >= evBMin_ && result <= evBMax_)
            {
                this->postEventB();
            }

            if (result >= evCMin_ && result <= evCMax_)
            {
                this->postEventC();
            }
        }

        void EgRandomGenerator::onEntry()
        {
            if (mode_ == RandomGenerateReactorMode::ONE_SHOT)
            {
                this->postRandomEvents();
            }
        }

        void EgRandomGenerator::update()
        {
            if (mode_ == RandomGenerateReactorMode::ON_UPDATE)
            {
                this->postRandomEvents();
            }
        }
    } // namespace state_reactors
} // namespace smacc