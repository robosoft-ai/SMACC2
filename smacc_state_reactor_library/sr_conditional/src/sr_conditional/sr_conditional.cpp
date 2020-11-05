#include <sr_conditional/sr_conditional.h>

namespace smacc
{
namespace state_reactors
{
Srsr_conditional::~Srsr_conditional()
{
}

bool Srsr_conditional::triggers()
{
    return this->conditionFlag;
}
} // namespace state_reactors
} // namespace smacc