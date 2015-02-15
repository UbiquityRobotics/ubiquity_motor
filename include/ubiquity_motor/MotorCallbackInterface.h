#include <ubiquity_motor/UbqiuityMotorCommand.h>

class MotorCallbackInterface
{
public:
    // The prefix "mcbi" is to prevent naming clashes.
    virtual void mcbiCallbackFunction(UbqiuityMotorCommand) = 0;
};