#ifndef MOTORCALLBACK_H
#define MOTORCALLBACK_H

#include <ubiquity_motor/MotorCommand.h>

class MotorCallbackInterface
{
public:
    // The prefix "mcbi" is to prevent naming clashes.
    virtual void mcbiCallbackFunction(MotorCommand) = 0;
};

#endif