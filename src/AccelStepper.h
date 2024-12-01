// SRStepper.h

// AccelStepper code - Copyright (C) 2009-2020 Mike McCauley

#ifndef AccelStepper_h
#define AccelStepper_h

#include <stdlib.h>
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif

// These defs cause trouble on some versions of Arduino
#undef round

// Use the system yield() whenever possoible, since some platforms require it for housekeeping, especially
// ESP8266
#if (defined(ARDUINO) && ARDUINO >= 155) || defined(ESP8266)
#define YIELD yield();
#else
#define YIELD
#endif

/////////////////////////////////////////////////////////////////////
/// \class AccelStepper AccelStepper.h <AccelStepper.h>
/// \brief Support for stepper motors with acceleration etc.
///
/// This defines a single 2 or 4 pin stepper motor, or stepper moter with fdriver chip, with optional
/// acceleration, deceleration, absolute positioning commands etc. Multiple
/// simultaneous steppers are supported, all moving
/// at different speeds and accelerations.
///
/// \par Operation
/// This module operates by computing a step time in microseconds. The step
/// time is recomputed after each step and after speed and acceleration
/// parameters are changed by the caller. The time of each step is recorded in
/// microseconds. The run() function steps the motor once if a new step is due.
/// The run() function must be called frequently until the motor is in the
/// desired position, after which time run() will do nothing.
///
/// \par Positioning
/// Positions are specified by a signed long integer. At
/// construction time, the current position of the motor is consider to be 0. Positive
/// positions are clockwise from the initial position; negative positions are
/// anticlockwise. The current position can be altered for instance after
/// initialization positioning.
///
/// \par Caveats
/// This is an open loop controller: If the motor stalls or is oversped,
/// AccelStepper will not have a correct
/// idea of where the motor really is (since there is no feedback of the motor's
/// real position. We only know where we _think_ it is, relative to the
/// initial starting point).
///
/// \par Performance
/// The fastest motor speed that can be reliably supported is about 4000 steps per
/// second at a clock frequency of 16 MHz on Arduino such as Uno etc.
/// Faster processors can support faster stepping speeds.
/// However, any speed less than that
/// down to very slow speeds (much less than one per second) are also supported,
/// provided the run() function is called frequently enough to step the motor
/// whenever required for the speed set.
/// Calling setAcceleration() is expensive,
/// since it requires a square root to be calculated.
///
/// Gregor Christandl reports that with an Arduino Due and a simple test program,
/// he measured 43163 steps per second using runSpeed(),
/// and 16214 steps per second using run();
class AccelStepper
{
public:
    AccelStepper(uint8_t count, uint8_t data, uint8_t clk, uint8_t latch, uint8_t oe = 0, uint8_t rst = 5);

    /// Set the target position. The run() function will try to move the motor (at most one step per call)
    /// from the current position to the target position set by the most
    /// recent call to this function. Caution: moveTo() also recalculates the speed for the next step.
    /// If you are trying to use constant speed movements, you should call setSpeed() after calling moveTo().
    /// \param[in] absolute The desired absolute position. Negative is
    /// anticlockwise from the 0 position.
    void moveTo(long absolute);

    /// Set the target position relative to the current position.
    /// \param[in] relative The desired position relative to the current position. Negative is
    /// anticlockwise from the current position.
    void move(long relative);

    /// Poll a single motor, and step it if a step is due, implementing
    /// accelerations and decelerations to achieve the target position. You must call this as
    /// frequently as possible, but at least once per minimum step time interval,
    /// preferably in your main loop. Note that each call to run() will make at most one step, and then only when a step is due,
    /// based on the current speed and the time since the last step.
    /// \param[in] motorId The motor to poll.
    /// \return true if the motor is still running to the target position.
    bool run(uint8_t motorId);

    /// Poll a single motor and step it if a step is due, implementing a constant
    /// speed as set by the most recent call to setSpeed(). You must call this as
    /// frequently as possible, but at least once per step interval,
    /// \param[in] motorId The motor to poll.
    /// \return true if the motor was stepped.
    bool runSpeed(uint8_t motorId);

    /// Poll all motors, and step them if a step is due, implementing
    /// accelerations and decelerations to achieve the target position. You must call this as
    /// frequently as possible, but at least once per minimum step time interval,
    /// preferably in your main loop. Note that each call to runAll() will make at most one step per motor,
    /// and then only when a step is due, based on the current speed and the time since the last step.
    /// \return true if any motor is still running to the target position.
    bool runAll();

    /// Poll all motors and step them if a step is due, implementing a constant
    /// speed as set by the most recent call to setSpeed(). You must call this as
    /// frequently as possible, but at least once per step interval,
    /// \return true if any motor was stepped.
    bool runSpeedAll();

    /// Sets the maximum permitted speed for the selected motor; or all of them if motorId is negative.
    /// The run() function will accelerate up to the speed set by this function.
    /// Caution: the maximum speed achievable depends on your processor and clock speed.
    /// The default maxSpeed is 1.0 steps per second.
    /// \param[in] motorId The motor to set the maximum speed for. If negative, sets the maximum speed for all motors.
    /// \param[in] speed The desired maximum speed in steps per second. Must
    /// be > 0. Caution: Speeds that exceed the maximum speed supported by the processor may
    /// Result in non-linear accelerations and decelerations.
    void setMaxSpeed(int8_t motorId, float speed);

    /// Returns the maximum speed configured for this stepper
    /// that was previously set by setMaxSpeed();
    /// \param[in] motorId The motor to get the maximum speed for.
    /// \return The currently configured maximum speed
    float maxSpeed(uint8_t motorId);

    /// Sets the acceleration/deceleration rate.
    /// \param[in] motorId The motor to set the acceleration for. If negative, sets the acceleration for all motors.
    /// \param[in] acceleration The desired acceleration in steps per second
    /// per second. Must be > 0.0. This is an expensive call since it requires a square
    /// root to be calculated. Dont call more ofthen than needed
    /// \param[in] deceleration The desired deceleration in steps per second. If zero, uses the acceleration setting.
    void setAcceleration(int8_t motorId, float acceleration, float deceleration = 0);

    /// Returns the acceleration/deceleration rate configured for this stepper
    /// that was previously set by setAcceleration();
    /// \param[in] motorId The motor to get the acceleration for.
    /// \return The currently configured acceleration
    float acceleration(uint8_t motorId);

    /// Returns the deceleration rate configured for this stepper
    /// that was previously set by setAcceleration();
    /// \param[in] motorId The motor to get the deceleration for.
    /// \return The currently configured acceleration
    float deceleration(uint8_t motorId);

    /// Sets the desired constant speed for use with runSpeed().
    /// \param[in] motorId The motor to set the speed for. If negative, sets the speed for all motors.
    /// \param[in] speed The desired constant speed in steps per
    /// second. Positive is clockwise. Speeds of more than 1000 steps per
    /// second may unreliable on Arduino hardware. Very slow speeds may be set (eg 0.00027777 for
    /// once per hour, approximately. Speed accuracy depends on the Arduino
    /// crystal. Jitter depends on how frequently you call the runSpeed() function.
    /// The speed will be limited by the current value of setMaxSpeed()
    void setSpeed(int8_t motorId, float speed);

    /// The most recently set speed.
    /// \param[in] motorId The motor to get the speed for.
    /// \return the most recent speed in steps per second
    float speed(uint8_t motorId);

    /// The distance from the current position to the target position.
    /// \param[in] motorId The motor to get the distance for.
    /// \return the distance from the current position to the target position
    /// in steps. Positive is clockwise from the current position.
    long distanceToGo(uint8_t motorId);

    /// The most recently set target position.
    /// \param[in] motorId The motor to get the target position for.
    /// \return the target position
    /// in steps. Positive is clockwise from the 0 position.
    long targetPosition(uint8_t motorId);

    /// The current motor position.
    /// \param[in] motorId The motor to get the current position for.
    /// \return the current motor position
    /// in steps. Positive is clockwise from the 0 position.
    long currentPosition(uint8_t motorId);

    /// Resets the current position of the motor, so that wherever the motor
    /// happens to be right now is considered to be the new 0 position. Useful
    /// for setting a zero position on a stepper after an initial hardware
    /// positioning move.
    /// Has the side effect of setting the current motor speed to 0.
    /// \param[in] motorId The motor to reset the position for.
    /// \param[in] position The position in steps of wherever the motor
    /// happens to be right now.
    void setCurrentPosition(uint8_t motorId, long position);

    /// Moves the motor (with acceleration/deceleration)
    /// to the target position and blocks until it is at
    /// position. Dont use this in event loops, since it blocks.
    /// \param[in] motorId The motor to run to the target position.
    void runToPosition(uint8_t motorId);

    /// Moves all motors (with acceleration/deceleration)
    /// to the target position and blocks until they are all at
    /// position. Dont use this in event loops, since it blocks.
    void runAllToPosition();

    /// Executes runSpeed() unless the targetPosition is reached.
    /// This function needs to be called often just like runSpeed() or run().
    /// Will step the motor if a step is required at the currently selected
    /// speed unless the target position has been reached.
    /// Does not implement accelerations.
    /// \param[in] motorId The motor to run to the target position.
    /// \return true if it stepped
    bool runSpeedToPosition(uint8_t motorId);

    /// Moves a single motor (with acceleration/deceleration)
    /// to the new target position and blocks until it is at
    /// position. Dont use this in event loops, since it blocks.
    /// \param[in] motorId The motor to run to the new target position.
    /// \param[in] position The new target position.
    void runToNewPosition(uint8_t motorId, long position);

    /// Sets a new target position that causes the stepper
    /// to stop as quickly as possible, using the current speed and acceleration parameters.
    /// \param[in] motorId The motor to stop. If negative, stops all motors.
    void stop(int8_t motorId);

    /// Disable motor pin outputs by setting them all LOW
    /// Depending on the design of your electronics this may turn off
    /// the power to the motor coils, saving power.
    /// This is useful to support Arduino low power modes: disable the outputs
    /// during sleep and then reenable with enableOutputs() before stepping
    /// again.
    /// If the enable Pin is defined, sets it to OUTPUT mode and clears the pin to disabled.
    /// \param[in] motorId The motor to disable outputs for. If negative, disables outputs for all motors.
    virtual void disableOutputs(int8_t motorId);

    /// Enable motor pin outputs by setting the motor pins to OUTPUT
    /// mode. Called automatically by the constructor.
    /// If the enable Pin is defined, sets it to OUTPUT mode and sets the pin to enabled.
    /// \param[in] motorId The motor to enable outputs for. If negative, enables outputs for all motors.
    virtual void enableOutputs(int8_t motorId);

    /// Gets the enabled/disabled state for the selected motor
    /// \param[in] motorId The motor to get the enabled state for.
    /// \return true if the motor is enabled, false if it is disabled.
    virtual bool enabled(uint8_t motorId);

    /// Sets the minimum pulse width allowed by the stepper driver. The minimum practical pulse width is
    /// approximately 20 microseconds. Times less than 20 microseconds
    /// will usually result in 20 microseconds or so.
    /// \param[in] minWidth The minimum pulse width in microseconds.
    void setMinPulseWidth(unsigned int minWidth);

    /// Sets the inversion for stepper driver pins for the selected motor
    /// \param[in] motorId The motor to set the pin inversion for.
    /// \param[in] directionInvert True for inverted direction pin, false for non-inverted
    /// \param[in] stepInvert      True for inverted step pin, false for non-inverted
    /// \param[in] enableInvert    True for inverted enable pin, false (default) for non-inverted
    void setPinsInverted(uint8_t motorId, bool directionInvert = false, bool stepInvert = false, bool enableInvert = false);

    /// Checks to see if a motor is currently running to a target
    /// \param[in] motorId The motor to check. If negative, checks all motors.
    /// \return true if the speed is not zero or not at the target position for the selected motor, or any motor if motorId is negative.
    bool isRunning(int8_t motorId);

    /// Virtual destructor to prevent warnings during delete
    virtual ~AccelStepper() {};

protected:
    /// \brief Direction indicator
    /// Symbolic names for the direction the motor is turning
    typedef enum
    {
        DIRECTION_CCW = 0, ///< Counter-Clockwise
        DIRECTION_CW = 1   ///< Clockwise
    } Direction;

    /// Forces the library to compute a new instantaneous speed and set that as
    /// the current speed. It is called by
    /// the library:
    /// \li  after each step
    /// \li  after change to maxSpeed through setMaxSpeed()
    /// \li  after change to acceleration through setAcceleration()
    /// \li  after change to target position (relative or absolute) through
    /// move() or moveTo()
    /// \return the new step interval
    virtual unsigned long computeNewSpeed();

    /// Low level function to set the motor output pins
    /// bit 0 of the mask corresponds to _pin[0]
    /// bit 1 of the mask corresponds to _pin[1]
    /// You can override this to impment, for example serial chip output insted of using the
    /// output pins directly
    virtual void setOutputPins(uint8_t mask);

    /// Called to execute a step. Only called when a new step is
    /// required. Subclasses may override to implement new stepping
    /// interfaces. The default calls step1(), step2(), step4() or step8() depending on the
    /// number of pins defined for the stepper.
    /// \param[in] step The current step phase number (0 to 7)
    virtual void step(long step);

    /// Called to execute a clockwise(+) step. Only called when a new step is
    /// required. This increments the _currentPos and calls step()
    /// \return the updated current position
    long stepForward();

    /// Called to execute a counter-clockwise(-) step. Only called when a new step is
    /// required. This decrements the _currentPos and calls step()
    /// \return the updated current position
    long stepBackward();

    /// Called to execute a step on a stepper driver (ie where pins == 1). Only called when a new step is
    /// required. Subclasses may override to implement new stepping
    /// interfaces. The default sets or clears the outputs of Step pin1 to step,
    /// and sets the output of _pin2 to the desired direction. The Step pin (_pin1) is pulsed for 1 microsecond
    /// which is the minimum STEP pulse width for the 3967 driver.
    /// \param[in] motorId The motor to step.
    virtual void step(uint8_t motorId);

    /// Current direction motor is spinning in
    /// Protected because some peoples subclasses need it to be so
    bool *_direction; // 1 == CW

    /// The current interval between steps in microseconds.
    /// 0 means the motor is currently stopped with _speed == 0
    unsigned long *_stepInterval;

private:
    uint8_t _count; // Number of steppers we are controlling

    /// Arduino pin number assignments for the 2 or 4 pins required to interface to the
    /// stepper motor or driver
    uint8_t _pin[4];

    /// Whether the _pins is inverted or not
    uint8_t _pinInverted[4];

    /// The current absolution position in steps.
    long _currentPos; // Steps

    /// The target position in steps. The AccelStepper library will move the
    /// motor from the _currentPos to the _targetPos, taking into account the
    /// max speed, acceleration and deceleration
    long _targetPos; // Steps

    /// The current motos speed in steps per second
    /// Positive is clockwise
    float _speed; // Steps per second

    /// The maximum permitted speed in steps per second. Must be > 0.
    float _maxSpeed;

    /// The acceleration to use to accelerate or decelerate the motor in steps
    /// per second per second. Must be > 0
    float _acceleration;
    float _sqrt_twoa; // Precomputed sqrt(2*_acceleration)

    /// The last step time in microseconds
    unsigned long _lastStepTime;

    /// The minimum allowed pulse width in microseconds
    unsigned int _minPulseWidth;

    /// Is the enable pin inverted?
    bool _enableInverted;

    /// Enable pin for stepper driver, or 0xFF if unused.
    uint8_t _enablePin;

    /// The pointer to a forward-step procedure
    void (*_forward)();

    /// The pointer to a backward-step procedure
    void (*_backward)();

    /// The step counter for speed calculations
    long _n;

    /// Initial step size in microseconds
    float _c0;

    /// Last step size in microseconds
    float _cn;

    /// Min step size in microseconds based on maxSpeed
    float _cmin; // at max speed
};

#endif
