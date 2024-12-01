SRStepper was forked from AccelStepper by Mike McCauley. The important bits - the acceleration/deceleration routines - are retained,
but a significant amount of AccelStepper's motor compatibility is removed (because I don't need it). On the other hand, it gains
some new functionality related to how acceleration/deceleration is handled within a single move.

SRStepper is designed to be used within a chain of 74'595 shift registers. Each register bit is mapped to a pin on a typical stepper
driver board (e.g. the DRV8825 or A4988). Whilst the code has been written for the DRV8825, it should be adaptable to any board that
has up to 8 control pins.

For the DRV8825, the supported pins are:

  | Label | Description | 74'595 Pin |
  |-------|-------------|------------|
  | `STP` | Step        | QA         |
  | `DIR` | Direction   | QB         |
  | `SLP` | Sleep       | QC         |
  | `RST` | Reset       | QD         |
  | `M0`, `M1` and `M2`| Step config | QE-QG      |
  | `EN`  | Enable      | QH         |

Note that Sleep, Reset and Enable are all active `low`.


This was the Arduino AccelStepper library. It provided an object-oriented interface for 2, 3 or 4 pin stepper motors and motor drivers.

Now it only provides a single option - 2-pin stepper drivers (step + direction), with ancillary functions such as sleep, enable, reset
and micro-step configuration all programmable; and the constructor takes pins for a 74'595 shift register as parameters, instead of 
motor pins.  

AccelStepper significantly improved on the standard Arduino Stepper library in several ways. SRStepper retains much of these
improvements, drops some of them, and adds new ones:

  - Supports acceleration and deceleration
  - Supports multiple simultaneous steppers, with independent concurrent stepping on each stepper
  - API functions never delay() or block
  - ~~Supports 2, 3 and 4 wire steppers, plus 3 and 4 wire half steppers. (nope - only driver boards)~~
  - ~~Supports alternate stepping functions to enable support of AFMotor (nope)~~
  - Supports stepper drivers such as the Sparkfun EasyDriver (based on 3967 driver chip)
  - Very slow speeds are supported
  - Extensive API (I guess)
  - Subclass support (Buggered if I ever got that to work. I hate C++)

In addition:
  - Supports any number of steppers via daisy-chained 74'595s
  - Supports different acceleration & deceleration rates on the same move
  - Programmable micro-stepping, from whole steps to 1/32nds (where the driver board supports this)
  - Auto sleep, or stay awake, depending on desired use case
