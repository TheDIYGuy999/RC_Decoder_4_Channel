#This is an Arduino 4 channel RC signal to DC motor converter shield
## Features:
- Reads 4 standard RC servo signals and converts them to control 4 DC motors
- Auto channel zero adjustment during setup()
- 5V, 16MHz or 3-3v / 8MHz Pro Micro
- DRV8833 H-Bridge driver, controlled with my DRV8833 Arduino library
- Motor 1 & 2 can be used to drive a caterpillar vehicle:
* direct mode (2x throttle, one for each caterpillar)
* caterpillar mode means, you have one throttle and one steering input. The speed of the inner wheel can be fully reversed, up to 100%. The vehicle then turns "in place"
* semi caterpillar mode. Same as above, but the speed of the inner wheel can only be reduced about 30%
* arrays for steering behaviour calibration, now with nonlinear mapping
* two additional motor outputs for other vehicle functions

## Usage

See pictures in this repo

(c) 2016 TheDIYGuy999
