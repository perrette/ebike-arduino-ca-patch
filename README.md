# CA path for an ebike with Arduino

## Motivation

The Cycle Analyst 3.1 (firmware 3.2b3) does not play out well with the SEMPU T2 torque sensor:
- the CA handles 36 poles max instead of the 48 poles in the sensor
- the motor kicks in when I press the pedal, as [I wrote on Endless Sphere](https://endless-sphere.com/sphere/threads/delayed-overshoot-in-torque-assist-after-pause-in-pedalling-sempu-t2-cycle-analyst.122328/)

## Solution

Use an Arduino instead...

More details will be posted soon. Here a draft as a placeholder.