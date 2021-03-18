# ATMEL_SFU_BOT


SMARTFARM bottom board sources


Board : Custom board (ATSAMD21J18a)


The role of the bottom board is land control based on the RTC, which operates through feedback controls associated with sensors and actuators.
The role of the top board is to measure illumination and communicate with the user.

It has a local clock counted through rtc callback, and operates the actuator to maintain the condition set by the user according to the sensor value at a certain time.

It is also possible to operate the actuator at a specific time A through B using the scheduling function.
