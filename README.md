source ~/esp/v5.2.3/esp-idf/export.sh

idf.py menuconfig

clear && idf.py build flash monitor

idf.py fullclean

## Summing waves and overflowing values.
There is a problem with summing waves. The problem is that when you add two waves together, there is a chance that the resulting value goes out of range. I.e. the summed value is greater than 32767 or less than -32767 for a 16bit sample. 

The solution to this problem I've gone for is Floating-Point Summing. For every buffer, you calculate the min and max value, and then you normalise the sample for your -32767 to 32767 range. The downside to this is, of course, that the normalisation may be different from one buffer to the next, which may create audio artifacts such as clipping. 

## Phase shift
Another potential problem is that all your waves start in the same phase, and you get artifacts as a result. A solution to this could be to phase shift the waves in relation to each other. 

One way to mitigate this would be to randomise the initial phase of any new waves. The downside is that it could randomly end up in a bad place.

Another way would be to for each new wave calculate its peak in in a 512 sample (stereo) buffer, and the time to get to that first peak. This time-to-first-peak would be stored as a variable in the wave. Each new wave would then have to pick a starting phase which would be as far away from the other peaks as possible (or mulitples of said peaks). There are ways for this to go wrong aswell, when dealing with square waves or irregular waves, for example. However, this would probably be preferable to the randomisation method....Provided that it wouldn't take too long time, of course.

## Problem: Stuttering
The audio is stuttering for some reason. Apparently I'm not the only one (https://www.esp32.com/viewtopic.php?t=10140)