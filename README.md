# Info / Instructions / Inspiration

We are building the version of the sand plotter shown here: https://www.youtube.com/watch?v=QRkoTvQqc7E
Watch out - the video shows different iterations of the project! Check towards the end for the one we're building

The original code and design files are here (updated/made better in this repo): https://drive.google.com/drive/folders/1fLfOw5r9TXtF1k8URlNvvU5tCRSYWfgB?usp=sharing
- I updated the design files into three laser cuttable sheets. Our steppers are much taller than the ones they use and the design files have been adapted for 40mm tall steppers

They use this kind of stepper: https://www.bastelgarage.ch/42x42x23mm-stepper-motor-nema-17-pancake-17hs4023-0-7a-0-15nm
We use this one: https://www.berrybase.de/en/act-motor-nema17-stepper-motor-17hs4417l20p1-x2-1.80-step-angle-0.4-nm-holding-torque-1.7a

Another version of the same project is here: https://www.instructables.com/Worlds-First-Cycloid-Art-Table-How-I-Built-This-Ar/
This has some instructions and has better code, but the construction is quite different so beware!

Extra CNC Shield Hacks can be found here: https://www.instructables.com/Arduino-CNC-Shield-V3-Enhanced-Capabilities-and-Fe/

The most work is needed on the code side, to get the code running properly and producing cool patterns. 
- In the end it would be great to the project running using Gcode i.e.
    - Universal Gcode Sender (https://winder.github.io/ugs_website/ and
    - Sandify (https://sandify.org/)
- But it might be too much for one weekend!

As well as building a sand plotter, we can also just build the base and come up with other uses for it

# How does it work - motors and code

- There are two motors - rotary (CW, CCW) and linear (IN, OUT)
- A XY target point is defined
- It is converted into angle and magnitude
- Steppers step towards it (at given speed?)
- When target point is reached, next target is selected

# Issues

- On startup, motors start and run for a long time before changing direction
- How can we slow it down to see better what's going on?
- Might we need to hack the board for independent powering on and off? Doesn't seem so from my tests
- How do we set/know the initial position? Always mount gear in centre? Move to end and move back?
- Serial commands in the code do basically seem to work

# What do the knobs do?

One of speed, one for LED brightness - I think!

# Is microstepping used?

No, jumpers need to be enabled for this - we don't need it!

# How many steps for a full turn for our motors? Is one step one cog turn?

- 1.8 degrees, so 200 steps for 360 degrees

# Serial Command List

'q': Toggles the enable state of the rotation (X) motor and, if the linear motor is off, toggles overall movement.
'w': Changes the direction of the rotation (X) motor.
'e': Increases the speed of the rotation (X) motor by 50 units.
'r': Decreases the speed of the rotation (X) motor by 50 units.
'a': Toggles the enable state of the linear (Y) motor and, if the rotation motor is off, toggles overall movement.
's': Changes the direction of the linear (Y) motor.
'd': Decreases the speed of the linear (Y) motor by 50 units.
'f': Increases the speed of the linear (Y) motor by 50 units.
'x': Commands the system to plot the star shape.
'z': Commands the system to plot the line shape.
'Y': Rotates the linear axis 360 degrees by stepping the motor 200 times with a fixed delay.
