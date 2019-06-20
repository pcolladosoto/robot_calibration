# UAH Calibration for Differential Robots

We have written a Python script that automatically calibrates the needed data to compute the position of the robot through odometry.

This software has been written as part of the **Padrino Tecnológico** initiative from the *University of Alcalá, Spain*.

The theory behind the methods we propose can be found over at [**Hindawi's Journal of Sensors**](https://www.hindawi.com/journals/js/2019/8269256/?utm_medium=author&utm_source=Hindawi).

## Rough Theoretical Summary

The main approach for differential robot location has traditionally been odometry. As we "know" the diameter of our wheels and the number of revolutions they turn we can theoretically compute the linear displacement of the robot as well as the final direction it's heading. The "bad thing" is that our errors will be cumulative, meaning that after a few meters our estimation of the position will be practically worthless.

This deviation is due to both systematic and non-systematic errors:

* **Systematic errors**: They are due to the characteristics of the robot itself and will (usually) remain constant through a robot's lifespan.

* **Non-systematic errors**: They are caused by he interaction of our robot with the medium. Examples are wheel slippage when going through a slippery area or unpredictable non-flat terrain.

Due to the inherent unpredictability of non-systematic errors, the ones we can try to compensate for are the systematic ones by means of calibration. The UAH's proposal tries to simplify the calibration process.

Before starting our discussion we should note how our motors have encoders attached to them, so that with every turn of a motor we record and encoder-specific number of pulses. We uses these signals to derive the different data we need throughout the process.

## UAH's Calibration Procedure

The parameters we will try to "correct" with calibration are:

* **E<sub>d</sub>**: This factor accounts for the difference in diameter of both wheels. Due to manufacturing alone these diameters cannot be guaranteed to be exactly equal... This factor is expressed as the ratio of both wheels' diameter (E<sub>d</sub> = D<sub>L</sub> / D<sub>R</sub>).

* **C<sub>m</sub>**: This conversion factor changes received pulses to mm and is given by C<sub>m</sub> = PI * Diameter / Pulses<sub>REV</sub>.

* **Wheelbase (b)**: This parameter accounts for the distance from the contact point of the right wheel with the floor to the contact point of the left one with the floor as well. As the contact zone is an area rather than a point there will always be some uncertainty as to what is the *effective* wheelbase (b<sub>act</sub>).

We only have to adjust 3 elements, that's great! But... How should we do it?


### Adjusting **E<sub>d</sub>**

As shown in the reference article, we can relate the relation of both wheels' diameter to the number of pulses we record when doing a full turn when one of them is stopped. We find these pulses indirectly thanks to a few ultrasound **HC-SRO4**. As we are turning around these distances will be somewhat periodic, so after taking the auto-correlation of these distances we can find the number of emitted pulses by looking for a maximum! We can then show how the ratio of these pulses is equivalent to the ratio of both diameters, which happens to be, you guessed it, E<sub>d</sub>.

### Adjusting **C<sub>m</sub>**

This factor is arguably the easiest one to adjust. Assuming our robot's deviation is quite small (under 2 deg) we can neglect its associated error. If we make it move for **d** mm and we record **n** pulses, the real **C<sub>m</sub>** factor will be given by C<sub>m</sub> = d / n.

### Adjusting **b**

When we make the robot turn while maintaining one of the wheels stopped we are roughly describing a circumference of length 2 * PI * b. This distance should be exactly the same as the number of pulses emitted by the moving wheel times the conversion factor we have computed previously (C<sub>m</sub>). In the literature you will see how we have a constant **K** appearing in the final expression. This is due to the introduction of a piece with a bearing whose aim is to facilitate the turning of the robot during the tests. How to take this factor into account is shown in the literature.

## High level view of the automation script

This script is intended to run on a **Raspberry Pi B Rev 1.2** mounted as an on-board computer. As the robot is controlled by an **Arduino Mega** micro-controller we have to interface with it by means of a serial port. This first challenge will be dealt with in the ***Interfacing section*** below.

Once we manage to send orders to the robot and then collect the data we need we need to tackle the elephant in the room, how to treat and process that data so that we can extract our desired results? This will be assessed in the ***Computations section***.

In each of the aforementioned sections we will break down every single function so that the make the overall process be as clear as we possibly can. Let's get to work, shall we?

### Interfacing section
