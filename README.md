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

### Constants and global Variables

Before digging into the logic behind the procedure itself we should begin by looking at the series of global variables we have defined:

1. **ARDUINO_BAUDRATE:** The firmware running in the Arduino board will set up the serial port at a given rate. This data will be used when opening the port.

2. **REDUCING_FACTOR:** The coupling between the motors and the wheels themselves doesn't necessarily have to be 1 (that is, one motor revolution is equivalent to one wheel revolution). This should be taken into account when estimating the initial value of the pulse -> mm conversion factor C<sub>m</sub>.

3. **ENC_PULSES_PER_REV:** This value is encoder-specific. Our encoders generate 3 pulses per **MOTOR** revolution, for example.

4. **NOM_DIAMETER:** The nominal diameter is the one we expect our wheels to have when buying them. We will see that the "real" diameter has a small deviation with respect to this value.

5. **WHEELBASE:** We can take an approximate measurement of the wheelbase with some measuring tape. We measured from the center of one wheel to the center of the other one. This measurement should be close to the real wheelbase, but some deviation is tolerable. This is one of the things we want to calibrate after all!

6. **N_TURNS**: The number of turns we the robot will carry out when calibrating. The more turns it does, the more accurate the calibration will become but the longer the calibration will take. Please take into account that the firmware will only get 2000 measurements, so after these measurements are taken doing more turns will be completely pointless. We carried out our experiments with 8 turns.

7. **E<sub>D</sub>, E<sub>S</sub>, K:** These are constants needed for the calibration as explained above.

8. **PULSES_PER_REV:** If our *REDUCING_FACTOR* is different than 1 the we will see how the pulses the encoder emits per revolution is **NOT** the same as the number of pulses emitted when the wheel experiences one revolution. In our case, the reducing factor is 50,9. This means that our wheel will experience an entire revolution when the motors have carried out 50,9 revolutions! Then, as our encoder emits 3 pulses per **MOTOR** revolution we would find that for a wheel rev (that's why we have computed this value) becomes 50,9 * 3 = 152,7 pulses / wheel<sub>REV</sub>.

9. **MM_TO_PULSES:** This is the mm -> pulses conversion factor we have talked about before (C<sub>m</sub>). It is obtained as the quotient between the circumference of the wheels and the number of pulses per wheel revolution, namely: MM_TO_PULSES = PI * NOM_DIAMETER / PULSES_PER_REV.

10. **ESTIMATED_PULSES_PER_TURN:** When turning with one of the wheels stopped, the moving one will describe a circumference of length 2 &ast; PI &ast; WHEELBASE. Then, the wheel will have experienced (2 &ast; PI &ast; WHEELBASE) / (PI &ast; NOM_DIAMETER) revolutions. Knowing the PULSES_PER_REV of the wheel we can just find this parameter as: [(2 &ast; PI &ast; WHEELBASE) / (PI &ast; NOM_DIAMETER)] &ast; PULSES_PER_REV.

11. **REAL_PULSES_PER_TURN:** This is the calibrated value we will obtain after computations.

12. **Add the remaining ones!**

### Interfacing section

The main aim of this module is opening the serial port for Arduino and getting the initial data we need as input for the calibration. We will also send the different commands to trigger the needed movements of the robot. We will finally recover the data and store it in files to be processed later on.

#### Function breakdown

1. *`initial_data()`*: We need the estimated measures of the robot as a starting point for the calibration, namely the nominal diameter (the one the wheels are supposed to have) and the wheelbase. We have also included a handful of other input data in an attempt to make the program as general as possible. These include the number of pulses the motor's encoder emits per revolution, the reducing factor between the motor and the wheels if any and Arduino's baud-rate. Whilst getting the data, the function updates a series of global variables containing the basic data of the robot and computes any needed secondary parameters. It will also generate the command we need to make the robot do a 360&deg; turn. All the commands, are explained in the [**Annex**](#Annex). We would like to draw attention to the part where we take the logarithm in base 10 of a number. We do so to find the number of digits in said number so that we can "craft" the command with the appropriate format.

2. *`print_updated_data()`*: This function just prints the global variables to the screen. It has been made mainly for debugging purposes, but it can be used to provide additional feedback to the end user. It ends up by prompting the user whether to continue or halt the process.

3. *`find_N_open_serial_port()`*: This function is the first one to use the PySerial library. It will try to open every serial port. As ports with nothing connected to them will raise a serial exception we just handle it by continuing the loop. When we open a serial port with something connected we try to check whether we have an Arduino board connected or not to the Raspberry Pi. To check all the possible ports we have used a RegExp (Regular Expresion) and then broken down the different paths with the function `glob()` Due to the nature of serial devices it is extremely difficult to get any information about the manufacturer. When dealing with a Linux-based system, attaching a serial device triggers the creation of a file in the `/dev/serial/by-id` folder. This file contains a fixed string that can be used to identify a device. By calling Linux's `ls` command on this file we can check whether an Arduino board is connected or not. The catch is that is we have several boards we cannot know for sure which port belongs to the Arduino board. When working with original Arduino boards we can just call the `dmesg` command and we will see a message containing the model of the board as well as the port where it's been attached. By parsing `dmesg`'s output we could know where our MCU is for sure. Another way of checking for a specific board would be by looking up the `VendorID` number from the `dmesg` command and query an online database for the manufacturer number and proceed just like with an original chip. We can see how this is quickly getting out of hand, so we have decided to stick with the first approach, as we are handling a very specific environment. In order to read the output of the `ls` command we have created a temporary file. The returned value by ls is either a 0 or an error code, depending on whether the command worked or failed. This little fix circumvents that issue. The return value of this function is the port we have opened to the Arduino.









## Annex <a name="Annex"></a>
