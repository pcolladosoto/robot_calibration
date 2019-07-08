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

As shown in the reference article, we can relate the relation of both wheels' diameter to the number of pulses we record when doing a full turn when one of them is stopped. We find these pulses indirectly thanks to a few ultrasound **HC-SRO4**. As we are turning around these distances will be somewhat periodic, so after taking their auto-correlation we can find the number of emitted pulses by looking for a maximum. We can then show how the ratio of these pulses is equivalent to the ratio of both diameters, which happens to be, you guessed it, E<sub>d</sub>.

We would like to shed some light on why this maximum coincides with the number of pulses it takes our robot to complete one turn. The auto-correlation of a function is the convolution with itself inverted, as we will see later on. In order to visualize this reasoning picture a discrete, periodic signal whose period is `[1, 1, 2]`. As we take the convolution we will observe how less and less samples overlap and therefore add to the overall expression. The catch here is that adding less samples is **NOT** equivalent to having a smaller overall result! An example of this would be that `1·1 + 1·1 + 2·1 < 1·1 + 2·2`. Notice how in the second case we have less "overlapping" but the overall result is larger! Nevertheless, the overall tendency of this auto-correlation is to diminish as we will see in some of the attached figures below. In some way, when we are moving `N·PERIOD` samples we are "multiplying the big numbers by the big numbers" so that they have more "weight" and the "small numbers by the small numbers", so that the "big numbers" are adding all they can to the overall contribution. It helps to think of the static signal as the "weights" of the moving samples.

### Adjusting **C<sub>m</sub>**

This factor is arguably the easiest one to adjust. Assuming our robot's deviation is quite small (under 2 deg) we can neglect its associated error. If we make it move for **d** mm and we record **n** pulses, the real **C<sub>m</sub>** factor will be given by C<sub>m</sub> = d / n.

### Adjusting **b**

When we make the robot turn while maintaining one of the wheels stopped we are roughly describing a circumference of length 2 &ast; PI &ast; b. This distance should be exactly the same as the number of pulses emitted by the moving wheel times the conversion factor we have computed previously (C<sub>m</sub>). In the literature you will see how we have a constant **K** appearing in the final expression. This is due to the introduction of a piece with a bearing whose aim is to facilitate the turning of the robot during the tests. How to take this factor into account is shown in the literature.

## High level view of the automation script

This script is intended to run on a **Raspberry Pi B Rev 1.2** mounted as an on-board computer. As the robot is controlled by an **Arduino Mega** micro-controller we have to interface with it by means of a serial port. This first challenge will be dealt with in the ***Interfacing section*** below.

Once we manage to send orders to the robot and then collect the data we need we need to tackle the elephant in the room, how to treat and process that data so that we can extract our desired results? This will be assessed in the ***Computations section***.

We finally find the ***Miscellaneous section*** where we have additional functions whose functionality doesn't exactly fit in any of the above.

In each of the aforementioned sections we will break down every single function so that the make the overall process be as clear as we possibly can. Let's get to work, shall we?

### Used libraries

In order not to "reinvent the wheel" over and over again we have taken advantage of several libraries to make the code faster and more legible. The functionalities they contribute to our program are:

1.  

### Constants and Global Variables

Before digging into the logic behind the procedure itself we should begin by looking at the series of global variables we have defined. We would also like to make a few comments on the structure this data will have.

By taking a look at the source code you can see how we have defined 5 dictionaries. A dictionary is just a generalization of a list (or array for those whose background is in C) that can be indexed by strings, not just integers. That feature lets us give descriptive names to each of the variables whilst attaining the consistency a single data structure provides. Each of these dictionaries contains data relative to a given aspect of the script.

One could argue that having all the data contained in dictionaries bloats the syntax through the script. It's true that the program could have been more compact if we had only used global variables instead of global dictionaries, but the benefits and clearer structure that derive from these approach outweighed the cons that arise from having to index the dictionaries over and over again.

#### Data Constants and result "holders"

We should begin by pointing out that we will find both constants that will be computed in the fly and variables that will contain the final result of our experiments. Not all the following values are what we traditionally know as CONSTANTS like the ones in **C**, for example.

All these values can be found in the `CONSTANTS` dictionary, where the value of each of the following is indexed by a string containing **EXACTLY** the names given below.

1. `ARDUINO_BAUDRATE:` The firmware running in the Arduino board will set up the serial port at a given rate. This data will be used when opening the port. The firmware's default is 38400 bauds.

2. `REDUCING_FACTOR:` The coupling between the motors and the wheels themselves doesn't necessarily have to be 1 (that is, one motor revolution is equivalent to one wheel revolution). This should be taken into account when estimating the initial value of the pulse -> mm conversion factor C<sub>m</sub>.

3. `ENC_PULSES_PER_REV:` This value is encoder-specific. Our encoders generate 3 pulses per **MOTOR** revolution, for example.

4. `NOM_DIAMETER:` The nominal diameter is the one we expect our wheels to have when buying them. We will see that the "real" diameter has a small deviation with respect to this value.

5. `WHEELBASE:` We can take an approximate measurement of the wheelbase with some measuring tape. We measured from the center of one wheel to the center of the other one. This measurement should be close to the real wheelbase, but some deviation is tolerable. This is one of the things we want to calibrate after all!

6. `N_TURNS`: The number of turns we the robot will carry out when calibrating. The more turns it does, the more accurate the calibration will become but the longer the calibration will take. Please take into account that the firmware will only get 2000 measurements, so after these measurements are taken doing more turns will be completely pointless. We carried out our experiments with 8 turns.

7. `N_REPS`: In order to increase the accuracy of the test we have included an option that lets the user repeat the same test as many times as they want whilst keeping track of the outcomes. The script will return the average of all these tests, making our data be more coherent.

8. `ED, ES, K, DR:` These are constants needed for the calibration as explained above.

9. `PULSES_PER_REV:` If our *REDUCING_FACTOR* is different than 1 the we will see how the pulses the encoder emits per revolution is **NOT** the same as the number of pulses emitted when the wheel experiences one revolution. In our case, the reducing factor is 50,9. This means that our wheel will experience an entire revolution when the motors have carried out 50,9 revolutions! Then, as our encoder emits 3 pulses per **MOTOR** revolution we would find that for a wheel rev (that's why we have computed this value) becomes 50,9 * 3 = 152,7 pulses / wheel<sub>REV</sub>.

10. `MM_TO_PULSES:` This is the mm -> pulses conversion factor we have talked about before (C<sub>m</sub>). It is obtained as the quotient between the circumference of the wheels and the number of pulses per wheel revolution, namely: MM_TO_PULSES = PI * NOM_DIAMETER / PULSES_PER_REV.

11. `ESTIMATED_PULSES_PER_TURN:` When turning with one of the wheels stopped, the moving one will describe a circumference of length 2 &ast; PI &ast; WHEELBASE. Then, the wheel will have experienced (2 &ast; PI &ast; WHEELBASE) / (PI &ast; NOM_DIAMETER) revolutions. Knowing the PULSES_PER_REV of the wheel we can just find this parameter as: [(2 &ast; PI &ast; WHEELBASE) / (PI &ast; NOM_DIAMETER)] &ast; PULSES_PER_REV.

12. `REAL_PULSES_PER_TURN`: This is the calibrated value we will obtain after computations.

13. `NOISE_THRESHOLD`: During our experiments we have observed how the ultrasound sensors aren't immune to noise. Having an approximate measuring range of 1,5 meters, we saw how the gave back measurements in the [1,6; 1,7] m range. As these are **NOT** real measure we decided to filter them out by imposing a necessary minimum value when building our signal. Said requirement is controlled by means of this constant.

14. `WINDOW_LIMITS`: When looking for a maximum in our signal we are not dealing with the entire signal, which can become quite large. We only consider values within a certain window instead. As we know what the expected result should look like (as recorded in the `ESTIMATED_PULSES_PER_TURN` constant), we can be pretty sure the real value will be found within a range containing our expected result.

#### Commands

These constants contain the commands we will send to the MCU. The ones we find defined have been computed taking our platform's defaults into account.

They can be found within the `COMMANDS` dictionary. As with the `CONSTANTS` one, the indices are the commands below given as strings.

1. `GET_DATA = 'F'`

2. `TURN_L_WHEEL_STOPPED = 'C0000371'`

3. `TURN_R_WHEEL_STOPPED = 'D0000371'`

4. `TURN_BOTH_WHEELS = '2360'`

5. `GO_STRAIGHT_3M = '43000'`

6. `GO_STRAIGHT_1M = '41000'` (used for debugging)

For a discussion on the syntax please refer to the [**Annex**](#Annex).

#### Error Constants

The `ERROR_MESSAGES` dictionary contains all the strings we can use to shed some light on the issues at hand. After thorough testing we stopped using these messages in the main code, but we have left them in case the user finds them useful for any purpose.

As with the above, these messages can be found within the aforementioned dictionary. The indices are `"PORT_NOT_FOUND"`, `"PORT_NOT_REACHABLE"` and `"ERROR_SIGNAL_TREATMENT"`. These are quite self explanatory. Their associated message are found within the dictionary itself, so we encourage you to take a look.

#### Switches

These variables are used to control the flow of the program depending on the user configuration. They are contained within the `SWITCHES` dictionary and, except for the `DBG` flag, they are should **NOT** be directly modified by the user in the source code. They will be adjusted on the fly. Their uses are:

1. `VERBOSE`: This switch will be configured during the initial setup and will be set to `True` by default. It controls how much information about the process is fed back to the user.

2. `DBG`: This flag should be used for enabling debugging. It just controls whether or not to delete the temporary data files we extract from the micro-controller. This is the only parameter that has to be changed in the source code.

3. `COMPUTE_XXX`: These variables are used to keep track of the tests that have been carried out and skipped so that we only compute those values whose data we have. If not taken into account, division by zero errors are quite common if any tests are skipped...

4. `PHASES_XXX`: As with the ones before, the use of these flags is to keep track of what intermediate values have been obtained so that we don't trigger any errors when we continue computing.

#### Terminal Colors

As our interface is just the CLI (Command Line Interface) we have decided to use colors to our advantage to make the script more visually appealing whilst also making the output data easier to interpret. As our programming knowledge is mainly based on C, we decided to go the "low-level" way and use ANSI escape sequences. When the terminal reads these codes it changes the color it uses to print to the screen. Another popular example of this method can be found in the `.bashrc` files in the home directory of a Linux based OS. This is how the command prompt is colored, at least in my `UBUNTU 18.04` machine. We know that there is a `termcolor` library that achieves the same result, but our approach exposes the inner workings of the process and is overall simpler to grasp in our opinion.

ANSI escape sequences are actually quite interesting, so we encourage you to take a look at the [*Wikipedia page*](https://en.wikipedia.org/wiki/ANSI_escape_code) covering them.

### Interfacing section

The main aim of this module is opening the serial port for Arduino and getting the initial data we need as input for the calibration. We will also send the different commands to trigger the needed movements of the robot. We will finally recover the data and store it in files to be processed later on.

#### Function breakdown

1. `initial_data()`: We need the estimated measures of the robot as a starting point for the calibration, namely the nominal diameter (the one the wheels are supposed to have) and the wheelbase. We have also included a handful of other input data in an attempt to make the program as general as possible. These include the number of pulses the motor's encoder emits per revolution, the reducing factor between the motor and the wheels if any and the Arduino board's baud-rate. Whilst getting the data, the function updates a series of global variables containing the basic data of the robot and computes any needed secondary parameters. It will also generate the command we need to make the robot do a 360&deg; turn. All the commands, are explained in the [**Annex**](#Annex). We would like to draw attention to the part where we take the logarithm in base 10 of a number. We do so to find the number of digits in said number so that we can "craft" the command with the appropriate format. This function will also be in charge of configuring the behavior of the program based on the user's preferences. To accept a default value just press ENTER without typing anything.

2. `print_updated_data()`: This function just prints the global variables to the screen. It has been made mainly for debugging purposes, but it can be used to provide additional feedback to the end user. It ends up by prompting the user whether to continue or halt the process.

3. `find_N_open_serial_port()`: This function is the first one to use the PySerial library. It will try to open every serial port. As ports with nothing connected to them will raise a serial exception we just handle it by continuing the loop. When we open a serial port with something connected we try to check whether we have an Arduino board connected or not to the Raspberry Pi. To check all the possible ports we have used a RegExp (Regular Expresion) and then broken down the different paths with the function `glob()`. Due to the nature of serial devices it is kind of difficult to get information about the manufacturer. When dealing with a Linux-based system, attaching a serial device triggers the creation of a file in the `/dev/serial/by-id` folder. This file contains a fixed string that can be used to identify a device. By calling Linux's `ls` command on this file we can check whether an Arduino board is connected or not. The catch is that is we have several boards we cannot know for sure which port belongs to the Arduino board. When working with original Arduino boards we can just call the `dmesg` command and we will see a message containing the model of the board as well as the port where it's been attached. By parsing `dmesg`'s output we could know where our MCU is for sure. Another way of checking for a specific board would be by looking up the `VendorID` number from the `dmesg` command and query an online database for the manufacturer number and proceed just like with an original chip. We can see how this is quickly getting out of hand, so we have decided to stick with the first approach, as we are handling a very specific environment. In order to read the output of the `ls` command we call the `os` module and read its return value. We then only check whether the target string (that depends on the hardware we are connecting) is within `ls`'s output. If so, we open the serial port to our MCU. The return value of this function is the port we have opened to the Arduino.

5. `execute_command([string object] command, [port object] port)`: This function is just a wrapper that sends orders to the Arduino board. After sending the command, it will remain in an idle state until it reads two dots (`.`) from the serial port. This is due to the fact that the firmware starts sending dots when it has finished carrying out the requested action. These dots are our "stop condition" after all. Again, the commands are explained in the [**Annex**](#Annex).

6. `get_data([port object] port)`: This functions triggers the output of data from the board and waits for the `END` string signaling that there is no more data to come. All this data will be stored on a text file for further processing. The file will be returned with the R/W pointer at the beginning.

7. `get_straight_data([port object] port)`: This function is in charge of getting the recorded pulses when carrying out the last test. We have decided to use another function because the string format we have to read and process is not the same than it was before. It is based on regular expressions, just like the function `extract_data()`. In order to know a little bit more about its inner workings please refer to the aforementioned function down below.

##### Small firmware tweaks

We not only need to send orders to the Arduino board, but we also need to update the value of some parameters! As this hadn't been designed we had to get our hands dirty and write two new functions for the MCU's software. These are:

1. `parse_input()`: Once called, the function will start to read the input from the serial port byte by byte. If the byte is a digit it will store it in a C-string (character array) and if it is not it will call a function called `update_param()`. After calling said function we reset the number array with `NULL`s. This approach let's us be flexible and accept numbers ranging from 1 to 4 digits. The specifics of the command string will be discussed in the [**Annex**](#Annex).

2. `update_param(char* number, char parameter)`: Once called, we will update a parameter depending on the input argument parameter, which is just a `char`. The new value is the `float` equivalent of the number string `number`.

#### Computations section

In this section we will see how we go from having RAW text input to sanitizing it and extracting a calibrated value for each of the needed parameters. As before, we have broken down the task into several functions:

1. `read_N_parse([file object] data, [python list] p_array, [python_list] d_array)`: The input data to this section is contained in a file. The data we get from the Arduino contains the number of encoder pulses, a space, the distance measured by the sensors in centimeters, a space, and a semicolon followed by 3 dots. It looks something like: `XXXX YYYY ;...`, where `XXXX` and `YYYY` are the pulses and distances respectively. As the number of digits may vary we found it cumbersome to check each line byte by byte, so we took advantage of the power of RegExps. This function will read the input file data line by line whilst making sure "rubbish" lines are not taken into account. The data will be extracted by our next function, `extract_data()`, which we will call for every line in the document.

2. `extract_data([python_list] string_to_parse, [python_list] p_array, [python_list] d_array)`: Using the function `findall()` from the `re` library we are able to extract the data we need and begin our process. The RegExp we are using matches ASCII digits (remember RegExps only work on text) so we need to convert them to integer values before appending them to their corresponding arrays. One might argue that we could condense these 2 first functions int one. Whilst that is absolutely true we believe this approach is more modular and improves overall code readability and clarity, something we prefer to prioritize.

4. `populate_signal([python_list] pulses_array, [python_list] distances_array)`: Before understanding the way this function operates, we should take a closer look at the nature of the signals handed to us as input for this module.
<br><br>
First of all consider that in our case the encoders will emit 152,7 pulses per wheel revolution. Given that our tests were carried out by performing 8 turns and that a turn implies a fair number of wheel revolutions we are looking at a considerable quantity of pulses. Due to design constraints of the firmware, these pulses are tracked with an unsigned char whose values lies in the range [0, 255]. Let's face it, overflow is going to happen... This condition is checked by looking at the current and past number of pulses. As the number of pulses is monotonously increasing, we know that if a future index is larger than a past an overflow has taken place! In order to keep track of these overflows we have defined a variable `n_jumps`. We will add 256 to our current index for each overflow we have detected, so that the number of pulses we are using as index in our signal is in agreement with what it should be if we didn't have to cope with the char data type size constraints.
<br><br>
Due to firmware limitations we will see that we don't have a distance measurement for each encoder pulse. We will instead have measurements for certain pulse quantities, and no information for pulses in between said quantities. We chose to hold the last registered values for these "dark" intervals, so that's why we have a while loop within the for loop, we are just assuming the distance didn't change too much so that we can say it's roughly the same.
<br><br>
when appending distance information to the array we are building we can see a conditional check that looks at the `NOISE_THRESHOLD` constants and filters out longer (with a higher value) pulses.
<br><br>
After finishing, this function returns the real distance signal we need to process to find our parameters. We are almost there!

6. `convolution_time([python_list] original_signal, [python_list] reversed_signal)`: This function returns the convolution of the inputs. As discussed in the linked article at the beginning of this document, the auto-correlation of a signal is equivalent to the convolution of the signal with itself but reversed. In math terms: X[n] &ast; X[-n] (j) = R(j), where R(j) is the autocorrelation of X[n] as a function of j and X[n] &ast; X[-n] (j) is the convolution of X[n] with itself reversed as a function of j too. Again, one could argue that functions **7** and **6** could have been joined into only one. We have taken this approach for the sake of readability.

7. `find_maximum([python_list] convoluted_signal)`: This function is more a signal processing problem than a programming one. As seen in the linked article, the profile of the auto-correlation will show clearly identifiable peaks where an entire turn has been detected. As we are interested in the number of pulses for this to happen we will be looking for the index where we find a maximum value.
<br><br>
The catch here is that we won't be looking at the entire signal, but only at a portion of it between two limits we define: `lower_limit` and `upper_limit` in the code (no surprise!). As math assures us there will be a maximum between these we just have to look for this maximum and record the index where it takes place, which we do with a normal `for` loop. The function will return the value where we have this maximum, which directly depends on the index we found above.

#### Misc functions

These functions don't precisely fit into any of the two categories above, so we have decided to include all of them here. Their purpose goes from handling program flow to showing data along the way.

1. `IRQ_setup()`: During testing we halted the program through a keyboard interrupt (**CTRL + C**) more often than not, we we decided to treat this interrupt and close the program gracefully. We have used the `signal` library for this purpose. The configuration of the interrupt is done is this function, where we attach the handler `handler()` to the interrupt `signal.SIGINT`, which is the one the keyboard interrupt raises.

2. `handler()`: This function will trigger on the reception of a keyboard interrupt and just exit the program after printing that the script is about to be terminated.

3. `user_tweaks([string] mode)`: This function is in charge of handling the program flow. The function is called after any important section and it clears the screen whilst waiting for user input to continue. This is a way of letting the user have a tighter control on the program and not the other way around whilst clarifying the operation off the script altogether. The mode parameter has been left to enable additional functionality if required in the future.

4. `show_signal([list] input_signal, [string] y_label, [string] title)`: This function is in charge of plotting any `input_signal` we feed it. The `y_label` and `title` string set those elements within the graph to give it a little bit more context. The back-end we use for plotting is `matplotlib`, even though we are not even scratching its power in our implementation.

## Annex <a name="Annex"></a>

### Commands for interfacing with Arduino's Firmware

The following commands are sent as plain-text through a serial port:

1. `F`: This commands triggers the firmware to print information relative to the pulses and distances to the serial port. This is the information we will save into a file for later processing.

2. `S[X...X[WMD]]`: This notation is quite tricky, but the idea is simple. This command **SETs** the calibrated values. `X...X` implies that we can have any number of digits (up to MAX_DIGS, defined in the firmware). `[WMD]` means that we can update any of these parameters in the order we want:

  * `W`: Update the wheelbase.
  * `M`: Update the MM -> pulses conversion factor.
  * `D`: Update the wheel diameter deviation E<sub>d</sub>

  And the overall square brackets means we can update one, two or the three parameters with only one command. We have tried to make it as flexible as possible!

3. `C000XXXX`: This makes the robot turn whilst maintaining the left wheel stopped. `XXXX` is the distance in centimeters the moving wheel (i.e the right one) has to traverse.

4. `D000XXXX`: This makes the robot turn whilst maintaining the right wheel stopped. `XXXX` is the distance in centimeters the moving wheel (i.e the left one) has to traverse.

5. `2XXX`: Turn with both wheels at the same speed but in opposite directions. The center of this rotation is in the middle of the middle point separating the wheels instead of in the stopped wheel. `XXX` is the angle to turn in degrees.

6. `4XXXX`: Go straight until the robot traverses `XXXX` millimeters.

### Figures

We are attaching some figures displaying the plots of both the incoming distance signals as well as their auto-correlation.
