# UAH Calibration for Differential Robots

We have written a Python script that automatically calibrates the needed data to compute the position of the robot through odometry.

This software has been written as part of the **Padrino Tecnológico** initiative from the *University of Alcalá, Spain*.

The theory behind the methods we propose can be found over at [**Hindawi's Journal of Sensors**](https://www.hindawi.com/journals/js/2019/8269256/?utm_medium=author&utm_source=Hindawi).

## Dependencies

In order to use the program you'll need to install several dependencies. It may vary from system to system so we encourage you to read through any dependency-related issue that may arise to solve it. Either way, with what we'll install here you should be good to go.

Please note that as we need to access serial ports living under `/dev` we'll need to run the program with super-user privileges (i.e prepending `sudo`). One could argue that we might add the user who's going to run the script to the group of the appropriate serial-port file and that would indeed be a more elegant solution. In case you wan't to look into it run `ls -l /dev` to find the group of the specific file and then execute `sudo usermod -a -G <file's group> <username>`. Please check that the group has **R/W** permissions on said file as we need to both read **AND** write data to the port.

For a "quick and dirty" solution just run the script with super-user privileges by executing `sudo python3 <Script_name.py>`. This has implications on the way we install dependencies. We will go ahead and use Python's package manager `pip3`. Please bear in mind that this script has been written with **Python3** in mind so be careful when using tools such as `pip` as they may be running `pip2` or `pip3` under the hood depending on your system configuration... If we are following the "quick" route we will need to execute `sudo pip3 install <module_name>` when installing packages. Just a note, if `pip3` isn't installed run `sudo apt install python3-pip` to get it. If on the other hand we are configuring the appropriate permissions for the device file we'll be okay just running `pip3 install <module_name>`.

The modules you'll likely need to install are:

* **pyserial**: `sudo pip3 install pyserial`
* **numpy**: `sudo pip3 install numpy`
* **matplotlib**: `sudo pip3 install matplotlib`

We are now ready to discuss how to operate the program and get that calibration done!

## Quick Start Guide

Once all the dependencies are installed place yourself in the folder containg the file `Calibration_script_final.py` and run `sudo pyhton3 Calibration_script_final.py`. In case you decided to configure file permissions appropriately you can just run `pyhton3 Calibration_script_final.py`. Th program will autodetect the Arduino board and it'll also guide you throughout the entire process so there should be any problem.

In case you want to see the signal vectors we build to get the required data you need to manually uncomment the calls to `show_signal()` within `main()` you will find on lines `376`, `378`, `382`, `400`, `402`, `406`, `424`, `426` and `430`. Note that these calls are blocking in the sense that you must close the windows manually to resume operation.

Whenerver you are presented with a choice you will nee to use an **UPPER CASE** letter to accept, just like in `apt` as a negative answer is usually "safer". This has been our design choice.

The program may generate a number of `.txt` file if not instructed to delete them. We used them to check whether the way we treated our data was coherent with a Matlab version whose operation had been tested. You can delete them without worries if the program doesn't automatically delete them.

We have included some system calls relying on UNIX commands so if you inted to run the script on a Windows platform you'll need to adjust these... The commands are given as a parameter to `os.system()` so that you can find them more easily.

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

### Used libraries <a name=Imports></a>

In order not to "reinvent the wheel" over and over again we have taken advantage of several libraries to make the code faster and more legible. The functionalities they contribute to our program are:

1. `glob`: This library is used to expand paths by feeding regular expressions into it as well as wildcard characters such as **&ast;** in Unix systems, for example. We use it to expand the paths to every possible device under the `/dev` directory in order to try and find the port the MCU (Micro-Controller Unit) has been assigned.

2. `serial`: Even though its name can be misleading, this is in fact the [*PySerial*](https://pythonhosted.org/pyserial/) API for handling serial communication. Please refer to the documentation to go through any chunk of code using this library.

3. `os`: This library lets us perform system calls from the program, such as executing commands and have them interpreted by *BASH*. We also use it to delete unused files within the program. We know this is less portable than calling Python wrappers who then fall back to system specific procedures, but we found this approach to be faster as we are guaranteed to be working in a &ast;nix system. This way we didn't get bogged down in documentation (even more!).

4. `time`: As we have to interface with the MCU's firmware we have to make some delays throughout the code. I believe it to be a horrible programming practice, but as the MCU cleans the input serial buffer continuously we had no other choice but to wait to send some commands...

5. `signal`: As discussed in the *Misc functions* section, we have decided to attach a handler to keyboard interrupts to handle them more gracefully. This is done by means of this library.

6. `numpy`: We have only included the `convolve()` fucntion from this library. We have to use it to take the auto-correlation of the input signals and let's be honest, our own implementation of this function would have been much slower if it were t have worked in the first place ;).

7. `re`: This library handles everything related to regular expressions. We use them to find numbers within the strings we get back from the MCU when asking for data, so that's why we have only used the `findall()` method.

8. `math`: As we are always dealing with circumferences of different radii we inevitably had to use PI. Instead of just defining it to be something like `3,14159...` we decided to go with the value provided, as it is way more exact. We also used logarithms in base 10 in order to compute the number of digits a given number (in base 10) had. We use it when "crafting" some commands so that we adhere to the syntax requirements. You can read more about it in the `initial_data()` section down below.

9. `matplotlib`: We only imported [*Pyplot*](https://matplotlib.org/api/pyplot_summary.html) from this behemoth of a library so that we can represent lists as graphs along the execution of the program. This way the user can verify the integrity of the data as well as the profiles of the graphs to ensure everything is going as it should be. We make use of its methods in the `show_signal()` function. Please note that pyplot is a module (collection of related functions), not a standalone function.

With all the imports out of the way lets get to know our program more in depth.

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

15. `STOP_CHAR`: This is the character sent by the robot when in an idle state. We use it to determine when the execution of a command finishes so that we can time data retrieval and processing accordingly. When we need the length of saif stop character we have used the `len()` so that changing this string "just works".

16. `ARDUINO_ID_STRING`: In order to detect whether the plugged in serial device is an Arduino board or not we rely on a file created under `/dev/serial/by-id` when we open the corresponding serial port. This ID is board dependent so you'll need to adjust it based on your own board. It's been updated to work with the current platform as of 11/12/19. To find your ID just run `ls /dev/serial/by-id`. This behavior has been implemented to allow for an easier automatic detection in the presence of serveral serial devices but has not been thoriughly tested so in case you experience any errors just substitute the call to `check_if_arduino()` on line `86` to `True` and change the `path` on line `85` by a hardcoded string containing the path to the serial device file like `/dev/ttyUSB0` for instance.

#### Commands

These constants contain the commands we will send to the MCU. The ones we find defined have been computed taking our platform's defaults into account.

They can be found within the `COMMANDS` dictionary. As with the `CONSTANTS` one, the indices are the commands below given as strings.

1. `GET_DATA = 'F'`

2. `TURN_L_WHEEL_STOPPED = 'C0000371'`

3. `TURN_R_WHEEL_STOPPED = 'D0000371'`

4. `TURN_BOTH_WHEELS = '2360'`

5. `GO_STRAIGHT_3M = '43000'`

6. `GO_STRAIGHT_1M = '41000'` (used for debugging)

7. `TURN_L_LITTLE = '2020'` (used for debugging)

8. `TURN_R_LITTLE = '3020'` (used for debugging)

9. `GET_STRAIGHT_DATA = 'Q'`

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

### Function breakdown

#### Interfacing section

The main aim of this module is opening the serial port for Arduino and getting the initial data we need as input for the calibration. We will also send the different commands to trigger the needed movements of the robot. We will finally recover the data and store it in files to be processed later on.

1. `initial_data()`: We need the estimated measures of the robot as a starting point for the calibration, namely the nominal diameter (the one the wheels are supposed to have) and the wheelbase. We have also included a handful of other input data in an attempt to make the program as general as possible. These include the number of pulses the motor's encoder emits per revolution, the reducing factor between the motor and the wheels if any and the Arduino board's baud-rate. Whilst getting the data, the function updates a series of global variables containing the basic data of the robot and computes any needed secondary parameters. It will also generate the command we need to make the robot do a 360&deg; turn. All the commands, are explained in the [**Annex**](#Annex). We would like to draw attention to the part where we take the logarithm in base 10 of a number. We do so to find the number of digits in said number so that we can "craft" the command with the appropriate format. This function will also be in charge of configuring the behavior of the program based on the user's preferences. To accept a default value just press ENTER without typing anything.

2. `print_updated_data()`: This function just prints the global variables to the screen. It has been made mainly for debugging purposes, but it can be used to provide additional feedback to the end user. It ends up by prompting the user whether to continue or halt the process.

3. `find_N_open_serial_port()`: This function is the first one to use the PySerial library. It will try to open every serial port. As ports with nothing connected to them will raise a serial exception we just handle it by continuing the loop. When we open a serial port with something connected we try to check whether we have an Arduino board connected or not to the Raspberry Pi. To check all the possible ports we have used a RegExp (Regular Expresion) and then broken down the different paths with the function `glob()`. Due to the nature of serial devices it is kind of difficult to get information about the manufacturer. When dealing with a Linux-based system, attaching a serial device triggers the creation of a file in the `/dev/serial/by-id` folder. This file contains a fixed string that can be used to identify a device. By calling Linux's `ls` command on this file we can check whether an Arduino board is connected or not. The catch is that is we have several boards we cannot know for sure which port belongs to the Arduino board. When working with original Arduino boards we can just call the `dmesg` command and we will see a message containing the model of the board as well as the port where it's been attached. By parsing `dmesg`'s output we could know where our MCU is for sure. Another way of checking for a specific board would be by looking up the `VendorID` number from the `dmesg` command and query an online database for the manufacturer number and proceed just like with an original chip. We can see how this is quickly getting out of hand, so we have decided to stick with the first approach, as we are handling a very specific environment. In order to read the output of the `ls` command we call the `os` module and read its return value. We then only check whether the target string (that depends on the hardware we are connecting) is within `ls`'s output. If so, we open the serial port to our MCU. The return value of this function is the port we have opened to the Arduino.

4. `execute_command([string object] command, [port object] port, [string, optional] f_name)`: This function is just a wrapper that sends orders to the Arduino board. After sending the command, it will remain in an idle state until it reads two dots (`.`) from the serial port. This is due to the fact that the firmware starts sending dots when it has finished carrying out the requested action. These dots are our "stop condition" after all. Again, the commands are explained in the [**Annex**](#Annex). Please note how the `f_name` parameter is optional. It is used for specifying the filename of the file containing the dumped data from the MCU so that it can be further processed in programs such as MATLAB in case it's needed.

5. `get_data([port object] port)`: This functions triggers the output of data from the board and waits for the `END` string signaling that there is no more data to come. All this data will be stored on a text file for further processing. The file will be returned with the R/W pointer at the beginning.

6. `get_straight_data([port object] port)`: This function is in charge of getting the recorded pulses when carrying out the last test. We have decided to use another function because the string format we have to read and process is not the same than it was before. It is based on regular expressions, just like the function `extract_data()`. In order to know a little bit more about its inner workings please refer to the aforementioned function down below.

##### Small firmware tweaks

We not only need to send orders to the Arduino board, but we also need to update the value of some parameters! As this hadn't been designed we had to get our hands dirty and write two new functions for the MCU's software. These are:

1. `parse_input()`: Once called, the function will start to read the input from the serial port byte by byte. If the byte is a digit it will store it in a C-string (character array) and if it is not it will call a function called `update_param()`. After calling said function we reset the number array with `NULL`s. This approach let's us be flexible and accept numbers ranging from 1 to 4 digits. The specifics of the command string will be discussed in the [**Annex**](#Annex).

2. `update_param(char* number, char parameter)`: Once called, we will update a parameter depending on the input argument parameter, which is just a `char`. The new value is the `float` equivalent of the number string `number`.

We decided to include them here due to their strong relation with the interfacing section.

#### Computations section

In this section we will see how we go from having RAW text input to sanitizing it and extracting a calibrated value for each of the needed parameters. As before, we have broken down the task into several functions:

1. `read_N_parse([file object] data, [python list] p_array, [python_list] d_array)`: The input data to this section is contained in a file. The data we get from the Arduino contains the number of encoder pulses, a space, the distance measured by the sensors in centimeters, a space, and a semicolon followed by 3 dots. It looks something like: `XXXX YYYY ;...`, where `XXXX` and `YYYY` are the pulses and distances respectively. As the number of digits may vary we found it cumbersome to check each line byte by byte, so we took advantage of the power of RegExps. This function will read the input file data line by line whilst making sure "rubbish" lines are not taken into account. The data will be extracted by our next function, `extract_data()`, which we will call for every line in the document.

2. `extract_data([python_list] string_to_parse, [python_list] p_array, [python_list] d_array)`: Using the function `findall()` from the `re` library we are able to extract the data we need and begin our process. The RegExp we are using matches ASCII digits (remember RegExps only work on text) so we need to convert them to integer values before appending them to their corresponding arrays. One might argue that we could condense these 2 first functions int one. Whilst that is absolutely true we believe this approach is more modular and improves overall code readability and clarity, something we prefer to prioritize.

3. `populate_signal([python_list] pulses_array, [python_list] distances_array)`: Before understanding the way this function operates, we should take a closer look at the nature of the signals handed to us as input for this module.
<br><br>
First of all consider that in our case the encoders will emit 152,7 pulses per wheel revolution. Given that our tests were carried out by performing 8 turns and that a turn implies a fair number of wheel revolutions we are looking at a considerable quantity of pulses. Due to design constraints of the firmware, these pulses are tracked with an unsigned char whose values lies in the range [0, 255]. Let's face it, overflow is going to happen... This condition is checked by looking at the current and past number of pulses. As the number of pulses is monotonously increasing, we know that if a future index is larger than a past an overflow has taken place! In order to keep track of these overflows we have defined a variable `n_jumps`. We will add 256 to our current index for each overflow we have detected, so that the number of pulses we are using as index in our signal is in agreement with what it should be if we didn't have to cope with the char data type size constraints.
<br><br>
Due to firmware limitations we will see that we don't have a distance measurement for each encoder pulse. We will instead have measurements for certain pulse quantities, and no information for pulses in between said quantities. We chose to hold the last registered values for these "dark" intervals, so that's why we have a while loop within the for loop, we are just assuming the distance didn't change too much so that we can say it's roughly the same.
<br><br>
when appending distance information to the array we are building we can see a conditional check that looks at the `NOISE_THRESHOLD` constants and filters out longer (with a higher value) pulses.
<br><br>
After finishing, this function returns the real distance signal we need to process to find our parameters. We are almost there!

4. `convolution_time([python_list] original_signal, [python_list] reversed_signal)`: This function returns the convolution of the inputs. As discussed in the linked article at the beginning of this document, the auto-correlation of a signal is equivalent to the convolution of the signal with itself but reversed. In math terms: X[n] &ast; X[-n] (j) = R(j), where R(j) is the autocorrelation of X[n] as a function of j and X[n] &ast; X[-n] (j) is the convolution of X[n] with itself reversed as a function of j too. Again, one could argue that functions **7** and **6** could have been joined into only one. We have taken this approach for the sake of readability.

5. `find_maximum([python_list] convoluted_signal)`: This function is more a signal processing problem than a programming one. As seen in the linked article, the profile of the auto-correlation will show clearly identifiable peaks where an entire turn has been detected. As we are interested in the number of pulses for this to happen we will be looking for the index where we find a maximum value.
<br><br>
The catch here is that we won't be looking at the entire signal, but only at a portion of it between two limits we define: `lower_limit` and `upper_limit` in the code (no surprise!). As math assures us there will be a maximum between these we just have to look for this maximum and record the index where it takes place, which we do with a normal `for` loop. The function will return the value where we have this maximum, which directly depends on the index we found above.
<br><br>
As the index of the correlation accounts for "how much" we have to displace the signal in order to record a maximum, we are interested in said displacement. As the length of the convoluted_signal signal is `2 · L - 1`, where `L` is the length of the original signal, we know that we will have the absolute maximum at index `(2 · L - 1) / 2` which can be seen either as `L` or `L - 1` due to the `- 1 / 2` factor we get from the expression.
<br><br>
Let's say we found a peak at index `j`, being `j < L`. Then, the separation between theses values is given by `L - j` and, as previously seen, this is exactly the period we are looking for: `P = L - j`. This holds true if the peak at `j` is the one closest to the absolute maximum, either to its left or to its right. We would otherwise have to normalize by the number of encountered peaks, as each peak we found states that we have displaced ourselves another full period. Then, the more general expression would end up being: `P = (L - j_n) / n`, assuming `j_n < L`. We would otherwise have `P = (j_n - L) / n` if `j_n > L`.

#### Misc functions

These functions don't precisely fit into any of the two categories above, so we have decided to include all of them here. Their purpose goes from handling program flow to showing data along the way.

1. `IRQ_setup()`: During testing we halted the program through a keyboard interrupt (**CTRL + C**) more often than not, we we decided to treat this interrupt and close the program gracefully. We have used the `signal` library for this purpose. The configuration of the interrupt is done is this function, where we attach the handler `handler()` to the interrupt `signal.SIGINT`, which is the one the keyboard interrupt raises.

2. `handler()`: This function will trigger on the reception of a keyboard interrupt and just exit the program after printing that the script is about to be terminated.

3. `user_tweaks([string] mode)`: This function is in charge of handling the program flow. The function is called after any important section and it clears the screen whilst waiting for user input to continue. This is a way of letting the user have a tighter control on the program and not the other way around whilst clarifying the operation off the script altogether. The mode parameter has been left to enable additional functionality if required in the future.

4. `show_signal([list] input_signal, [string] y_label, [string] title)`: This function is in charge of plotting any `input_signal` we feed it. The `y_label` and `title` string set those elements within the graph to give it a little bit more context. The back-end we use for plotting is `matplotlib`, even though we are not even scratching its power in our implementation.

#### Program structure, the main() function

Even though it's not strictly necessary, as we have a strong background in **C** we have decided to wrap the program in a `main()` function and just call it upon execution. By checking that `__name__ == "__main__"` we are taking advantage of the environment Python sets up for us. If we are executing the script directly the variable `__name__` will indeed be equal to `__main__` but if it is not executed directly the above won't hold. This lets anybody import our module in case they want to use if for any further project.

Upon entering the function we will define the necessary lists for operation as well as noting that we are going to use some `GOLBAL` variables. The lists we have defined are:

1. `p_array`: It will be the array holding information regarding pulses read from  the Arduino board.

2. `d_array`: It is exactly the same than the one before, but it will contain the distances read by the ultrasound sensors.

3. `X_PULSES`: These lists will be filled with the computed pulses in each of the tests. As we have all of them together it'll be easy to take the average of all of them when needed.

After printing information regarding the program structure we will open the serial port to the MCU and ask for the initial data. The user will be able to accept the defaults here without a problem.

We will then carry out each of the three tests that involve turning whilst asking for the necessary input along the way. We should note that in order to copy one list into another one cannot simply say `list_b = list_a`! When you do the above you are just creating a "list pointer" that points to the first list, `list_a` in this case. It took us a while to spot this one out...

The rest of the script is a little bit easier to follow, but if you have any doubts feel free to reach me at: pcolladosoto@gmail.com.

## Calibration procedure

After digging into the code we are now capable of talking about the procedure one has to follow to calibrate the differential robot.

The first requirement is to find an open space so that the robot is around `2 m` away from any obstacle. As the reach of the ultrasound sensors is of about `1,5 m` the firmware has been prepared to treat any measurement above said sensitivity as a `0`. Thus, the robot will "think" it has a distance of `0` all around. What we have to do now is place a given obstacle (I used a cardboard box) about `20 cm` away from the robots ultrasound sensors. In order to get the best possible results it is a good idea to make the obstacle be as parallel as possible to the plan where the ultrasound sensor lies so that we get the best possible results. Otherwise, the reflection of the emitted pulse may not reach the sensor in its entirety muddying our results...

What we achieve with the above is a perfectly periodic scenario, where we will see `0`s all around except when we are facing the obstacle, where we will find a sudden peak. As seen in the screenshots above this produces a crisp graph profile that's easier to handle.

In between tests you may have to reposition the robot and readjust the obstacle so that it is parallel to the ultrasound sensor. Be careful not to set up a periodic scenario. This could be done by placing two similar obstacles `180º` away from each other. Then for `1` "real" lap the robot would think it has effectively made `2`!

If the wheels begin slipping and the robot drifts consider looking for another surface or increasing the friction between both surfaces with tape or by putting weight on top of the slippery wheels.

For the straight test you will have to measure the distance the real distance the robot has traversed. In order to do so you need a easuring taper of at least `3,5` m, as the robot will move around `3`m. The measure you have to input to the program **MUST** be in mm, so be extra careful or the calibration will be for nothing...

These are some practical tips that complement the ones found in the article we referenced at the start of the document. Happy calibrating! :)

## Testing procedure

In order to test the newly calibrated robot we decided to go for what we call the 3-&pi;-3 procedure. As the name implies, what we are doing is ordering the robot to advance 3 meters, make a 180 degree turn (&pi; radians) and then traverse another 3 meters. In a wonderfully perfect world we would see how the position of the robot would be the same.

Actually, saying the position of the robot will be the same is kind of vague... We need to define a suitable reference! In order to do so, we taped a thin piece of paper to the back of the robot so that it touched the floor. Now, the robot's firmware has been written so that the reference of the robot is the middle point of its wheelbase. Remember that the wheelbase is the distance between both wheel's point of contact with the ground. Measuring our specific platform shows that the distance from said center to the point where we taped our piece of paper is 30 mm.

Taking into account that the **center** of the wheelbase should be the one ending up in the initial position it had, we can see how we need to get some math out of the way to get the exact deviations. As we will be throwing the word *reference* quite often from now on it would be a good idea to get some nomenclature out of the way. We'll refer to the paper tape as the measurement reference, as it is the one we used in our tests and to the center of the wheelbase as the robot's reference. Note how the following computations take the measurements reference as their reference (see how we were going to say reference a lot? :laughing:)

We define the robot's initial reference as R_o and it has an initial value of (30, 0). After the robot has carried out the 3-&pi;-3 test the tape will have a position T_f = (X, Y). Now, the center will be 30 mm closer to the origin due to the 30 mm separation between the tape and the robot's center, so the final position of the robot's reference will be R_f = (X - 30, Y). Note that, as both references lie on the same line we don't have to "adjust" the Y component. Then, the displacement of the robot's reference becomes R_f - R_o = (X - 60, Y).

Finally, take into account that if the orientation of the robot is not 0 or &pi; radians we will have to take into account the heading's deviation to compute the error we have in both components... That could get kind of tricky, but as the measurements are quite small we can get away with being a little bit lazy :grin:

## Test results

We will attach the raw results we obtained when carrying out the tests to verify the calibration. We will then take into account the correction factor for the `X` coordinate so that the results are comparable to the ones shown in the cited article. We carried out 5 tests per configuration because we didn't observe a lot of variability. These 5 tests give a good enough sample then. We are attaching said data below.

Please note that the parameter values we have used for the procedure are given in the table below:

|                         | KKI (ED) | MM -> Pulses (mmperpulse) | Wheelbase (WHEELDIST) [mm] |
| ----------------------- | -------- | ------------------------- | -------------------------- |
|   Calibrated Firmware   |  0.9985  |         1.67941           |         540.183            |
| Non-calibrated Firmware |    1     |          1.68             |           535              |

These were obtained after carrying out the procedure described in the referenced article using our own script in the process.


### Raw data with a calibrated Firmware

|        | Real X | Real Y | Internal X | Internal Y |
| ------ | ------ | ------ | ---------- | ---------- |
| Test A |  150   |  150   |     60     |    121     |
| Test B |  220   |  475   |     84     |    376     |
| Test C |  144   |  334   |     13     |    247     |
| Test D |  180   |  645   |     40     |    653     |
| Test E |  180   |  580   |     40     |    542     |

### Raw data with a non-calibrated Firmware

|        | Real X | Real Y | Internal X | Internal Y |
| ------ | ------ | ------ | ---------- | ---------- |
| Test A |  145   |  390   |     37     |    455     |
| Test B |  155   |  420   |     39     |    515     |
| Test C |  119   |  335   |     18     |    406     |
| Test D |  105   |  234   |     -8     |    348     |
| Test E |  115   |  295   |     -3     |    353     |

### "Processing" the data

As discussed above we need to "treat" the data by correcting the `X` coordinate. We have taken this approach just in case the correction has a misconception... This still shows the measured data so that it can be treated correctly in that case. We have also computed the error between the real measured data and the one the robot believes it has internally. We have opted to compute this error as `Error_N = Internal_N - Real_N`. Said data is given below.

#### Processed data with a calibrated Firmware

|        | Real X | Real Y | Internal X | Internal Y | X Error | Y Error |
| ------ | ------ | ------ | ---------- | ---------- | ------- | ------- |
| Test A |  90    |  150   |     60     |    121     |  -30    |  -29    |
| Test B |  160   |  475   |     84     |    376     |  -76    |  -99    |
| Test C |  84    |  334   |     13     |    247     |  -71    |  -87    |
| Test D |  120   |  645   |     40     |    653     |  -80    |   8     |
| Test E |  120   |  580   |     40     |    542     |  -80    |  -38    |

#### Processed data with a non-calibrated Firmware

|        | Real X | Real Y | Internal X | Internal Y | X Error | Y Error |
| ------ | ------ | ------ | ---------- | ---------- | ------- | ------- |
| Test A |  85    |  390   |     37     |    455     |  -48    |  65     |
| Test B |  95    |  420   |     39     |    515     |  -56    |  95     |
| Test C |  59    |  335   |     18     |    406     |  -41    |  71     |
| Test D |  45    |  234   |     -8     |    348     |  -53    |  114    |
| Test E |  55    |  295   |     -3     |    353     |  -58    |  58     |

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

5. `2XXX`: Turn with both wheels at the same speed but in opposite directions. The center of this rotation is in the middle of the imaginary line joining both wheels instead of in the stopped wheel. `XXX` is the angle to turn in degrees.

6. `4XXXX`: Go straight until the robot traverses `XXXX` millimeters.

### Figures

We are attaching some figures displaying the plots of both the incoming distance signals as well as their auto-correlation and the signals we "populate". The test we carried out made the robot do **5** laps, so you'll find 5 peaks in the distance signals and 5 peaks on each side of the convoluted one. The nomenclature we have used is:

* &ast; RAW Distances: The ones we read from the ultrasound sensors directly, we haven't treated the signal yet...
* &ast; Populated Signal: After calling `populate_signal()` we end up with this graph. It's ready to be convoluted.
* &ast; Convolution: The auto-correlation of each given signal.

The &ast; is our way of saying that these three graphs have been plotted for each test:

* Turning with the left wheel stopped
* Turning with the right wheel stopped
* Turning with both wheels in opposite directions

And finally, the screenshots are (in the order above):

#### Left wheel stopped

<p align="center">
  <img src="./Graphs/Test_j/L_RAW.png">
</p>

<p align="center">
  <img src="./Graphs/Test_j/L_baked.png">
</p>

<p align="center">
  <img src="./Graphs/Test_j/L_conv.png">
</p>

<!--
![L RAW Distance](./Graphs/Test_j/L_RAW.png)
![L Baked](./Graphs/Test_j/L_baked.png)
![L Convoluted](./Graphs/Test_j/L_conv.png)
-->

#### Right wheel stopped

<p align="center">
  <img src="./Graphs/Test_j/R_RAW.png">
</p>

<p align="center">
  <img src="./Graphs/Test_j/R_baked.png">
</p>

<p align="center">
  <img src="./Graphs/Test_j/R_conv.png">
</p>

<!--
![R RAW Distance](./Graphs/Test_j/R_RAW.png)
![R Baked](./Graphs/Test_j/R_baked.png)
![R Convoluted](./Graphs/Test_j/R_conv.png)
-->

#### Both wheels in opposite directions

<p align="center">
  <img src="./Graphs/Test_j/Both_RAW.png">
</p>

<p align="center">
  <img src="./Graphs/Test_j/Both_baked.png">
</p>

<p align="center">
  <img src="./Graphs/Test_j/Both_conv.png">
</p>

<!--
![Both RAW Distance](./Graphs/Test_j/Both_RAW.png)
![Both Baked](./Graphs/Test_j/Both_baked.png)
![Both Convoluted](./Graphs/Test_j/Both_conv.png)
-->