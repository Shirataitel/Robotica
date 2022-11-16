Shira Taitelbaum 322207341
# Creating New Krembot Program

To create a new program, you'll need to create at least one new controller, and a configuration file.


From this point, for these instructions, we will assume your workspace is in $HOME/krembot_ws
The controller and configuration files are stored in a dedicated directories inside the directory:

- controllers - contain controllers
- config - contain configuration files


## Controller Programs

This plugin was designed to be as close as possible to working on real hardware both in terms of simulation and code architecture. Coming from real Krembot hardware, you probably know that .ino file is used as a "main" file. In Argos the main file is called "controller". This plugin hides most of the controller boiler plate code, and makes the controller file look very similar to an ordinary .ino file. 
Go into ````$HOME/krembot_ws/controllers/```` and check out the example controllers.

### Creating New Controller

1. Go into the controllers folder in your workspace
2. Inside that directory, you will find ````program_template```` directory, that contains a template code files for krembot program.
This folder serves as an example. **Copy it and rename to match your new program**.
3. Edit [CMakeLists.txt](CMakeLists.txt) inside that directory, and add your directory name to the list with ````add_subdirectory```` command. Save and close.
4. Go inside the new directory. Edit [CMakeLists.txt](CMakeLists.txt) and change the macro on line 4 ("set(PROGRAM_NAME ... ) "). Replace "program_template" with the name you gave this directory. If your program contain more source (.cpp) files than just the main .ino file, this is also the place to list them - go head and write those files under line 10 ("set(SRC_FILES .... )"). Save and close. 
5. Edit the krembot.ino.cpp file. This will be your main file (.ino). Add your code to this file. The places where you should put your code in are marked with "todo:" tags. Add source files as needed.

### Compiling the Controller

We are ready to compile your code. Go into ````build````, and  ````make```` in the terminal.
Fix any compilation errors as needed until the code compiles cleanly. You are now ready to run
your code in simulation.


## Configuration Files

The Argos simulator requires the user to configure the simulation environment. This is done inside a ````.argos```` configuration file. Some configuration parameters are mandatory (e.g. what controllers should the simulator load, and where they are located), and some are not (e.g. the names of the robots). Go inside ````$HOME/krembot_ws/config/```` and check out the example configuration files.

Note that a specific parameter "library" inside the config files must point to the compiled binary of the program you create. 
Below, we explain how to modify this parameter to run your first example program.

The simulator execution command includes the configuration file to use. This enables the simulator to load a specific set of user configuration.

### Creating New Argos Configuration File

1. cd into your workspace config folder
2. Create a copy of the [program_template.argos](program_template.argos) file, and give it the new name you gave your program inside your controller's [CMakeLists.txt](CMakeLists.txt) file.
3. Replace "program_template" everywhere in the file with your new program name.
4. The parameter "library" (see, e.g., line 53) provides full path to your compiled controller (.so file). This file can be found in ````build/your_program```` directory after building your code.
5. Further changes in this file can be made to configure simulation. The file is well-documented, so it should be relatively easy. More documentation can be found at [the ARGoS simulation documentation](https://www.argos-sim.info/documentation.php)

## Running Your Controller

In order to run argos simulation, your should use the argos3 program, and give it the correct configuration file using the "-c" flag, which accepts a name (including path, if necessary) to your new program's .argos configuration file. Before you use this command, make sure you sourced the setup_env.sh file (see instructions above). 

````
argos3 -c path/to/your/argos/file.argos
````

In case you see an error telling you that your program .so file could not be found (it will complain of libYOUR_PROGRAM.so not being found, where "YOUR_PROGRAM" is the new program name. Make sure the path you provided to the .so file inside the configuration file is correct. If it is, then read the error carefully. Argos runs your controller by dynamically linking to it. Code linking errors are hiding in these error message, and you might miss them at first glance.
