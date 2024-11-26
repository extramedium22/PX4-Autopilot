Date: 241126

If you are looking at the PX4 offical documentation, please go to file "README_original.md"

This branch is try to realize and replicate quadcopter FTC with one or more rotor failure.

For quick start, you can try the following code to start the simulation:
```shell
$ cd ./PX4-Autopilot/
$ make px4_sitl gz_x250_test
```
then you will start a gazebo simulation with vechicle x250.

The main controller design is in file ~/PX4-Autopilot/src/examples/uniftcctrl/uniftcctrl.cpp.
At present, the method to realize the rotor failure is realized by flipping the channel 6 switch of RC. In the sitl environment, you can use PX4's uorb message mechanism to publish the corresponding signal, or you can use the **motor_eff_ctrl** module to control the motor efficiency through the following code:
```shell
$ cd ./PX4-Autopilot/build/px4_sitl_default/bin
$ ./px4-motor_efficiency_publish 1 1 1 1
```
Of course, you need to pay attention to the corresponding code changes in the **uniftcctrl** module.

The repo also integrates the **quadcopterbsctrl** and **quadqmodelbsctrl** which are backstepping control methods based on vector and quaternion description, seperately. If you want these modules to work, remember that you need to modify the corresponding make target in ~/ px4-autopilot /boards/px4 (currently these modules conflict), You also need to change the ehicle boot option in ~/PX4-Autopilot/ROMFS/px4fum_common.

For more info, please refer to file "readme_240808.txt"
