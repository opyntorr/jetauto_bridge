# ORADAR MS200 LiDAR setup for ROS1

Clone the following repository

```bash
git clone git@github.com:lehoangan2906/Oradar-ms200-setup-file.git oradar_ros
```

## Install Driver

in the `oradar_ros` folder we’ve just cloned, open a terminal and input the following command:

```bash
$ cd sdk
$ mkdir build
$ cd build
$ cmake ..
$ make -j4
$ sudo make install 
```

If the system does not prompt any errors, the driver is successfully installed.

## Bind LiDAR port name

Open the terminal under the `oradar_ros` package

Run the following command:

```bash
dmesg | tail # run before plugging in the lidar
```

And afterward, plug in the LiDAR and re-run the above command.

Observe for newly added device. Depends on the device name “ttyACM*” or “ttyUSB*”, we’ll need to modify the `oradar.rules` file accordingly.

```
KERNEL=="ttyUSB*",ATTRS{idVendor}=="10c4",ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="oradar"
# oradar.rules file config for ttyUSB*
```

```
KERNEL=="ttyACM*",ATTRS{idVendor}=="1a86",ATTRS{idProduct}=="55d4", MODE:="0777", SYMLINK+="oradar"

```

After completing the above steps, copy the `oradar.rules` file and bind it to the `/etc/udev/rules.d/` file.

```bash
$ sudo cp oradar.rules /etc/udev/rules.d
```

Then, unplug and re-plug the LiDAR and enter the follwing command into the terminal:

```bash
$ ll /dev/oradar
```

![Screenshot 2024-05-24 at 11.26.20 AM.png](ORADAR%20MS200%20LiDAR%20setup%20for%20ROS1%204298a08c30df476980c14f9d74691ca9/Screenshot_2024-05-24_at_11.26.20_AM.png)

The above content indicates that the binding is successful. The end is not necessarily 0 and changes according to the order in which the devices are inserted.

## Create a new workspce and compile function packages

Take creation name as `oradar_ws` as an example.

Input the following command:

```bash
$ mkdir -p oradar_ws/src
$ cd oradar_ws/src
$ catkin_init_workspace
```

Then, copy the `oradar_ros` folder to the `oradar_ws/src` directory.

Then, in the `oradar_ws` directory, use `catkin_make` to compile.

```bash
$ cd oradar_ws
$ catkin_make
```

After the compilation is passed, add the path of the workspace to `~/.bashrc`.

```bash
$ sudo vim ~/.bashrc
```

Copy the following content to the end of `.bashrc`

```bash
source ~/oradar_ws/devel/setup.bash --extend
```

Save and exit

```bash
:wq
```

Reopen a terminal, enter the following open the LiDAR and display its data on RVIZ:

```bash
$ roslaunch oradar_lidar ms200_scan_view.launch
```

![Screenshot 2024-05-24 at 11.34.41 AM.png](ORADAR%20MS200%20LiDAR%20setup%20for%20ROS1%204298a08c30df476980c14f9d74691ca9/Screenshot_2024-05-24_at_11.34.41_AM.png)