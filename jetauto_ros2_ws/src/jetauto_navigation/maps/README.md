# Maps

Save maps here (or anywhere) after mapping with `jetauto_slam`:

```bash
ros2 launch jetauto_slam slam.launch.py use_rviz:=true   # drive around to map
ros2 run nav2_map_server map_saver_cli -f ~/jetauto_map   # writes jetauto_map.yaml + .pgm
```

Then navigate with that map:

```bash
ros2 launch jetauto_navigation navigation.launch.py map:=$HOME/jetauto_map.yaml use_rviz:=true
```
