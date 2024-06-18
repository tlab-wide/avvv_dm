# avvv_dm

## Build

Clone this repository:

```
git clone git@github.com:hoosh-ir/avvv_dm.git
```

### Visualiser

Build the geographic library. From the root directory of the repository run:

```
cmake -S geographiclib-2.3 -B geographiclib-2.3/build
make -C geographiclib-2.3/build -j4
```

Navigate to the `visualiser` directory:

```
cd visualiser
```

Resolve ROS2 dependencies using `rosdep`:

```
source /opt/ros/humble/setup.bash
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

Build the whole workspace using `colcon`. While still in the visualiser directory run:

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Analyser

Navigate to the `analyser` directory:

```
cd analyser
```

Install the dependencies using pip:

```
pip3 install -r requirements.txt
```

### GUI (Not Working Yet)

Install Qt6 libraries and dependencies:

```
sudo apt install qt6-tools-dev qt6-wayland
```

Navigate to the `ui` directory:

```
cd ui/visually_launcher
```

Build the application:

```
cmake -S . -B build
make -C build -j4
```
## Troubleshoot

- In case you run into a SetupToolsDeprecationWarning issue when building any of the ROS packages, you need to downgrade you `setuptools` Python package using:
```
pip3 install setuptools==58.2.0
```