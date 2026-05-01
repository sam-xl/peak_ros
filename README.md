[![docker_ci](https://github.com/sam-xl/peak_ros/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/sam-xl/peak_ros/actions/workflows/industrial_ci_action.yml)

# peak_ros
ROS2 driver for use with PEAK MicroPulse devices.

## Installation

### ROS Workspace

```bash
cd ros2_ws/src
git clone https://github.com/sam-xl/peak_ros.git
rosdep install --from-paths peak_ros --ignore-src
cd ..
colcon build
```

## Usage

```bash
ros2 launch peak_ros capture.launch
```

Call either of the services `/peak/take_single_measurement` or `/peak/stream_data`.

```bash
ros2 service call /peak/take_single_measurement peak_ros/srv/TakeSingleMeasurement "take_single_measurement: true"
```

...or...

```bash
ros2 service call /peak/stream_data peak_ros/srv/StreamData "stream_data: true"
```

After this RViz ought to show the current b scan as a pointcloud on `/peak/b_scan`.

![](assets/b_scan.png)

If you set appropriate gate parameters in the [config file](src/peak_ros/config/default.yaml) you ought to get a gated b scan as another pointcloud on `/peak/gated_b_scan`, which will depict the front wall, if enabled, and the backwall or any defects.

![](assets/gated_b_scan.png)


### Reconstructions in A Fixed External Frame with TF
The [reconstruction configuration parameters](https://github.com/sam-xl/peak_ros/blob/jazzy/peak_ros/config/default.yaml#L47-L50) allow for live reconstruction of the gated ultrasound observations in an external fixed frame.

![](assets/live_recon.png)

Furthermore the [export_pcd launch argument](https://github.com/sam-xl/peak_ros/blob/jazzy/peak_ros/launch/init.launch#L14) allows for the export of the reconstruction to a PCD file for further processing and visualisation in other tools such as [CloudCompare](https://www.danielgm.net/cc/).

![](assets/pcd.png)


### Using Custom MPS files
Currently this driver requires MPS files to contain the following directives in order to correctly parse data:

1. DOF - Needed to determine the resolution and size of the returned data (currently only 1 and 4 are supported)
2. GATS - Needed to determine the length of each a scan
3. SWP - Needed to determine the number of focal laws (Note, that only a single sweep is currently supported)

Please ensure that your MPS file contains only one of each of these directives and that it accurately reflects what you are trying to achieve. MPS files ought to be placed in the `src/peak_ros/mps` and then modify the [launch file](src/peak_ros/launch/init.launch) `mps_file` launch argument. Alternatively, the `mps_path` launch argument can be used to load MPS files from an absolute path.

### Streaming Rates
Streaming rates can be set in the [config file](src/peak_ros/config/default.yaml). However, please be aware that your mps file will dictate the upper bound for your streaming rate as the number of focal laws and listening time for each focal law, GATS command, dictates the acquisition time needed by the Peak hardware.

### Dynamic parameters
Some parameters can be set dynamically through the ROS2 parameter interface.
Most importantly, this allows auto-gain to be implemented. These are:

* `ltpa.gate.start` (int)\
Start of the inspection gate (values that are actually transmitted), in machine units.
* `ltpa.gate.end` (int)\
End of the inspection gate, in machine units.
* `ltpa.delay` (int)\
Wedge delay, which offsets all timing measurements, in machine units. A start gate of 0 will therefore transmit data only starting from this delay.
* `ltpa.gain` (int)\
Gain, in 0.25dB units.
* `ltpa.prf` (int)\
Maximum pulse repetition frequency.

## Bugs and Feature Requests
Please report bugs and request features using the [Issue Tracker](https://github.com/sam-xl/peak_ros/issues).

# Authors
The driver and ROS node were written by [Matthew Shields](https://github.com/MShields1986).
The port to ROS 2 and further enhancements were written by [SAM XL](https://github.com/sam-xl).

## Funding
The authors acknowledge the support of:
- RCSRF1920/10/32; Spirit Aerosystems/Royal Academy of Engineering Research Chair “In-process Non-destructive Testing for Aerospace Structures”
- Research Centre for Non-Destructive Evaluation Technology Transfer Project Scheme
