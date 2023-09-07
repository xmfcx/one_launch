# autoware_vehicle

## Contents

This package contains:
- Sensor calibration data
  - [sensors_calibration.yaml](config/sensors_calibration.yaml)
  - [sensor_kit_calibration.yaml](config/sensor_kit_calibration.yaml)
- Vehicle dimensions
  - [vehicle_info.param.yaml](config/vehicle_info.param.yaml)
  - [mirror.param.yaml](config/mirror.param.yaml)
- Planning simulator configuration
  - [simulator_model.param.yaml](config/simulator_model.param.yaml)
- Vehicle model mesh
  - [lexus.dae](mesh/lexus.dae)
- URDF files
  - [main.xacro](urdf/main.xacro)
    - [sensors.xacro](urdf/sensors.xacro)
      - [sensor_kit.xacro](urdf/sensor_kit.xacro)
    - [vehicle.xacro](urdf/vehicle.xacro)

## Function

The launch file runs:
- `robot_state_publisher`
- `vehicle_info`

### `robot_state_publisher`

Publishes the TF tree for the vehicle and sensor kit.

Also publishes the `/robot_description` which enables `rviz2` to visualize the vehicle and sensor models.

### `vehicle_info`

Publishes the vehicle dimensions.
