# M3DGR Sensor Calibration

This document provides the extrinsic and intrinsic calibration parameters for the M3DGR acquisition platform. All extrinsic transforms are expressed as **translation** `T = [x, y, z]` (meters) and **rotation** `R` (3×3 row-major rotation matrix), defining the transform from the **source** frame to the **target** frame:

```
p_target = R · p_source + T
```

Coordinate conventions follow the sensor arrangement in the [README](readme.md#41-acquisition-platform): **X** (red), **Y** (green), **Z** (blue).

---

## Table of Contents

1. [Extrinsic Calibration](#1-extrinsic-calibration)
2. [Camera Intrinsics](#2-camera-intrinsics)
3. [IMU Noise Parameters](#3-imu-noise-parameters)

---

## 1. Extrinsic Calibration

### 1.1 Camera ↔ Camera IMU (D435i)

| Direction | T (m) | R |
|-----------|-------|---|
| `camera` → `camera_imu` | `[0.03668114, -0.00477653, 0.0316039]` | `[0.99957087, 0.00215313, 0.02921355, -0.00192891, 0.99996848, -0.00770122, -0.02922921, 0.00764156, 0.99954353]` |

### 1.2 Camera IMU ↔ Livox Avia

| Direction | T (m) | R |
|-----------|-------|---|
| `camera_imu` → `avia` | `[-0.19501, 0.0471106, 0.0870882]` | `[0.0502661, -0.0116117, 0.998668, -0.99829, 0.0292847, 0.0505876, -0.0298331, -0.999504, -0.0101199]` |
| `avia` → `camera_imu` | `[0.0594305, 0.083401, 0.193248]` | `[0.0502661, -0.99829, -0.0298331, -0.0116117, 0.0292847, -0.999504, 0.998668, 0.0505876, -0.0101199]` |

### 1.3 Camera ↔ Livox Avia

| Direction | T (m) | R |
|-----------|-------|---|
| `camera` → `avia` | `[-0.16154853, 0.01195102, 0.09044823]` | `[0.0210767, -0.0038717, 0.9997703, -0.9993969, 0.0275209, 0.0211754, -0.0275965, -0.9996137, -0.0032894]` |
| `avia` → `camera` | `[0.0178448, 0.0894589, 0.161556]` | `[0.0210767, -0.999397, -0.0275965, -0.0038717, 0.0275209, -0.999614, 0.99977, 0.0211754, -0.0032894]` |

### 1.4 Camera ↔ Livox MID-360

| Direction | T (m) | R |
|-----------|-------|---|
| `camera` → `mid360` | `[0.07433267, -0.03806468, -0.43920869]` | `[0.0722207, 0.5210047, 0.8504930, -0.9973866, 0.0394820, 0.0605080, -0.0020542, -0.8526402, 0.5224945]` |
| `mid360` → `camera` | `[-0.0442358, -0.411712, 0.168568]` | `[0.0722207, -0.997387, -0.0020542, 0.521005, 0.039482, -0.85264, 0.850493, 0.060508, 0.522494]` |

### 1.5 Camera IMU ↔ Livox MID-360

| Direction | T (m) | R |
|-----------|-------|---|
| `camera_imu` → `mid360` | `[0.046263, -0.00420981, -0.460019]` | `[0.0981574, 0.514299, 0.851975, -0.995106, 0.0409386, 0.0899349, 0.0113748, -0.856633, 0.515801]` |
| `mid360` → `camera_imu` | `[-0.00349766, -0.417688, 0.198242]` | `[0.0981574, -0.995106, 0.0113748, 0.514299, 0.0409386, -0.856633, 0.851975, 0.0899349, 0.515801]` |

### 1.6 LiDAR ↔ Built-in IMU

| Direction | T (m) | R |
|-----------|-------|---|
| `avia` → `avia_imu` | `[0.04165, 0.02326, -0.0284]` | Identity |
| `mid360` → `mid360_imu` | `[-0.011, -0.02329, 0.04412]` | Identity |

> **ROS topics:** `/livox/avia/imu`, `/livox/mid360/imu`, `/camera/imu`

### 1.7 Other Sensors

| Direction | T (m) | R |
|-----------|-------|---|
| GNSS antenna → `avia` | `[-0.358, 0.086, 0.429]` | — |
| `avia` → `mid360` | `[0.259632, -0.0425554, -0.431109]` | `[0.849802, -0.0398291, -0.525594, 0.0393196, 0.999153, -0.0121414, 0.525632, -0.0103484, 0.850649]` |

### 1.8 Wheel Odometry → Camera IMU

4×4 homogeneous transform `body_T_wheel` (OpenCV matrix format):

```yaml
body_T_wheel: !!opencv-matrix
  rows: 4
  cols: 4
  dt: d
  data: [ 4.2873564019253907e-02, -9.9906999607154057e-01,
          4.5826256555663858e-03, 1.0000278019634017e-00,
          2.3548883729155812e-02, -3.5750257528033291e-03,
          -9.9971629438855181e-01, 0.0477569625897234e-01,
          9.9880293731215963e-01, 4.2969316267296165e-02,
          2.3373709079293481e-02, 2.0902387796334685e-01, 0., 0., 0., 1. ]
```

> **ROS topic:** `/odom`

---

## 2. Camera Intrinsics

Realsense D435i RGB camera (`/camera/color/image_raw`), pinhole model, image size **640 × 480**.

```yaml
model_type: PINHOLE
camera_name: camera
image_width: 640
image_height: 480
distortion_parameters:
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
projection_parameters:
  fx: 607.79772949218
  fy: 607.83526613281
  cx: 328.79772949218
  cy: 245.53321838378
```

---

## 3. IMU Noise Parameters

Allan-variance calibration was performed on the built-in IMUs of Livox Avia and Livox MID-360. Parameters follow the Kalibr / `camera_imu_calib` convention:

| Symbol | Meaning | Unit |
|--------|---------|------|
| `gyr_n` | Gyroscope white noise density | rad/s/√Hz |
| `gyr_w` | Gyroscope bias random walk | rad/s²/√Hz |
| `acc_n` | Accelerometer white noise density | m/s²/√Hz |
| `acc_w` | Accelerometer bias random walk | m/s³/√Hz |

### 3.1 Livox Avia IMU (`/livox/avia/imu`)

```yaml
%YAML:1.0
---
type: IMU
name: avia_livox
Gyr:
   unit: " rad/s"
   avg-axis:
      gyr_n: 3.2194626705221046e-03
      gyr_w: 7.1383392426109448e-05
   x-axis:
      gyr_n: 3.8712676707517088e-03
      gyr_w: 5.4232235093274314e-05
   y-axis:
      gyr_n: 3.0304141908419177e-03
      gyr_w: 3.4490973342527322e-05
   z-axis:
      gyr_n: 2.7567061499726890e-03
      gyr_w: 1.2542696884252670e-04
Acc:
   unit: " m/s^2"
   avg-axis:
      acc_n: 2.0846225165441278e-02
      acc_w: 3.4099978362929511e-04
   x-axis:
      acc_n: 2.3192944295536082e-02
      acc_w: 2.5468552538454820e-04
   y-axis:
      acc_n: 1.6171815820099911e-02
      acc_w: 4.4489284947598376e-04
   z-axis:
      acc_n: 2.3173915380687839e-02
      acc_w: 3.2342097602735333e-04
```

### 3.2 Livox MID-360 IMU (`/livox/mid360/imu`)

```yaml
%YAML:1.0
---
type: IMU
name: mid360_livox
Gyr:
   unit: " rad/s"
   avg-axis:
      gyr_n: 1.3254540721802960e-03
      gyr_w: 5.2026292534341382e-05
   x-axis:
      gyr_n: 1.7365308449651231e-03
      gyr_w: 7.5454684311023211e-05
   y-axis:
      gyr_n: 9.0754865192403493e-04
      gyr_w: 3.6022415725254170e-05
   z-axis:
      gyr_n: 1.3322827196517301e-03
      gyr_w: 4.4601777566746771e-05
Acc:
   unit: " m/s^2"
   avg-axis:
      acc_n: 1.1472382386578356e-02
      acc_w: 3.3397564584716999e-04
   x-axis:
      acc_n: 1.0782603227998023e-02
      acc_w: 3.6009068179303791e-04
   y-axis:
      acc_n: 7.7407256618423868e-03
      acc_w: 2.3928539706832852e-04
   z-axis:
      acc_n: 1.5893818269894656e-02
      acc_w: 4.0255085868014343e-04
```
