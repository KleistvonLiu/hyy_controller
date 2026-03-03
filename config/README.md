# End Effector JSON Examples

This project supports these three end effector types in `config/jeserver_end_effectors.json`:

- `gripper_DH`
- `gripper_piper`
- `endeffector`

Use the `end_effectors` array, and keep its size equal to the robot count.

## 1) `gripper_DH` example

```json
{
  "end_effectors": [
    {
      "type": "gripper_DH",
      "port": "/dev/ttyUSB0",
      "baud": 115200,
      "id": 1
    }
  ]
}
```

## 2) `gripper_piper` example

```json
{
  "end_effectors": [
    {
      "type": "gripper_piper",
      "can_ifname": "can0",
      "recv_timeout_ms": 1000,
      "monitor_period_ms": 50,
      "fps_period_ms": 100,
      "is_ok_window": 5,
      "enable_sdk_gripper_limit": false,
      "enable_abnormal_filter": true,
      "sdk_range_min_m": 0.0,
      "sdk_range_max_m": 0.07
    }
  ]
}
```

## 3) `endeffector` example

```json
{
  "end_effectors": [
    {
      "type": "endeffector",
      "port": "/dev/ttyS1",
      "baud": 115200
    }
  ]
}
```

