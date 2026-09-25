# Failure excerpts

These excerpts are transcribed from the local raw files in `runs/20260922-225811631/logs/` and document why no visual stream was captured.

RGB recorder (`rgb.err.log`):

```text
ImportError: DLL load failed while importing cv2: An Application Control policy has blocked this file.
```

ROS bag writer (`bag.err.log`):

```text
ImportError: DLL load failed while importing _rclpy_pybind11: An Application Control policy has blocked this file.
```

The `ros2` router launcher failed on the same blocked `_rclpy_pybind11` extension (`zenoh.err.log`). The capture wrapper then exited 1 because `capture/rgb_video.json` did not exist. The complete local logs remain in the ignored run directory; no policy change or repair was attempted.
