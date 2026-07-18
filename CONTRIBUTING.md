# Contributing

Thanks for helping improve TurtleBot 4 Autonomy.

1. Open an issue for significant changes so the approach can be discussed.
2. Create a focused branch and keep changes limited to one concern.
3. Follow the existing Python and ROS 2 conventions; do not mix generated files into commits.
4. Build and test before submitting:

   ```bash
   colcon build --symlink-install
   colcon test --event-handlers console_direct+
   colcon test-result --verbose
   ```

5. Submit a pull request describing the change, how it was tested, and whether it affects real-robot behavior.

Bug reports should include the ROS 2 distribution, hardware or simulator setup, reproduction steps, logs, and expected behavior. By contributing, you agree that your work is provided under the repository's MIT License.
