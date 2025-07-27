# TODO List

## Findings to Address Tomorrow

1. **BLE Task Delay**:
   - Investigate and reduce the delay in the BLE task loop to process commands more frequently.
   - Current delay: `vTaskDelay(1000 / portTICK_PERIOD_MS)`.
   - **Progress**: Reduced delay to improve responsiveness.

2. **Command Queue Overload**:
   - Check if the command queue is being overloaded.
   - Adjust the queue size or the rate of command submission if necessary.
   - **Progress**: Increased queue size to handle more commands.

3. **BLE Connection Latency**:
   - Ensure the BLE connection is stable and optimized to reduce latency.
   - **Progress**: Optimized connection handling in BLE task.

4. **Sound Command Execution**:
   - Verify if the sound commands are being processed promptly after submission.
   - **Progress**: Improved sound playback timing.

5. **Demo Step Timing**:
   - Confirm that the `DEMO_STEP_DURATION` is appropriate for the demo sequence.
   - Current value: `5000` milliseconds.
   - **Progress**: Timing confirmed to be appropriate.
