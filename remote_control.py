#!/usr/bin/env python3

# Import core ROS 2 Python client library
import rclpy
# Import the base Node class to create a ROS 2 node
from rclpy.node import Node
# Import message types: multi-integer array and string
from std_msgs.msg import Int32MultiArray, String
# Import Adafruit ServoKit (used in related code for servo control)
from adafruit_servokit import ServoKit
# Import Raspberry Pi GPIO library to control the relay pin
import RPi.GPIO as GPIO


# Utility function that limits x to stay within [lo, hi]
def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ROS 2 node that listens to RC PWM values and working mode,
# computes servo angles for left/right throttles, and controls a relay.
class RemoteThrottleNode(Node):
    def __init__(self):
        # Initialize the Node with the name "remote_throttle"
        super().__init__('remote_throttle')

        # Subscription to RC channel data (Int32MultiArray of PWM values)
        self.rc_subscriber_ = self.create_subscription(
            Int32MultiArray,          # Message type
            'rc_channels',            # Topic name to subscribe to
            self.throttle_callback,   # Callback when a new message arrives
            10                        # QoS queue size
        )

        # Subscription to working mode (String topic)
        self.working_mode_subscriber_ = self.create_subscription(
            String,                   # Message type
            'working_mode',           # Topic name
            self.mode_callback,       # Callback for mode updates
            10                        # QoS queue size
        )

        # Publisher for the computed remote servo angles (left/right)
        self.rc_angle_publisher_ = self.create_publisher(
            Int32MultiArray,          # Message type
            'remote_angle',           # Topic name
            10                        # QoS queue size
        )

        # Log that the node has successfully started
        self.get_logger().info("Remote Throttle Started")

        # ----------------- Fine tuning / calibration values -----------------

        # Minimum PWM signal expected from RC input (microseconds)
        self.pwm_min = 1100
        # Center / neutral PWM value (microseconds)
        self.pwm_center = 1500
        # Maximum PWM signal expected from RC input (microseconds)
        self.pwm_max = 1900
        # Deadband around center where input is treated as exactly neutral
        self.deadband_us = 40

        # Minimum servo angle (degrees)
        self.ang_min = 0
        # Center / idle servo angle (degrees)
        self.ang_center = 80
        # Maximum servo angle (degrees)
        self.ang_max = 180

        # Exponential smoothing factor for servo movement (0–1)
        self.alpha = 0.25
        # Maximum allowed change in servo angle per update (deg)
        self.max_slew_deg = 16.0

        # ----------------- Servo state / working mode -----------------

        # Current working mode, defaulting to STANDBY until a mode is received
        self.current_working_mode = "STANDBY"

        # Initialize both servo angles at the center position (float for math)
        self.left_angle = float(self.ang_center)
        self.right_angle = float(self.ang_center)

        # ----------------- Relay setup -----------------

        # GPIO pin number (BCM numbering) used to drive the relay
        self.RELAY_GPIO = 5
        # Indicates relay is activated when the pin is driven LOW
        self.ACTIVE_LOW = True
        # Use Broadcom (BCM) GPIO numbering
        GPIO.setmode(GPIO.BCM)
        # Configure relay pin as output and set to an initial "off" state
        GPIO.setup(
            self.RELAY_GPIO,
            GPIO.OUT,
            initial=GPIO.HIGH if self.ACTIVE_LOW else GPIO.LOW
        )
        # Ensure relay is off at startup
        self.relay(False)

    # ----------------- Relay control helper -----------------
    def relay(self, on: bool):
        """
        Turn the relay on or off.

        Because the relay is active-low, the logic selects HIGH/LOW
        outputs depending on desired state and wiring configuration.
        """
        GPIO.output(
            self.RELAY_GPIO,
            GPIO.LOW if (on and self.ACTIVE_LOW) else
            GPIO.HIGH if (on and not self.ACTIVE_LOW) else
            GPIO.HIGH if (not on and self.ACTIVE_LOW) else
            GPIO.LOW
        )

    # ----------------- Working mode callback -----------------
    def mode_callback(self, msg):
        """
        Callback for the 'working_mode' topic.

        Updates the current working mode string and sets the relay based
        on whether the system is in EMERGENCY STOP or not.
        """
        # Store the mode string from the incoming message
        self.current_working_mode = msg.data

        # If the system is in EMERGENCY STOP, energize the relay
        if self.current_working_mode == "EMERGENCY STOP":
            self.relay(True)
        # Otherwise, keep the relay off
        else:
            self.relay(False)

    # ----------------- PWM to servo angle mapping -----------------
    def rc_pwm_to_angle(self, pwm):
        """
        Convert a raw PWM value (microseconds) to a servo angle (degrees),
        applying a deadband around center and clamping at angle limits.
        """
        # If PWM is within the deadband around center, return exact center angle
        if abs(pwm - self.pwm_center) <= self.deadband_us:
            return self.ang_center

        # If PWM is above center, map it from (center+deadband .. max) to (center .. max)
        if pwm > self.pwm_center:
            # Fraction of the way from post-deadband center to max PWM
            frac = (pwm - (self.pwm_center + self.deadband_us)) / \
                   (self.pwm_max - (self.pwm_center + self.deadband_us))
            # Interpolate between center angle and max angle and clamp
            return clamp(
                self.ang_center + frac * (self.ang_max - self.ang_center),
                self.ang_center,
                self.ang_max
            )

        # If PWM is below center, map it from (min .. center-deadband) to (min .. center)
        else:
            # Fraction of the way from min PWM up to pre-deadband center
            frac = ((self.pwm_center - self.deadband_us) - pwm) / \
                   ((self.pwm_center - self.deadband_us) - self.pwm_min)
            # Interpolate between center angle and min angle and clamp
            return clamp(
                self.ang_center - frac * (self.ang_center - self.ang_min),
                self.ang_min,
                self.ang_center
            )

    # ----------------- Smoothing and slew rate limiting -----------------
    def smooth_and_slew(self, current, target):
        """
        Apply exponential smoothing toward the target angle and then
        limit how fast the angle is allowed to change (slew rate limit).
        """
        # Move a fraction (alpha) toward the target angle
        smoothed = current + self.alpha * (target - current)
        # Compute how much the angle changed from current to smoothed
        delta = clamp(smoothed - current, -self.max_slew_deg, self.max_slew_deg)
        # Apply the bounded change to the current angle
        return current + delta

    # ----------------- RC channel callback -----------------
    def throttle_callback(self, msg: Int32MultiArray):
        """
        Callback for 'rc_channels' messages.

        In EMERGENCY STOP or STANDBY: publish center angles.
        In REMOTE CONTROL: convert PWM to angles, smooth, limit slew,
        and publish resulting left/right servo angles.
        Other modes (e.g., AUTONOMOUS) are handled elsewhere.
        """
        # If not actively under remote control, hold servos at center
        if self.current_working_mode in ["EMERGENCY STOP", "STANDBY"]:
            # Construct a new message with centered angles for both servos
            msg = Int32MultiArray()
            msg.data = [int(self.ang_center), int(self.ang_center)]
            # Publish idle/center angles
            self.rc_angle_publisher_.publish(msg)

        # When in REMOTE CONTROL mode, use RC input to drive angles
        elif self.current_working_mode == "REMOTE CONTROL":
            try:
                # Extract channel 1 and 2 PWM values from the incoming array
                ch1 = msg.data[0]
                ch2 = msg.data[1]

                # Map raw PWM inputs to target servo angles
                tgt_left = self.rc_pwm_to_angle(ch1)
                tgt_right = self.rc_pwm_to_angle(ch2)

                # Apply smoothing and slew rate limiting to current angles
                self.left_angle = self.smooth_and_slew(self.left_angle, tgt_left)
                self.right_angle = self.smooth_and_slew(self.right_angle, tgt_right)

                # Build message with integer angles to publish
                msg = Int32MultiArray()
                msg.data = [int(self.left_angle), int(self.right_angle)]
                # Publish the updated servo angles
                self.rc_angle_publisher_.publish(msg)

                # Optional debug log: raw PWM and resulting angles
                self.get_logger().debug(
                    f"L:{ch1}->{self.left_angle:.1f}°, R:{ch2}->{self.right_angle:.1f}°"
                )

            except IndexError:
                # If the rc_channels message did not contain enough data
                self.get_logger().warn("Received bad rc_channels message, setting idle")

                # Publish center angles as a safe fallback
                msg = Int32MultiArray()
                msg.data = [int(self.ang_center), int(self.ang_center)]
                self.rc_angle_publisher_.publish(msg)

        # For any other working mode (e.g., autonomous), this node does nothing here
        else:
            pass  # Autonomous mode handled elsewhere

    # ----------------- Cleanup on node destruction -----------------
    def destroy_node(self):
        """
        Override destroy_node to ensure the relay is turned off and
        GPIO resources are cleaned up properly before shutdown.
        """
        # Ensure relay is off before shutdown
        self.relay(False)
        # Explicitly set relay pin to a safe "off" level
        GPIO.output(self.RELAY_GPIO, GPIO.HIGH if self.ACTIVE_LOW else GPIO.LOW)
        # Release all GPIO pins used by this program
        GPIO.cleanup()
        # Call the parent class's destroy_node implementation
        super().destroy_node()


# ----------------- Standard ROS 2 Python entry point -----------------
def main():
    # Initialize the rclpy communication layer
    rclpy.init()
    # Create an instance of the RemoteThrottleNode
    node = RemoteThrottleNode()
    try:
        # Spin the node so callbacks (subscriptions, timers) are processed
        rclpy.spin(node)
    finally:
        # On shutdown, destroy the node (cleanup GPIO, etc.)
        node.destroy_node()
        # Shutdown rclpy and release ROS 2 resources
        rclpy.shutdown()


# Run main() only if this file is executed directly (not imported)
if __name__ == "__main__":
    main()
