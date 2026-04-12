from hal.hardware.config import ServoParams, PWMParams
import numpy as np
import logging

# Try to import Adafruit servo controller
try:
    from hal.hardware.adafruit_servo_controller import AdafruitServoController, is_adafruit_available
    ADAFRUIT_AVAILABLE = is_adafruit_available()
except ImportError:
    ADAFRUIT_AVAILABLE = False
    AdafruitServoController = None

# Global Adafruit controller instance
_global_adafruit_controller = None

def get_global_adafruit_controller():
    """Get or create global Adafruit servo controller instance"""
    global _global_adafruit_controller
    if _global_adafruit_controller is None and ADAFRUIT_AVAILABLE:
        try:
            _global_adafruit_controller = AdafruitServoController()
            logging.info("✅ Global Adafruit servo controller initialized")
        except Exception as e:
            logging.error(f"Failed to initialize global Adafruit controller: {e}")
            _global_adafruit_controller = None
    return _global_adafruit_controller

class HardwareInterface:
    def __init__(self, use_adafruit=True):
        """Initialize hardware interface with optional Adafruit backend

        Parameters
        ----------
        use_adafruit : bool
            Use Adafruit CircuitPython backend if available (default: True)
        """
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()
        self.use_adafruit = use_adafruit and ADAFRUIT_AVAILABLE

        # Initialize Adafruit controller if requested and available
        if self.use_adafruit:
            self.adafruit_controller = get_global_adafruit_controller()
            if self.adafruit_controller and self.adafruit_controller.is_available():
                logging.info("✅ HardwareInterface using Adafruit CircuitPython backend")
            else:
                logging.warning("⚠️ Adafruit backend requested but not available, falling back to sysfs")
                self.use_adafruit = False
        else:
            self.adafruit_controller = None
            logging.info("ℹ️ HardwareInterface using sysfs PWM backend")

    def set_actuator_postions(self, joint_angles):
        """Set all actuator positions (existing API - unchanged)"""
        send_servo_commands(self.pwm_params, self.servo_params, joint_angles, self.use_adafruit)

    def set_actuator_position(self, joint_angle, axis, leg):
        """Set single actuator position (existing API - unchanged)"""
        send_servo_command(self.pwm_params, self.servo_params, joint_angle, axis, leg, self.use_adafruit)


def pwm_to_duty_cycle(pulsewidth_micros, pwm_params):
    """Converts a pwm signal (measured in microseconds) to a corresponding duty cycle on the gpio pwm pin

    Parameters
    ----------
    pulsewidth_micros : float
        Width of the pwm signal in microseconds
    pwm_params : PWMParams
        PWMParams object

    Returns
    -------
    float
        PWM duty cycle corresponding to the pulse width
    """
    pulsewidth_micros = int(pulsewidth_micros / 1e6 * pwm_params.freq * pwm_params.range)
    if np.isnan(pulsewidth_micros):
        return 0
    return int(np.clip(pulsewidth_micros, 0, 4096))


def angle_to_pwm(angle, servo_params, axis_index, leg_index):
    """Converts a desired servo angle into the corresponding PWM command

    Parameters
    ----------
    angle : float
        Desired servo angle, relative to the vertical (z) axis
    servo_params : ServoParams
        ServoParams object
    axis_index : int
        Specifies which joint of leg to control. 0 is abduction servo, 1 is inner hip servo, 2 is outer hip servo.
    leg_index : int
        Specifies which leg to control. 0 is front-right, 1 is front-left, 2 is back-right, 3 is back-left.

    Returns
    -------
    float
        PWM width in microseconds
    """
    neutral_angle = servo_params.neutral_angles[axis_index, leg_index]
    multiplier = servo_params.servo_multipliers[axis_index, leg_index]

    angle_deviation = (angle - neutral_angle) * multiplier
    pulse_width_micros = (
            servo_params.neutral_position_pwm
            + servo_params.micros_per_rad * angle_deviation
    )

    # Smart logging: only log when joint angles change significantly (used by both servo_interface and calibrate_tool)
    if not hasattr(angle_to_pwm, 'last_angles'):
        angle_to_pwm.last_angles = {}
        angle_to_pwm.log_count = 0

    angle_to_pwm.log_count += 1

    # Create unique key for this joint
    joint_key = f"{leg_index}_{axis_index}"
    angle_deg = angle * 180.0 / 3.14159

    # Check if this is first time or angle changed significantly (>5 degrees)
    should_log = False
    if joint_key not in angle_to_pwm.last_angles:
        should_log = True  # First time for this joint
    else:
        angle_change = abs(angle_deg - angle_to_pwm.last_angles[joint_key])
        if angle_change > 5.0:  # 5 degree threshold
            should_log = True

    if should_log:
        angle_to_pwm.last_angles[joint_key] = angle_deg
        leg_names = ["FR", "FL", "BR", "BL"]
        joint_names = ["Hip", "Thigh", "Calf"]
        neutral_deg = neutral_angle * 180.0 / 3.14159

        print(f"🔧 Servo [{leg_names[leg_index]}-{joint_names[axis_index]}]: "
              f"{angle_deg:+6.1f}° (neutral: {neutral_deg:+5.1f}°) → {pulse_width_micros:.0f}μs")

    return pulse_width_micros


def angle_to_duty_cycle(angle, pwm_params, servo_params, axis_index, leg_index):
    duty_cycle_f = angle_to_pwm(angle, servo_params, axis_index, leg_index) * 1e3
    if np.isnan(duty_cycle_f):
        return 0
    return int(duty_cycle_f)


def initialize_pwm(pi, pwm_params):
    pi.set_pwm_freq(pwm_params.freq)


def send_servo_commands(pwm_params, servo_params, joint_angles, use_adafruit=True):
    """Send commands to all servos (enhanced with Adafruit backend)

    Parameters
    ----------
    pwm_params : PWMParams
        PWM parameters including pin mapping
    servo_params : ServoParams
        Servo parameters including calibration data
    joint_angles : numpy.ndarray
        Joint angles array [3x4] for all servos
    use_adafruit : bool
        Use Adafruit backend if available (default: True)
    """
    if use_adafruit and ADAFRUIT_AVAILABLE:
        # Use Adafruit CircuitPython backend (proven working)
        controller = get_global_adafruit_controller()
        if controller and controller.is_available():
            for leg_index in range(4):
                for axis_index in range(3):
                    # Use existing angle calculation (this works correctly)
                    pulse_width_micros = angle_to_pwm(
                        joint_angles[axis_index, leg_index],
                        servo_params,
                        axis_index,
                        leg_index,
                    )
                    # Get channel from existing pin mapping
                    channel = pwm_params.pins[axis_index, leg_index]

                    # Use Adafruit backend instead of sysfs
                    controller.set_servo_pulse(channel, pulse_width_micros)
            return

    # Fallback to original sysfs implementation
    logging.warning("Using sysfs PWM fallback (may not work)")
    for leg_index in range(4):
        for axis_index in range(3):
            duty_cycle = angle_to_duty_cycle(
                joint_angles[axis_index, leg_index],
                pwm_params,
                servo_params,
                axis_index,
                leg_index,
            )
            # write duty_cycle to pwm linux kernel node
            try:
                file_node = "/sys/class/pwm/pwmchip0/pwm" + str(pwm_params.pins[axis_index, leg_index]) + "/duty_cycle"
                with open(file_node, "w") as f:
                    f.write(str(duty_cycle))
            except Exception as e:
                logging.error(f"sysfs PWM write failed for channel {pwm_params.pins[axis_index, leg_index]}: {e}")


def send_servo_command(pwm_params, servo_params, joint_angle, axis, leg, use_adafruit=True):
    """Send command to single servo (enhanced with Adafruit backend)

    Parameters
    ----------
    pwm_params : PWMParams
        PWM parameters including pin mapping
    servo_params : ServoParams
        Servo parameters including calibration data
    joint_angle : float
        Joint angle in radians
    axis : int
        Axis index (0=abduction, 1=inner, 2=outer)
    leg : int
        Leg index (0=front-right, 1=front-left, 2=back-right, 3=back-left)
    use_adafruit : bool
        Use Adafruit backend if available (default: True)
    """
    if use_adafruit and ADAFRUIT_AVAILABLE:
        # Use Adafruit CircuitPython backend (proven working)
        controller = get_global_adafruit_controller()
        if controller and controller.is_available():
            # Use existing angle calculation (this works correctly)
            pulse_width_micros = angle_to_pwm(joint_angle, servo_params, axis, leg)
            # Get channel from existing pin mapping
            channel = pwm_params.pins[axis, leg]

            # Use Adafruit backend instead of sysfs
            controller.set_servo_pulse(channel, pulse_width_micros)
            return

    # Fallback to original sysfs implementation
    logging.warning(f"Using sysfs PWM fallback for servo {axis},{leg} (may not work)")
    try:
        duty_cycle = angle_to_duty_cycle(joint_angle, pwm_params, servo_params, axis, leg)
        file_node = "/sys/class/pwm/pwmchip0/pwm" + str(pwm_params.pins[axis, leg]) + "/duty_cycle"
        with open(file_node, "w") as f:
            f.write(str(duty_cycle))
    except Exception as e:
        logging.error(f"sysfs PWM write failed for servo {axis},{leg}: {e}")


def deactivate_servos(pi, pwm_params, use_adafruit=True):
    """Deactivate all servos (enhanced with Adafruit backend)

    Parameters
    ----------
    pi : object
        Legacy pi object (unused with Adafruit backend)
    pwm_params : PWMParams
        PWM parameters including pin mapping
    use_adafruit : bool
        Use Adafruit backend if available (default: True)
    """
    if use_adafruit and ADAFRUIT_AVAILABLE:
        # Use Adafruit CircuitPython backend
        controller = get_global_adafruit_controller()
        if controller and controller.is_available():
            controller.disable_all_servos()
            logging.info("✅ All servos deactivated via Adafruit backend")
            return

    # Fallback to original implementation
    logging.warning("Using legacy servo deactivation (may not work)")
    try:
        for leg_index in range(4):
            for axis_index in range(3):
                pi.set_pwm(pwm_params.pins[axis_index, leg_index], 0, 0)
    except Exception as e:
        logging.error(f"Legacy servo deactivation failed: {e}")
