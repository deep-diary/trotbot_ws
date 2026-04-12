#!/usr/bin/env python3
"""
Adafruit CircuitPython Backend for TrotBot Servo Control
Provides reliable PCA9685 servo control using proven Adafruit libraries
"""

import time
import logging
from datetime import datetime

# Global controller instance
_adafruit_controller = None

def get_adafruit_controller():
    """Get or create global Adafruit servo controller instance"""
    global _adafruit_controller
    if _adafruit_controller is None:
        _adafruit_controller = AdafruitServoController()
    return _adafruit_controller

class AdafruitServoController:
    """Adafruit CircuitPython-based servo controller for PCA9685"""

    def __init__(self, i2c_address=0x40, frequency=50):
        """Initialize PCA9685 servo controller

        Parameters
        ----------
        i2c_address : int
            I2C address of PCA9685 (default: 0x40)
        frequency : int
            PWM frequency in Hz (default: 50Hz for servos)
        """
        self.i2c_address = i2c_address
        self.frequency = frequency
        self.pca = None
        self.initialized = False
        self.command_count = 0

        # Debug filtering settings (temporary for troubleshooting)
        # self.debug_channels = {13, 14}  # Front-Right leg Calf and Thigh only - COMMENTED OUT
        self.debug_channels = set()  # No debug channels - debug output disabled
        self.last_debug_time = {}  # Per-channel debug timing
        self.debug_interval = 5.0  # Debug output every 5 seconds

        # Initialize per-channel debug timing
        for channel in self.debug_channels:
            self.last_debug_time[channel] = 0
        
        try:
            # Import Adafruit libraries
            import board
            import busio
            from adafruit_pca9685 import PCA9685
            
            # Initialize I2C and PCA9685
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c, address=i2c_address)
            self.pca.frequency = frequency
            
            self.initialized = True
            # Initialization debug (filtered for Front-Right leg monitoring) - COMMENTED OUT
            # print(f"🔧 INIT_DEBUG: Adafruit PCA9685 initialized - monitoring channels {sorted(self.debug_channels)} (Front-Right leg)")
            logging.info(f"✅ Adafruit PCA9685 initialized at address 0x{i2c_address:02X}, {frequency}Hz")
            
        except ImportError as e:
            logging.warning(f"Adafruit CircuitPython libraries not available: {e}")
            self.initialized = False
        except Exception as e:
            logging.error(f"Failed to initialize Adafruit PCA9685: {e}")
            self.initialized = False
    
    def is_available(self):
        """Check if Adafruit controller is available and initialized"""
        return self.initialized and self.pca is not None
    
    def set_servo_pulse(self, channel, pulse_width_micros):
        """Set servo pulse width in microseconds

        Parameters
        ----------
        channel : int
            PCA9685 channel (0-15)
        pulse_width_micros : float
            Pulse width in microseconds (typically 1000-2000 for servos)

        Returns
        -------
        bool
            True if successful, False otherwise
        """
        # Increment command counter for tracking
        self.command_count += 1
        current_time = time.time()

        # Filtered debug output: only specific channels and per-channel time interval
        should_debug = (
            channel in self.debug_channels and
            (current_time - self.last_debug_time.get(channel, 0)) >= self.debug_interval
        )

        if should_debug:
            self.last_debug_time[channel] = current_time
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            channel_name = {13: "Calf", 14: "Thigh"}.get(channel, f"Ch{channel}")
            print(f"🔧 SERVO_DEBUG [{timestamp}] Command #{self.command_count}: Channel {channel} ({channel_name})")
            print(f"    Input: {pulse_width_micros:.3f}μs")

        if not self.is_available():
            if should_debug:
                print(f"    ❌ Controller not available")
            return False

        try:
            # Convert microseconds to PCA9685 duty cycle
            # PCA9685 uses 20ms period (50Hz) = 20,000 microseconds
            pulse_fraction = pulse_width_micros / 20000.0
            duty_cycle_raw = pulse_fraction * 0xFFFF
            duty_cycle = int(duty_cycle_raw)

            # Clamp duty cycle to valid range
            duty_cycle_clamped = max(0, min(duty_cycle, 0xFFFF))

            # Filtered calculation details (only for monitored channels at debug interval)
            if should_debug:
                print(f"    Calculations:")
                print(f"      Period: 20000μs (50Hz)")
                print(f"      Pulse fraction: {pulse_fraction:.6f}")
                print(f"      Duty cycle raw: {duty_cycle_raw:.1f}")
                print(f"      Duty cycle int: {duty_cycle}")
                print(f"      Duty cycle clamped: {duty_cycle_clamped}")
                if duty_cycle != duty_cycle_clamped:
                    print(f"      ⚠️ Duty cycle was clamped!")

            # Set the duty cycle on the specified channel
            self.pca.channels[channel].duty_cycle = duty_cycle_clamped

            # Filtered success confirmation (only for monitored channels at debug interval)
            if should_debug:
                channel_name = {13: "Calf", 14: "Thigh"}.get(channel, f"Ch{channel}")
                print(f"    ✅ PCA9685 channel {channel} ({channel_name}) set to {duty_cycle_clamped} (0x{duty_cycle_clamped:04X})")

            return True

        except Exception as e:
            error_msg = f"Failed to set servo pulse on channel {channel}: {e}"
            if should_debug:
                channel_name = {13: "Calf", 14: "Thigh"}.get(channel, f"Ch{channel}")
                print(f"    ❌ Exception on channel {channel} ({channel_name}): {error_msg}")
            logging.error(error_msg)
            return False
    
    def set_servo_angle(self, channel, angle_degrees):
        """Set servo angle in degrees (convenience method)

        Parameters
        ----------
        channel : int
            PCA9685 channel (0-15)
        angle_degrees : float
            Servo angle in degrees (0-180)

        Returns
        -------
        bool
            True if successful, False otherwise
        """
        # Filtered angle conversion debug (only for monitored channels with per-channel timing)
        if channel in self.debug_channels:
            current_time = time.time()
            if (current_time - self.last_debug_time.get(channel, 0)) >= self.debug_interval:
                channel_name = {13: "Calf", 14: "Thigh"}.get(channel, f"Ch{channel}")
                print(f"🔧 ANGLE_DEBUG: Converting angle {angle_degrees}° to pulse width for channel {channel} ({channel_name})")

        # Convert angle to pulse width (standard servo mapping)
        # 0° = 1000μs, 90° = 1500μs, 180° = 2000μs
        pulse_width_micros = 1000 + (angle_degrees / 180.0) * 1000

        # Filtered conversion result debug (only for monitored channels with per-channel timing)
        if channel in self.debug_channels:
            current_time = time.time()
            if (current_time - self.last_debug_time.get(channel, 0)) >= self.debug_interval:
                print(f"    Angle conversion: {angle_degrees}° → {pulse_width_micros:.3f}μs")

        return self.set_servo_pulse(channel, pulse_width_micros)

    def get_debug_status(self):
        """Get current debug filtering status

        Returns
        -------
        dict
            Debug status information
        """
        current_time = time.time()
        status = {
            'debug_channels': sorted(self.debug_channels),
            'debug_interval': self.debug_interval,
            'command_count': self.command_count,
            'per_channel_timing': {}
        }

        for channel in self.debug_channels:
            last_time = self.last_debug_time.get(channel, 0)
            time_since_last = current_time - last_time
            channel_name = {13: "Calf", 14: "Thigh"}.get(channel, f"Ch{channel}")
            status['per_channel_timing'][channel] = {
                'name': channel_name,
                'last_debug_time': last_time,
                'time_since_last_debug': time_since_last,
                'ready_for_debug': time_since_last >= self.debug_interval
            }

        return status

    def force_debug_output(self, channel, pulse_width_micros):
        """Force debug output for a specific channel (for testing)

        Parameters
        ----------
        channel : int
            PCA9685 channel
        pulse_width_micros : float
            Pulse width in microseconds
        """
        if channel in self.debug_channels:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            channel_name = {13: "Calf", 14: "Thigh"}.get(channel, f"Ch{channel}")
            print(f"🔧 FORCED_DEBUG [{timestamp}] Channel {channel} ({channel_name})")
            print(f"    Input: {pulse_width_micros:.3f}μs")
            print(f"    Command count: {self.command_count}")

    def disable_servo(self, channel):
        """Disable servo on specified channel
        
        Parameters
        ----------
        channel : int
            PCA9685 channel (0-15)
            
        Returns
        -------
        bool
            True if successful, False otherwise
        """
        if not self.is_available():
            return False
            
        try:
            # Set duty cycle to 0 to disable servo
            self.pca.channels[channel].duty_cycle = 0
            return True
        except Exception as e:
            logging.error(f"Failed to disable servo on channel {channel}: {e}")
            return False
    
    def disable_all_servos(self):
        """Disable all servo channels
        
        Returns
        -------
        bool
            True if successful, False otherwise
        """
        if not self.is_available():
            return False
            
        success = True
        for channel in range(16):
            if not self.disable_servo(channel):
                success = False
        
        return success
    
    def test_channel(self, channel, test_angles=[90, 0, 180, 90]):
        """Test servo movement on specified channel
        
        Parameters
        ----------
        channel : int
            PCA9685 channel to test
        test_angles : list
            List of angles to test (default: [90, 0, 180, 90])
            
        Returns
        -------
        bool
            True if test completed without errors
        """
        if not self.is_available():
            print(f"❌ Adafruit controller not available for channel {channel}")
            return False
        
        print(f"🎯 Testing servo on PCA9685 channel {channel}")
        
        try:
            for angle in test_angles:
                print(f"  → Setting to {angle}°")
                if self.set_servo_angle(channel, angle):
                    time.sleep(1)  # Wait for servo to move
                else:
                    print(f"  ❌ Failed to set angle {angle}°")
                    return False
            
            print(f"✅ Channel {channel} test completed")
            return True
            
        except Exception as e:
            print(f"❌ Channel {channel} test failed: {e}")
            return False
    
    def cleanup(self):
        """Clean up PCA9685 resources"""
        if self.pca:
            try:
                # Disable all servos before cleanup
                self.disable_all_servos()
                self.pca.deinit()
                logging.info("✅ Adafruit PCA9685 cleaned up")
            except Exception as e:
                logging.error(f"Error during PCA9685 cleanup: {e}")
    
    def __del__(self):
        """Destructor - ensure cleanup"""
        self.cleanup()


# Convenience functions for backward compatibility
def create_adafruit_controller():
    """Create and return Adafruit servo controller instance"""
    return AdafruitServoController()

def is_adafruit_available():
    """Check if Adafruit CircuitPython libraries are available"""
    try:
        import board
        import busio
        from adafruit_pca9685 import PCA9685
        return True
    except ImportError:
        return False
