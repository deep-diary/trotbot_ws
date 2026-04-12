"""
TrotBot Servo Calibration Module
Filesystem-based servo calibration storage for TrotBot quadruped robot
"""

import pickle
import numpy as np
from pathlib import Path
import logging

# Default calibration values
DEFAULT_MICROS_PER_RAD = 11.111 * 180.0 / np.pi
DEFAULT_NEUTRAL_ANGLE_DEGREES = np.array([
    [0., 0., 0., 0.],
    [0., 0., 0., 0.],
    [0., 0., 0., 0.]
])

# TrotBot calibration storage path
CALIBRATION_PATH = Path.home() / '.config' / 'trotbot' / 'nvmem'

# Global calibration variables
MICROS_PER_RAD = DEFAULT_MICROS_PER_RAD
NEUTRAL_ANGLE_DEGREES = DEFAULT_NEUTRAL_ANGLE_DEGREES.copy()

def read_calibration():
    """Read servo calibration data from TrotBot filesystem storage

    Returns
    -------
    dict
        Calibration data containing MICROS_PER_RAD and NEUTRAL_ANGLE_DEGREES
    """
    try:
        import numpy as np

        with open(CALIBRATION_PATH, 'rb') as fd:
            data = pickle.load(fd)
            logging.info(f"📁 Loaded calibration data from {CALIBRATION_PATH}")

            # Convert list back to numpy array if needed
            if isinstance(data['NEUTRAL_ANGLE_DEGREES'], list):
                data['NEUTRAL_ANGLE_DEGREES'] = np.array(data['NEUTRAL_ANGLE_DEGREES'])

            return data

    except ImportError:
        logging.warning("Numpy not available, using default calibration")
        return {
            'MICROS_PER_RAD': DEFAULT_MICROS_PER_RAD,
            'NEUTRAL_ANGLE_DEGREES': DEFAULT_NEUTRAL_ANGLE_DEGREES.copy()
        }
    except (FileNotFoundError, EOFError) as e:
        logging.warning(f"⚠️ Calibration file not found: {e}")
        logging.info("Using default calibration values")
        return {
            'MICROS_PER_RAD': DEFAULT_MICROS_PER_RAD,
            'NEUTRAL_ANGLE_DEGREES': DEFAULT_NEUTRAL_ANGLE_DEGREES.copy()
        }
    except Exception as e:
        logging.warning(f"⚠️ Could not read calibration data: {e}")
        logging.info("Using default calibration values")
        return {
            'MICROS_PER_RAD': DEFAULT_MICROS_PER_RAD,
            'NEUTRAL_ANGLE_DEGREES': DEFAULT_NEUTRAL_ANGLE_DEGREES.copy()
        }

def write_calibration(data):
    """Write servo calibration data to TrotBot filesystem storage

    Parameters
    ----------
    data : dict
        Calibration data containing MICROS_PER_RAD and NEUTRAL_ANGLE_DEGREES
    """
    try:
        import numpy as np

        # Ensure directory exists
        CALIBRATION_PATH.parent.mkdir(parents=True, exist_ok=True)

        # Convert to compatible format (no numpy dependencies in storage)
        compatible_data = {
            'MICROS_PER_RAD': float(data['MICROS_PER_RAD']),
            'NEUTRAL_ANGLE_DEGREES': data['NEUTRAL_ANGLE_DEGREES'].tolist() if isinstance(data['NEUTRAL_ANGLE_DEGREES'], np.ndarray) else data['NEUTRAL_ANGLE_DEGREES']
        }

        with open(CALIBRATION_PATH, 'wb') as fd:
            pickle.dump(compatible_data, fd, protocol=pickle.HIGHEST_PROTOCOL)

        logging.info(f"💾 Saved calibration data to {CALIBRATION_PATH}")

        # Update global variables
        global MICROS_PER_RAD, NEUTRAL_ANGLE_DEGREES
        MICROS_PER_RAD = data['MICROS_PER_RAD']
        NEUTRAL_ANGLE_DEGREES = np.array(data['NEUTRAL_ANGLE_DEGREES']) if not isinstance(data['NEUTRAL_ANGLE_DEGREES'], np.ndarray) else data['NEUTRAL_ANGLE_DEGREES']

    except Exception as e:
        logging.error(f"❌ Failed to save calibration data: {e}")
        raise

def get_calibration_status():
    """Get current calibration status and values

    Returns
    -------
    dict
        Status information including file existence, values, and path
    """
    calibration_exists = CALIBRATION_PATH.exists()

    # Try to get fresh calibration data
    try:
        fresh_data = read_calibration()
        micros_per_rad = fresh_data['MICROS_PER_RAD']
        neutral_angles = fresh_data['NEUTRAL_ANGLE_DEGREES']
        if hasattr(neutral_angles, 'tolist'):
            neutral_angles = neutral_angles.tolist()
        using_defaults = False
        calibration_source = "TrotBot calibration file"
    except:
        # Fall back to global variables
        micros_per_rad = MICROS_PER_RAD
        try:
            neutral_angles = NEUTRAL_ANGLE_DEGREES.tolist()
        except:
            neutral_angles = NEUTRAL_ANGLE_DEGREES
        using_defaults = not calibration_exists
        calibration_source = "Default values"

    return {
        'calibration_file_exists': calibration_exists,
        'calibration_path': str(CALIBRATION_PATH),
        'calibration_source': calibration_source,
        'micros_per_rad': micros_per_rad,
        'neutral_angle_degrees': neutral_angles,
        'using_defaults': using_defaults
    }

# Load calibration data on module import
def _load_calibration_on_import():
    """Load calibration data during module import with error handling"""
    global MICROS_PER_RAD, NEUTRAL_ANGLE_DEGREES
    try:
        calibration_data = read_calibration()
        MICROS_PER_RAD = calibration_data['MICROS_PER_RAD']
        NEUTRAL_ANGLE_DEGREES = calibration_data['NEUTRAL_ANGLE_DEGREES']
        logging.info("✅ Calibration data loaded successfully on import")
    except Exception as e:
        logging.warning(f"⚠️ Failed to load calibration on import: {e}")
        logging.info("Using default calibration values")
        # Keep default values

# Load calibration data
_load_calibration_on_import()
