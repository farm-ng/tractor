class ODenum:
    axis_state = {
        0: 'AXIS_STATE_UNDEFINED',
        1: 'AXIS_STATE_IDLE',
        2: 'AXIS_STATE_STARTUP_SEQUENCE',
        3: 'AXIS_STATE_FULL_CALIBRATION_SEQUENCE',
        4: 'AXIS_STATE_MOTOR_CALIBRATION',
        5: 'AXIS_STATE_SENSORLESS_CONTROL',
        6: 'AXIS_STATE_ENCODER_INDEX_SEARCH',
        7: 'AXIS_STATE_ENCODER_OFFSET_CALIBRATION',
        8: 'AXIS_STATE_CLOSED_LOOP_CONTROL',
        9: 'AXIS_STATE_LOCKIN_SPIN',
        10: 'AXIS_STATE_ENCODER_DIR_FIND',
        11: 'AXIS_STATE_HOMING',
    }
    axis_state_r = {v: k for k, v in axis_state.items()}

    motor_type = {
        0: 'MOTOR_TYPE_HIGH_CURRENT',
        1: 'MOTOR_TYPE_LOW_CURRENT',
        2: 'MOTOR_TYPE_GIMBAL',
    }
    motor_type_r = {v: k for k, v in motor_type.items()}

    control_mode = {
        0: 'CTRL_MODE_VOLTAGE_CONTROL',
        1: 'CTRL_MODE_CURRENT_CONTROL',
        2: 'CTRL_MODE_VELOCITY_CONTROL',
        3: 'CTRL_MODE_POSITION_CONTROL',
    }
    control_mode_r = {v: k for k, v in control_mode.items()}

    input_mode = {
        0: 'INPUT_MODE_INACTIVE',
        1: 'INPUT_MODE_PASSTHROUGH',
        2: 'INPUT_MODE_VEL_RAMP',
        3: 'INPUT_MODE_POS_FILTER',
        4: 'INPUT_MODE_MIX_CHANNELS',
        5: 'INPUT_MODE_TRAP_TRAJ',
        6: 'INPUT_MODE_CURRENT_RAMP',
        7: 'INPUT_MODE_MIRROR',
    }
    input_mode_r = {v: k for k, v in input_mode.items()}

    encoder_mode = {
        0x00: 'ENCODER_MODE_INCREMENTAL',
        0x01: 'ENCODER_MODE_HALL',
        0x02: 'ENCODER_MODE_SINCOS',
        0x100: 'ENCODER_MODE_SPI_ABS_CUI',
        0x101: 'ENCODER_MODE_SPI_ABS_AMS',
    }
    encoder_mode_r = {v: k for k, v in encoder_mode.items()}


class ODerror(ODenum):
    def get_errors(enum, value):
        result = list()
        for bit_n in range(0, 32):
            bit = value & (0x01 << bit_n)
            if(bit in enum):
                if(bit > 0):
                    result.append(enum[bit])
        return result

    axis_error = {
        0x00: 'ERROR_NONE',
        0x01: 'ERROR_INVALID_STATE',  # <! an invalid state was requested
        0x02: 'ERROR_DC_BUS_UNDER_VOLTAGE',
        0x04: 'ERROR_DC_BUS_OVER_VOLTAGE',
        0x08: 'ERROR_CURRENT_MEASUREMENT_TIMEOUT',
        0x10: 'ERROR_BRAKE_RESISTOR_DISARMED',  # <! the brake resistor was unexpectedly disarmed
        0x20: 'ERROR_MOTOR_DISARMED',  # <! the motor was unexpectedly disarmed
        0x40: 'ERROR_MOTOR_FAILED',  # Go to motor.hpp for information, check odrvX.axisX.motor.error for error value
        0x80: 'ERROR_SENSORLESS_ESTIMATOR_FAILED',
        0x100: 'ERROR_ENCODER_FAILED',  # Go to encoder.hpp for information, check odrvX.axisX.encoder.error for error value
        0x200: 'ERROR_CONTROLLER_FAILED',
        0x400: 'ERROR_POS_CTRL_DURING_SENSORLESS',
        0x800: 'ERROR_WATCHDOG_TIMER_EXPIRED',
        0x1000: 'ERROR_MIN_ENDSTOP_PRESSED',
        0x2000: 'ERROR_MAX_ENDSTOP_PRESSED',
        0x4000: 'ERROR_ESTOP_REQUESTED',
        0x8000: 'ERROR_DC_BUS_UNDER_CURRENT',
        0x10000: 'ERROR_DC_BUS_OVER_CURRENT',
        0x20000: 'ERROR_HOMING_WITHOUT_ENDSTOP',
    }

    motor_error = {
        0x0000: 'ERROR_NONE',
        0x0001: 'ERROR_PHASE_RESISTANCE_OUT_OF_RANGE',
        0x0002: 'ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE',
        0x0004: 'ERROR_ADC_FAILED',
        0x0008: 'ERROR_DRV_FAULT',
        0x0010: 'ERROR_CONTROL_DEADLINE_MISSED',
        0x0020: 'ERROR_NOT_IMPLEMENTED_MOTOR_TYPE',
        0x0040: 'ERROR_BRAKE_CURRENT_OUT_OF_RANGE',
        0x0080: 'ERROR_MODULATION_MAGNITUDE',
        0x0100: 'ERROR_BRAKE_DEADTIME_VIOLATION',
        0x0200: 'ERROR_UNEXPECTED_TIMER_CALLBACK',
        0x0400: 'ERROR_CURRENT_SENSE_SATURATION',
        0x8000: 'ERROR_CURRENT_LIMIT_VIOLATION',
    }

    encoder_error = {
        0x000: 'ERROR_NONE',
        0x001: 'ERROR_UNSTABLE_GAIN',
        0x002: 'ERROR_CPR_POLEPAIRS_MISMATCH',
        0x004: 'ERROR_NO_RESPONSE',
        0x008: 'ERROR_UNSUPPORTED_ENCODER_MODE',
        0x010: 'ERROR_ILLEGAL_HALL_STATE',
        0x020: 'ERROR_INDEX_NOT_FOUND_YET',
        0x040: 'ERROR_ABS_SPI_TIMEOUT',
        0x080: 'ERROR_ABS_SPI_COM_FAIL',
        0x100: 'ERROR_ABS_SPI_NOT_READY',
    }

    controller_error = {
        0x00: 'ERROR_NONE',
        0x01: 'ERROR_OVERSPEED',
        0x02: 'ERROR_INVALID_INPUT_MODE',
        0x04: 'ERROR_UNSTABLE_GAIN',
        0x08: 'ERROR_INVALID_MIRROR_AXIS',
    }

    sensorless_error = {
        0x00: 'ERROR_NONE',
        0x01: 'ERROR_UNSTABLE_GAIN',
    }
