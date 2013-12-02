package mavlink

type newMessageFunc func() Message

type Message interface {
        ID() uint8
        Size() uint8
}

var messageFactory = map[uint8]newMessageFunc{
	0: func() Message { return new(Heartbeat) },
	1: func() Message { return new(SysStatus) },
	2: func() Message { return new(SystemTime) },
	4: func() Message { return new(Ping) },
	5: func() Message { return new(ChangeOperatorControl) },
	6: func() Message { return new(ChangeOperatorControlAck) },
	7: func() Message { return new(AuthKey) },
	11: func() Message { return new(SetMode) },
	20: func() Message { return new(ParamRequestRead) },
	21: func() Message { return new(ParamRequestList) },
	22: func() Message { return new(ParamValue) },
	23: func() Message { return new(ParamSet) },
	24: func() Message { return new(GpsRawInt) },
	25: func() Message { return new(GpsStatus) },
	26: func() Message { return new(ScaledImu) },
	27: func() Message { return new(RawImu) },
	28: func() Message { return new(RawPressure) },
	29: func() Message { return new(ScaledPressure) },
	30: func() Message { return new(Attitude) },
	31: func() Message { return new(AttitudeQuaternion) },
	32: func() Message { return new(LocalPositionNed) },
	33: func() Message { return new(GlobalPositionInt) },
	34: func() Message { return new(RcChannelsScaled) },
	35: func() Message { return new(RcChannelsRaw) },
	36: func() Message { return new(ServoOutputRaw) },
	37: func() Message { return new(MissionRequestPartialList) },
	38: func() Message { return new(MissionWritePartialList) },
	39: func() Message { return new(MissionItem) },
	40: func() Message { return new(MissionRequest) },
	41: func() Message { return new(MissionSetCurrent) },
	42: func() Message { return new(MissionCurrent) },
	43: func() Message { return new(MissionRequestList) },
	44: func() Message { return new(MissionCount) },
	45: func() Message { return new(MissionClearAll) },
	46: func() Message { return new(MissionItemReached) },
	47: func() Message { return new(MissionAck) },
	48: func() Message { return new(SetGpsGlobalOrigin) },
	49: func() Message { return new(GpsGlobalOrigin) },
	50: func() Message { return new(SetLocalPositionSetpoint) },
	51: func() Message { return new(LocalPositionSetpoint) },
	52: func() Message { return new(GlobalPositionSetpointInt) },
	53: func() Message { return new(SetGlobalPositionSetpointInt) },
	54: func() Message { return new(SafetySetAllowedArea) },
	55: func() Message { return new(SafetyAllowedArea) },
	56: func() Message { return new(SetRollPitchYawThrust) },
	57: func() Message { return new(SetRollPitchYawSpeedThrust) },
	58: func() Message { return new(RollPitchYawThrustSetpoint) },
	59: func() Message { return new(RollPitchYawSpeedThrustSetpoint) },
	60: func() Message { return new(SetQuadMotorsSetpoint) },
	61: func() Message { return new(SetQuadSwarmRollPitchYawThrust) },
	62: func() Message { return new(NavControllerOutput) },
	63: func() Message { return new(SetQuadSwarmLedRollPitchYawThrust) },
	64: func() Message { return new(StateCorrection) },
	66: func() Message { return new(RequestDataStream) },
	67: func() Message { return new(DataStream) },
	69: func() Message { return new(ManualControl) },
	70: func() Message { return new(RcChannelsOverride) },
	74: func() Message { return new(VfrHud) },
	76: func() Message { return new(CommandLong) },
	77: func() Message { return new(CommandAck) },
	80: func() Message { return new(RollPitchYawRatesThrustSetpoint) },
	81: func() Message { return new(ManualSetpoint) },
	89: func() Message { return new(LocalPositionNedSystemGlobalOffset) },
	90: func() Message { return new(HilState) },
	91: func() Message { return new(HilControls) },
	92: func() Message { return new(HilRcInputsRaw) },
	100: func() Message { return new(OpticalFlow) },
	101: func() Message { return new(GlobalVisionPositionEstimate) },
	102: func() Message { return new(VisionPositionEstimate) },
	103: func() Message { return new(VisionSpeedEstimate) },
	104: func() Message { return new(ViconPositionEstimate) },
	105: func() Message { return new(HighresImu) },
	106: func() Message { return new(OmnidirectionalFlow) },
	107: func() Message { return new(HilSensor) },
	108: func() Message { return new(SimState) },
	109: func() Message { return new(RadioStatus) },
	110: func() Message { return new(FileTransferStart) },
	111: func() Message { return new(FileTransferDirList) },
	112: func() Message { return new(FileTransferRes) },
	113: func() Message { return new(HilGps) },
	114: func() Message { return new(HilOpticalFlow) },
	115: func() Message { return new(HilStateQuaternion) },
	147: func() Message { return new(BatteryStatus) },
	148: func() Message { return new(Setpoint8dof) },
	149: func() Message { return new(Setpoint6dof) },
	249: func() Message { return new(MemoryVect) },
	250: func() Message { return new(DebugVect) },
	251: func() Message { return new(NamedValueFloat) },
	252: func() Message { return new(NamedValueInt) },
	253: func() Message { return new(Statustext) },
	254: func() Message { return new(Debug) },
}


// Micro air vehicle / autopilot classes. This identifies the individual model.
type MAV_AUTOPILOT byte

const (
	// Generic autopilot, full support for everything
	MAV_AUTOPILOT_GENERIC MAV_AUTOPILOT = 0
	// PIXHAWK autopilot, http://pixhawk.ethz.ch
	MAV_AUTOPILOT_PIXHAWK = 1
	// SLUGS autopilot, http://slugsuav.soe.ucsc.edu
	MAV_AUTOPILOT_SLUGS = 2
	// ArduPilotMega / ArduCopter, http://diydrones.com
	MAV_AUTOPILOT_ARDUPILOTMEGA = 3
	// OpenPilot, http://openpilot.org
	MAV_AUTOPILOT_OPENPILOT = 4
	// Generic autopilot only supporting simple waypoints
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5
	// Generic autopilot supporting waypoints and other simple navigation commands
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6
	// Generic autopilot supporting the full mission command set
	MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7
	// No valid autopilot, e.g. a GCS or other MAVLink component
	MAV_AUTOPILOT_INVALID = 8
	// PPZ UAV - http://nongnu.org/paparazzi
	MAV_AUTOPILOT_PPZ = 9
	// UAV Dev Board
	MAV_AUTOPILOT_UDB = 10
	// FlexiPilot
	MAV_AUTOPILOT_FP = 11
	// PX4 Autopilot - http://pixhawk.ethz.ch/px4/
	MAV_AUTOPILOT_PX4 = 12
	// SMACCMPilot - http://smaccmpilot.org
	MAV_AUTOPILOT_SMACCMPILOT = 13
	// AutoQuad -- http://autoquad.org
	MAV_AUTOPILOT_AUTOQUAD = 14
	// Armazila -- http://armazila.com
	MAV_AUTOPILOT_ARMAZILA = 15
	// Aerob -- http://aerob.ru
	MAV_AUTOPILOT_AEROB = 16
)

type MAV_TYPE byte

const (
	// Generic micro air vehicle.
	MAV_TYPE_GENERIC MAV_TYPE = 0
	// Fixed wing aircraft.
	MAV_TYPE_FIXED_WING = 1
	// Quadrotor
	MAV_TYPE_QUADROTOR = 2
	// Coaxial helicopter
	MAV_TYPE_COAXIAL = 3
	// Normal helicopter with tail rotor.
	MAV_TYPE_HELICOPTER = 4
	// Ground installation
	MAV_TYPE_ANTENNA_TRACKER = 5
	// Operator control unit / ground control station
	MAV_TYPE_GCS = 6
	// Airship, controlled
	MAV_TYPE_AIRSHIP = 7
	// Free balloon, uncontrolled
	MAV_TYPE_FREE_BALLOON = 8
	// Rocket
	MAV_TYPE_ROCKET = 9
	// Ground rover
	MAV_TYPE_GROUND_ROVER = 10
	// Surface vessel, boat, ship
	MAV_TYPE_SURFACE_BOAT = 11
	// Submarine
	MAV_TYPE_SUBMARINE = 12
	// Hexarotor
	MAV_TYPE_HEXAROTOR = 13
	// Octorotor
	MAV_TYPE_OCTOROTOR = 14
	// Octorotor
	MAV_TYPE_TRICOPTER = 15
	// Flapping wing
	MAV_TYPE_FLAPPING_WING = 16
	// Flapping wing
	MAV_TYPE_KITE = 17
)

// These flags encode the MAV mode.
type MAV_MODE_FLAG byte

const (
	// 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly.
	MAV_MODE_FLAG_SAFETY_ARMED MAV_MODE_FLAG = 128
	// 0b01000000 remote control input is enabled.
	MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64
	// 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
	MAV_MODE_FLAG_HIL_ENABLED = 32
	// 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
	MAV_MODE_FLAG_STABILIZE_ENABLED = 16
	// 0b00001000 guided mode enabled, system flies MISSIONs / mission items.
	MAV_MODE_FLAG_GUIDED_ENABLED = 8
	// 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
	MAV_MODE_FLAG_AUTO_ENABLED = 4
	// 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
	MAV_MODE_FLAG_TEST_ENABLED = 2
	// 0b00000001 Reserved for future use.
	MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
)

// These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
type MAV_MODE_FLAG_DECODE_POSITION byte

const (
	// First bit:  10000000
	MAV_MODE_FLAG_DECODE_POSITION_SAFETY MAV_MODE_FLAG_DECODE_POSITION = 128
	// Second bit: 01000000
	MAV_MODE_FLAG_DECODE_POSITION_MANUAL = 64
	// Third bit:  00100000
	MAV_MODE_FLAG_DECODE_POSITION_HIL = 32
	// Fourth bit: 00010000
	MAV_MODE_FLAG_DECODE_POSITION_STABILIZE = 16
	// Fifth bit:  00001000
	MAV_MODE_FLAG_DECODE_POSITION_GUIDED = 8
	// Sixt bit:   00000100
	MAV_MODE_FLAG_DECODE_POSITION_AUTO = 4
	// Seventh bit: 00000010
	MAV_MODE_FLAG_DECODE_POSITION_TEST = 2
	// Eighth bit: 00000001
	MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 1
)

// Override command, pauses current mission execution and moves immediately to a position
type MAV_GOTO byte

const (
	// Hold at the current position.
	MAV_GOTO_DO_HOLD MAV_GOTO = 0
	// Continue with the next item in mission execution.
	MAV_GOTO_DO_CONTINUE = 1
	// Hold at the current position of the system
	MAV_GOTO_HOLD_AT_CURRENT_POSITION = 2
	// Hold at the position specified in the parameters of the DO_HOLD action
	MAV_GOTO_HOLD_AT_SPECIFIED_POSITION = 3
)

// These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
//                simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.
type MAV_MODE byte

const (
	// System is not ready to fly, booting, calibrating, etc. No flag is set.
	MAV_MODE_PREFLIGHT MAV_MODE = 0
	// System is allowed to be active, under assisted RC control.
	MAV_MODE_STABILIZE_DISARMED = 80
	// System is allowed to be active, under assisted RC control.
	MAV_MODE_STABILIZE_ARMED = 208
	// System is allowed to be active, under manual (RC) control, no stabilization
	MAV_MODE_MANUAL_DISARMED = 64
	// System is allowed to be active, under manual (RC) control, no stabilization
	MAV_MODE_MANUAL_ARMED = 192
	// System is allowed to be active, under autonomous control, manual setpoint
	MAV_MODE_GUIDED_DISARMED = 88
	// System is allowed to be active, under autonomous control, manual setpoint
	MAV_MODE_GUIDED_ARMED = 216
	// System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs)
	MAV_MODE_AUTO_DISARMED = 92
	// System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs)
	MAV_MODE_AUTO_ARMED = 220
	// UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
	MAV_MODE_TEST_DISARMED = 66
	// UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
	MAV_MODE_TEST_ARMED = 194
)

type MAV_STATE byte

const (
	// Uninitialized system, state is unknown.
	MAV_STATE_UNINIT MAV_STATE = 0
	// System is booting up.
	MAV_STATE_BOOT
	// System is calibrating and not flight-ready.
	MAV_STATE_CALIBRATING
	// System is grounded and on standby. It can be launched any time.
	MAV_STATE_STANDBY
	// System is active and might be already airborne. Motors are engaged.
	MAV_STATE_ACTIVE
	// System is in a non-normal flight mode. It can however still navigate.
	MAV_STATE_CRITICAL
	// System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
	MAV_STATE_EMERGENCY
	// System just initialized its power-down sequence, will shut down now.
	MAV_STATE_POWEROFF
)

type MAV_COMPONENT byte

const (
	MAV_COMP_ID_ALL MAV_COMPONENT = 0
	MAV_COMP_ID_GPS = 220
	MAV_COMP_ID_MISSIONPLANNER = 190
	MAV_COMP_ID_PATHPLANNER = 195
	MAV_COMP_ID_MAPPER = 180
	MAV_COMP_ID_CAMERA = 100
	MAV_COMP_ID_IMU = 200
	MAV_COMP_ID_IMU_2 = 201
	MAV_COMP_ID_IMU_3 = 202
	MAV_COMP_ID_UDP_BRIDGE = 240
	MAV_COMP_ID_UART_BRIDGE = 241
	MAV_COMP_ID_SYSTEM_CONTROL = 250
	MAV_COMP_ID_SERVO1 = 140
	MAV_COMP_ID_SERVO2 = 141
	MAV_COMP_ID_SERVO3 = 142
	MAV_COMP_ID_SERVO4 = 143
	MAV_COMP_ID_SERVO5 = 144
	MAV_COMP_ID_SERVO6 = 145
	MAV_COMP_ID_SERVO7 = 146
	MAV_COMP_ID_SERVO8 = 147
	MAV_COMP_ID_SERVO9 = 148
	MAV_COMP_ID_SERVO10 = 149
	MAV_COMP_ID_SERVO11 = 150
	MAV_COMP_ID_SERVO12 = 151
	MAV_COMP_ID_SERVO13 = 152
	MAV_COMP_ID_SERVO14 = 153
)

// These encode the sensors whose status is sent as part of the SYS_STATUS message.
type MAV_SYS_STATUS_SENSOR byte

const (
	// 0x01 3D gyro
	MAV_SYS_STATUS_SENSOR_3D_GYRO MAV_SYS_STATUS_SENSOR = 1
	// 0x02 3D accelerometer
	MAV_SYS_STATUS_SENSOR_3D_ACCEL = 2
	// 0x04 3D magnetometer
	MAV_SYS_STATUS_SENSOR_3D_MAG = 4
	// 0x08 absolute pressure
	MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 8
	// 0x10 differential pressure
	MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE = 16
	// 0x20 GPS
	MAV_SYS_STATUS_SENSOR_GPS = 32
	// 0x40 optical flow
	MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW = 64
	// 0x80 computer vision position
	MAV_SYS_STATUS_SENSOR_VISION_POSITION = 128
	// 0x100 laser based position
	MAV_SYS_STATUS_SENSOR_LASER_POSITION = 256
	// 0x200 external ground truth (Vicon or Leica)
	MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH = 512
	// 0x400 3D angular rate control
	MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL = 1024
	// 0x800 attitude stabilization
	MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048
	// 0x1000 yaw position
	MAV_SYS_STATUS_SENSOR_YAW_POSITION = 4096
	// 0x2000 z/altitude control
	MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL = 8192
	// 0x4000 x/y position control
	MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL = 16384
	// 0x8000 motor outputs / control
	MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = 32768
	// 0x10000 rc receiver
	MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 65536
)

type MAV_FRAME byte

const (
	// Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL)
	MAV_FRAME_GLOBAL MAV_FRAME = 0
	// Local coordinate frame, Z-up (x: north, y: east, z: down).
	MAV_FRAME_LOCAL_NED = 1
	// NOT a coordinate frame, indicates a mission command.
	MAV_FRAME_MISSION = 2
	// Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
	MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
	// Local coordinate frame, Z-down (x: east, y: north, z: up)
	MAV_FRAME_LOCAL_ENU = 4
)

type MAVLINK_DATA_STREAM_TYPE byte

const (
	MAVLINK_DATA_STREAM_IMG_JPEG MAVLINK_DATA_STREAM_TYPE = iota
	MAVLINK_DATA_STREAM_IMG_BMP
	MAVLINK_DATA_STREAM_IMG_RAW8U
	MAVLINK_DATA_STREAM_IMG_RAW32U
	MAVLINK_DATA_STREAM_IMG_PGM
	MAVLINK_DATA_STREAM_IMG_PNG
)

// Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data.
type MAV_CMD byte

const (
	// Navigate to MISSION.
	MAV_CMD_NAV_WAYPOINT MAV_CMD = 16
	// Loiter around this MISSION an unlimited amount of time
	MAV_CMD_NAV_LOITER_UNLIM = 17
	// Loiter around this MISSION for X turns
	MAV_CMD_NAV_LOITER_TURNS = 18
	// Loiter around this MISSION for X seconds
	MAV_CMD_NAV_LOITER_TIME = 19
	// Return to launch location
	MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
	// Land at location
	MAV_CMD_NAV_LAND = 21
	// Takeoff from ground / hand
	MAV_CMD_NAV_TAKEOFF = 22
	// Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	MAV_CMD_NAV_ROI = 80
	// Control autonomous path planning on the MAV.
	MAV_CMD_NAV_PATHPLANNING = 81
	// NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
	MAV_CMD_NAV_LAST = 95
	// Delay mission state machine.
	MAV_CMD_CONDITION_DELAY = 112
	// Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
	MAV_CMD_CONDITION_CHANGE_ALT = 113
	// Delay mission state machine until within desired distance of next NAV point.
	MAV_CMD_CONDITION_DISTANCE = 114
	// Reach a certain target angle.
	MAV_CMD_CONDITION_YAW = 115
	// NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
	MAV_CMD_CONDITION_LAST = 159
	// Set system mode.
	MAV_CMD_DO_SET_MODE = 176
	// Jump to the desired command in the mission list.  Repeat this action only the specified number of times
	MAV_CMD_DO_JUMP = 177
	// Change speed and/or throttle set points.
	MAV_CMD_DO_CHANGE_SPEED = 178
	// Changes the home location either to the current location or a specified location.
	MAV_CMD_DO_SET_HOME = 179
	// Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.
	MAV_CMD_DO_SET_PARAMETER = 180
	// Set a relay to a condition.
	MAV_CMD_DO_SET_RELAY = 181
	// Cycle a relay on and off for a desired number of cyles with a desired period.
	MAV_CMD_DO_REPEAT_RELAY = 182
	// Set a servo to a desired PWM value.
	MAV_CMD_DO_SET_SERVO = 183
	// Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
	MAV_CMD_DO_REPEAT_SERVO = 184
	// Control onboard camera system.
	MAV_CMD_DO_CONTROL_VIDEO = 200
	// Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	MAV_CMD_DO_SET_ROI = 201
	// NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
	MAV_CMD_DO_LAST = 240
	// Trigger calibration. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_CALIBRATION = 241
	// Set sensor offsets. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242
	// Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_STORAGE = 245
	// Request the reboot or shutdown of system components.
	MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
	// Hold / continue the current action
	MAV_CMD_OVERRIDE_GOTO = 252
	// start running a mission
	MAV_CMD_MISSION_START = 300
	// Arms / Disarms a component
	MAV_CMD_COMPONENT_ARM_DISARM = 400
	// Starts receiver pairing
	MAV_CMD_START_RX_PAIR = 500
)

// Data stream IDs. A data stream is not a fixed set of messages, but rather a
//      recommendation to the autopilot software. Individual autopilots may or may not obey
//      the recommended messages.
type MAV_DATA_STREAM byte

const (
	// Enable all data streams
	MAV_DATA_STREAM_ALL MAV_DATA_STREAM = 0
	// Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
	MAV_DATA_STREAM_RAW_SENSORS = 1
	// Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
	MAV_DATA_STREAM_EXTENDED_STATUS = 2
	// Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
	MAV_DATA_STREAM_RC_CHANNELS = 3
	// Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
	MAV_DATA_STREAM_RAW_CONTROLLER = 4
	// Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
	MAV_DATA_STREAM_POSITION = 6
	// Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA1 = 10
	// Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA2 = 11
	// Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA3 = 12
)

//  The ROI (region of interest) for the vehicle. This can be
//                 be used by the vehicle for camera/vehicle attitude alignment (see
//                 MAV_CMD_NAV_ROI).
type MAV_ROI byte

const (
	// No region of interest.
	MAV_ROI_NONE MAV_ROI = 0
	// Point toward next MISSION.
	MAV_ROI_WPNEXT = 1
	// Point toward given MISSION.
	MAV_ROI_WPINDEX = 2
	// Point toward fixed location.
	MAV_ROI_LOCATION = 3
	// Point toward of given id.
	MAV_ROI_TARGET = 4
)

// ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
type MAV_CMD_ACK byte

const (
	// Command / mission item is ok.
	MAV_CMD_ACK_OK MAV_CMD_ACK = iota
	// Generic error message if none of the other reasons fails or if no detailed error reporting is implemented.
	MAV_CMD_ACK_ERR_FAIL
	// The system is refusing to accept this command from this source / communication partner.
	MAV_CMD_ACK_ERR_ACCESS_DENIED
	// Command or mission item is not supported, other commands would be accepted.
	MAV_CMD_ACK_ERR_NOT_SUPPORTED
	// The coordinate frame of this command / mission item is not supported.
	MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED
	// The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible.
	MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE
	// The X or latitude value is out of range.
	MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE
	// The Y or longitude value is out of range.
	MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE
	// The Z or altitude value is out of range.
	MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE
)

// Specifies the datatype of a MAVLink parameter.
type MAV_PARAM_TYPE byte

const (
	// 8-bit unsigned integer
	MAV_PARAM_TYPE_UINT8 MAV_PARAM_TYPE = 1
	// 8-bit signed integer
	MAV_PARAM_TYPE_INT8 = 2
	// 16-bit unsigned integer
	MAV_PARAM_TYPE_UINT16 = 3
	// 16-bit signed integer
	MAV_PARAM_TYPE_INT16 = 4
	// 32-bit unsigned integer
	MAV_PARAM_TYPE_UINT32 = 5
	// 32-bit signed integer
	MAV_PARAM_TYPE_INT32 = 6
	// 64-bit unsigned integer
	MAV_PARAM_TYPE_UINT64 = 7
	// 64-bit signed integer
	MAV_PARAM_TYPE_INT64 = 8
	// 32-bit floating-point
	MAV_PARAM_TYPE_REAL32 = 9
	// 64-bit floating-point
	MAV_PARAM_TYPE_REAL64 = 10
)

// result from a mavlink command
type MAV_RESULT byte

const (
	// Command ACCEPTED and EXECUTED
	MAV_RESULT_ACCEPTED MAV_RESULT = 0
	// Command TEMPORARY REJECTED/DENIED
	MAV_RESULT_TEMPORARILY_REJECTED = 1
	// Command PERMANENTLY DENIED
	MAV_RESULT_DENIED = 2
	// Command UNKNOWN/UNSUPPORTED
	MAV_RESULT_UNSUPPORTED = 3
	// Command executed, but failed
	MAV_RESULT_FAILED = 4
)

// result in a mavlink mission ack
type MAV_MISSION_RESULT byte

const (
	// mission accepted OK
	MAV_MISSION_ACCEPTED MAV_MISSION_RESULT = 0
	// generic error / not accepting mission commands at all right now
	MAV_MISSION_ERROR = 1
	// coordinate frame is not supported
	MAV_MISSION_UNSUPPORTED_FRAME = 2
	// command is not supported
	MAV_MISSION_UNSUPPORTED = 3
	// mission item exceeds storage space
	MAV_MISSION_NO_SPACE = 4
	// one of the parameters has an invalid value
	MAV_MISSION_INVALID = 5
	// param1 has an invalid value
	MAV_MISSION_INVALID_PARAM1 = 6
	// param2 has an invalid value
	MAV_MISSION_INVALID_PARAM2 = 7
	// param3 has an invalid value
	MAV_MISSION_INVALID_PARAM3 = 8
	// param4 has an invalid value
	MAV_MISSION_INVALID_PARAM4 = 9
	// x/param5 has an invalid value
	MAV_MISSION_INVALID_PARAM5_X = 10
	// y/param6 has an invalid value
	MAV_MISSION_INVALID_PARAM6_Y = 11
	// param7 has an invalid value
	MAV_MISSION_INVALID_PARAM7 = 12
	// received waypoint out of sequence
	MAV_MISSION_INVALID_SEQUENCE = 13
	// not accepting any mission commands from this communication partner
	MAV_MISSION_DENIED = 14
)

// Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
type MAV_SEVERITY byte

const (
	// System is unusable. This is a "panic" condition.
	MAV_SEVERITY_EMERGENCY MAV_SEVERITY = 0
	// Action should be taken immediately. Indicates error in non-critical systems.
	MAV_SEVERITY_ALERT = 1
	// Action must be taken immediately. Indicates failure in a primary system.
	MAV_SEVERITY_CRITICAL = 2
	// Indicates an error in secondary/redundant systems.
	MAV_SEVERITY_ERROR = 3
	// Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
	MAV_SEVERITY_WARNING = 4
	// An unusual event has occured, though not an error condition. This should be investigated for the root cause.
	MAV_SEVERITY_NOTICE = 5
	// Normal operational messages. Useful for logging. No action is required for these messages.
	MAV_SEVERITY_INFO = 6
	// Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
	MAV_SEVERITY_DEBUG = 7
)



// The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot).
type Heartbeat struct {
	CustomMode	uint32	// A bitfield for use for autopilot-specific flags.
	Type	uint8	// Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
	Autopilot	uint8	// Autopilot type / class. defined in MAV_AUTOPILOT ENUM
	BaseMode	uint8	// System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
	SystemStatus	uint8	// System status flag, see MAV_STATE ENUM
	MavlinkVersion	uint8	// MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
}

func(self *Heartbeat) ID() uint8 {
	return 0
}

func(self *Heartbeat) Size() uint8 {
	return 9
}

// The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows wether the system is currently active or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occured it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
type SysStatus struct {
	OnboardControlSensorsPresent	uint32	// Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	OnboardControlSensorsEnabled	uint32	// Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	OnboardControlSensorsHealth	uint32	// Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	Load	uint16	// Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
	VoltageBattery	uint16	// Battery voltage, in millivolts (1 = 1 millivolt)
	CurrentBattery	int16	// Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
	DropRateComm	uint16	// Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	ErrorsComm	uint16	// Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	ErrorsCount1	uint16	// Autopilot-specific errors
	ErrorsCount2	uint16	// Autopilot-specific errors
	ErrorsCount3	uint16	// Autopilot-specific errors
	ErrorsCount4	uint16	// Autopilot-specific errors
	BatteryRemaining	int8	// Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
}

func(self *SysStatus) ID() uint8 {
	return 1
}

func(self *SysStatus) Size() uint8 {
	return 31
}

// The system time is the time of the master clock, typically the computer clock of the main onboard computer.
type SystemTime struct {
	TimeUnixUsec	uint64	// Timestamp of the master clock in microseconds since UNIX epoch.
	TimeBootMs	uint32	// Timestamp of the component clock since boot time in milliseconds.
}

func(self *SystemTime) ID() uint8 {
	return 2
}

func(self *SystemTime) Size() uint8 {
	return 12
}

// A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections.
type Ping struct {
	TimeUsec	uint64	// Unix timestamp in microseconds
	Seq	uint32	// PING sequence
	TargetSystem	uint8	// 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
	TargetComponent	uint8	// 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
}

func(self *Ping) ID() uint8 {
	return 4
}

func(self *Ping) Size() uint8 {
	return 14
}

// Request to control this MAV
type ChangeOperatorControl struct {
	TargetSystem	uint8	// System the GCS requests control for
	ControlRequest	uint8	// 0: request control of this MAV, 1: Release control of this MAV
	Version	uint8	// 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
	Passkey	[25]byte	// Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
}

func(self *ChangeOperatorControl) ID() uint8 {
	return 5
}

func(self *ChangeOperatorControl) Size() uint8 {
	return 28
}

// Accept / deny control of this MAV
type ChangeOperatorControlAck struct {
	GcsSystemId	uint8	// ID of the GCS this message 
	ControlRequest	uint8	// 0: request control of this MAV, 1: Release control of this MAV
	Ack	uint8	// 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
}

func(self *ChangeOperatorControlAck) ID() uint8 {
	return 6
}

func(self *ChangeOperatorControlAck) Size() uint8 {
	return 3
}

// Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.
type AuthKey struct {
	Key	[32]byte	// key
}

func(self *AuthKey) ID() uint8 {
	return 7
}

func(self *AuthKey) Size() uint8 {
	return 32
}

// Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.
type SetMode struct {
	CustomMode	uint32	// The new autopilot-specific mode. This field can be ignored by an autopilot.
	TargetSystem	uint8	// The system setting the mode
	BaseMode	uint8	// The new base mode
}

func(self *SetMode) ID() uint8 {
	return 11
}

func(self *SetMode) Size() uint8 {
	return 6
}

// Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also http://qgroundcontrol.org/parameter_interface for a full documentation of QGroundControl and IMU code.
type ParamRequestRead struct {
	ParamIndex	int16	// Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
	ParamId	[16]byte	// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
}

func(self *ParamRequestRead) ID() uint8 {
	return 20
}

func(self *ParamRequestRead) Size() uint8 {
	return 20
}

// Request all parameters of this component. After his request, all parameters are emitted.
type ParamRequestList struct {
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
}

func(self *ParamRequestList) ID() uint8 {
	return 21
}

func(self *ParamRequestList) Size() uint8 {
	return 2
}

// Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout.
type ParamValue struct {
	ParamValue	float32	// Onboard parameter value
	ParamCount	uint16	// Total number of onboard parameters
	ParamIndex	uint16	// Index of this onboard parameter
	ParamId	[16]byte	// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType	uint8	// Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
}

func(self *ParamValue) ID() uint8 {
	return 22
}

func(self *ParamValue) Size() uint8 {
	return 25
}

// Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component should acknowledge the new parameter value by sending a param_value message to all communication partners. This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message.
type ParamSet struct {
	ParamValue	float32	// Onboard parameter value
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
	ParamId	[16]byte	// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType	uint8	// Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
}

func(self *ParamSet) ID() uint8 {
	return 23
}

func(self *ParamSet) Size() uint8 {
	return 23
}

// The global position, as returned by the Global Positioning System (GPS). This is
//                 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
type GpsRawInt struct {
	TimeUsec	uint64	// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Lat	int32	// Latitude (WGS84), in degrees * 1E7
	Lon	int32	// Longitude (WGS84), in degrees * 1E7
	Alt	int32	// Altitude (WGS84), in meters * 1000 (positive for up)
	Eph	uint16	// GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	Epv	uint16	// GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	Vel	uint16	// GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
	Cog	uint16	// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	FixType	uint8	// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
	SatellitesVisible	uint8	// Number of satellites visible. If unknown, set to 255
}

func(self *GpsRawInt) ID() uint8 {
	return 24
}

func(self *GpsRawInt) Size() uint8 {
	return 30
}

// The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
type GpsStatus struct {
	SatellitesVisible	uint8	// Number of satellites visible
	SatellitePrn	[20]uint8	// Global satellite ID
	SatelliteUsed	[20]uint8	// 0: Satellite not used, 1: used for localization
	SatelliteElevation	[20]uint8	// Elevation (0: right on top of receiver, 90: on the horizon) of satellite
	SatelliteAzimuth	[20]uint8	// Direction of satellite, 0: 0 deg, 255: 360 deg.
	SatelliteSnr	[20]uint8	// Signal to noise ratio of satellite
}

func(self *GpsStatus) ID() uint8 {
	return 25
}

func(self *GpsStatus) Size() uint8 {
	return 101
}

// The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu struct {
	TimeBootMs	uint32	// Timestamp (milliseconds since system boot)
	Xacc	int16	// X acceleration (mg)
	Yacc	int16	// Y acceleration (mg)
	Zacc	int16	// Z acceleration (mg)
	Xgyro	int16	// Angular speed around X axis (millirad /sec)
	Ygyro	int16	// Angular speed around Y axis (millirad /sec)
	Zgyro	int16	// Angular speed around Z axis (millirad /sec)
	Xmag	int16	// X Magnetic field (milli tesla)
	Ymag	int16	// Y Magnetic field (milli tesla)
	Zmag	int16	// Z Magnetic field (milli tesla)
}

func(self *ScaledImu) ID() uint8 {
	return 26
}

func(self *ScaledImu) Size() uint8 {
	return 22
}

// The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw values without any scaling to allow data capture and system debugging.
type RawImu struct {
	TimeUsec	uint64	// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Xacc	int16	// X acceleration (raw)
	Yacc	int16	// Y acceleration (raw)
	Zacc	int16	// Z acceleration (raw)
	Xgyro	int16	// Angular speed around X axis (raw)
	Ygyro	int16	// Angular speed around Y axis (raw)
	Zgyro	int16	// Angular speed around Z axis (raw)
	Xmag	int16	// X Magnetic field (raw)
	Ymag	int16	// Y Magnetic field (raw)
	Zmag	int16	// Z Magnetic field (raw)
}

func(self *RawImu) ID() uint8 {
	return 27
}

func(self *RawImu) Size() uint8 {
	return 26
}

// The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
type RawPressure struct {
	TimeUsec	uint64	// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	PressAbs	int16	// Absolute pressure (raw)
	PressDiff1	int16	// Differential pressure 1 (raw)
	PressDiff2	int16	// Differential pressure 2 (raw)
	Temperature	int16	// Raw Temperature measurement (raw)
}

func(self *RawPressure) ID() uint8 {
	return 28
}

func(self *RawPressure) Size() uint8 {
	return 16
}

// The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
type ScaledPressure struct {
	TimeBootMs	uint32	// Timestamp (milliseconds since system boot)
	PressAbs	float32	// Absolute pressure (hectopascal)
	PressDiff	float32	// Differential pressure 1 (hectopascal)
	Temperature	int16	// Temperature measurement (0.01 degrees celsius)
}

func(self *ScaledPressure) ID() uint8 {
	return 29
}

func(self *ScaledPressure) Size() uint8 {
	return 14
}

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
type Attitude struct {
	TimeBootMs	uint32	// Timestamp (milliseconds since system boot)
	Roll	float32	// Roll angle (rad, -pi..+pi)
	Pitch	float32	// Pitch angle (rad, -pi..+pi)
	Yaw	float32	// Yaw angle (rad, -pi..+pi)
	Rollspeed	float32	// Roll angular speed (rad/s)
	Pitchspeed	float32	// Pitch angular speed (rad/s)
	Yawspeed	float32	// Yaw angular speed (rad/s)
}

func(self *Attitude) ID() uint8 {
	return 30
}

func(self *Attitude) Size() uint8 {
	return 28
}

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
type AttitudeQuaternion struct {
	TimeBootMs	uint32	// Timestamp (milliseconds since system boot)
	Q1	float32	// Quaternion component 1
	Q2	float32	// Quaternion component 2
	Q3	float32	// Quaternion component 3
	Q4	float32	// Quaternion component 4
	Rollspeed	float32	// Roll angular speed (rad/s)
	Pitchspeed	float32	// Pitch angular speed (rad/s)
	Yawspeed	float32	// Yaw angular speed (rad/s)
}

func(self *AttitudeQuaternion) ID() uint8 {
	return 31
}

func(self *AttitudeQuaternion) Size() uint8 {
	return 32
}

// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNed struct {
	TimeBootMs	uint32	// Timestamp (milliseconds since system boot)
	X	float32	// X Position
	Y	float32	// Y Position
	Z	float32	// Z Position
	Vx	float32	// X Speed
	Vy	float32	// Y Speed
	Vz	float32	// Z Speed
}

func(self *LocalPositionNed) ID() uint8 {
	return 32
}

func(self *LocalPositionNed) Size() uint8 {
	return 28
}

// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
//                is designed as scaled integer message since the resolution of float is not sufficient.
type GlobalPositionInt struct {
	TimeBootMs	uint32	// Timestamp (milliseconds since system boot)
	Lat	int32	// Latitude, expressed as * 1E7
	Lon	int32	// Longitude, expressed as * 1E7
	Alt	int32	// Altitude in meters, expressed as * 1000 (millimeters), above MSL
	RelativeAlt	int32	// Altitude above ground in meters, expressed as * 1000 (millimeters)
	Vx	int16	// Ground X Speed (Latitude), expressed as m/s * 100
	Vy	int16	// Ground Y Speed (Longitude), expressed as m/s * 100
	Vz	int16	// Ground Z Speed (Altitude), expressed as m/s * 100
	Hdg	uint16	// Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
}

func(self *GlobalPositionInt) ID() uint8 {
	return 33
}

func(self *GlobalPositionInt) Size() uint8 {
	return 28
}

// The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX.
type RcChannelsScaled struct {
	TimeBootMs	uint32	// Timestamp (milliseconds since system boot)
	Chan1Scaled	int16	// RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan2Scaled	int16	// RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan3Scaled	int16	// RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan4Scaled	int16	// RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan5Scaled	int16	// RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan6Scaled	int16	// RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan7Scaled	int16	// RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan8Scaled	int16	// RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Port	uint8	// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
	Rssi	uint8	// Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
}

func(self *RcChannelsScaled) ID() uint8 {
	return 34
}

func(self *RcChannelsScaled) Size() uint8 {
	return 22
}

// The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannelsRaw struct {
	TimeBootMs	uint32	// Timestamp (milliseconds since system boot)
	Chan1Raw	uint16	// RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan2Raw	uint16	// RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan3Raw	uint16	// RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan4Raw	uint16	// RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan5Raw	uint16	// RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan6Raw	uint16	// RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan7Raw	uint16	// RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan8Raw	uint16	// RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Port	uint8	// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
	Rssi	uint8	// Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
}

func(self *RcChannelsRaw) ID() uint8 {
	return 35
}

func(self *RcChannelsRaw) Size() uint8 {
	return 22
}

// The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
type ServoOutputRaw struct {
	TimeUsec	uint32	// Timestamp (microseconds since system boot)
	Servo1Raw	uint16	// Servo output 1 value, in microseconds
	Servo2Raw	uint16	// Servo output 2 value, in microseconds
	Servo3Raw	uint16	// Servo output 3 value, in microseconds
	Servo4Raw	uint16	// Servo output 4 value, in microseconds
	Servo5Raw	uint16	// Servo output 5 value, in microseconds
	Servo6Raw	uint16	// Servo output 6 value, in microseconds
	Servo7Raw	uint16	// Servo output 7 value, in microseconds
	Servo8Raw	uint16	// Servo output 8 value, in microseconds
	Port	uint8	// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
}

func(self *ServoOutputRaw) ID() uint8 {
	return 36
}

func(self *ServoOutputRaw) Size() uint8 {
	return 21
}

// Request a partial list of mission items from the system/component. http://qgroundcontrol.org/mavlink/waypoint_protocol. If start and end index are the same, just send one waypoint.
type MissionRequestPartialList struct {
	StartIndex	int16	// Start index, 0 by default
	EndIndex	int16	// End index, -1 by default (-1: send list to end). Else a valid index of the list
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
}

func(self *MissionRequestPartialList) ID() uint8 {
	return 37
}

func(self *MissionRequestPartialList) Size() uint8 {
	return 6
}

// This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
type MissionWritePartialList struct {
	StartIndex	int16	// Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
	EndIndex	int16	// End index, equal or greater than start index.
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
}

func(self *MissionWritePartialList) ID() uint8 {
	return 38
}

func(self *MissionWritePartialList) Size() uint8 {
	return 6
}

// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
type MissionItem struct {
	Param1	float32	// PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
	Param2	float32	// PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
	Param3	float32	// PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
	Param4	float32	// PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
	X	float32	// PARAM5 / local: x position, global: latitude
	Y	float32	// PARAM6 / y position: global: longitude
	Z	float32	// PARAM7 / z position: global: altitude
	Seq	uint16	// Sequence
	Command	uint16	// The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
	Frame	uint8	// The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
	Current	uint8	// false:0, true:1
	Autocontinue	uint8	// autocontinue to next wp
}

func(self *MissionItem) ID() uint8 {
	return 39
}

func(self *MissionItem) Size() uint8 {
	return 37
}

// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
type MissionRequest struct {
	Seq	uint16	// Sequence
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
}

func(self *MissionRequest) ID() uint8 {
	return 40
}

func(self *MissionRequest) Size() uint8 {
	return 4
}

// Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
type MissionSetCurrent struct {
	Seq	uint16	// Sequence
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
}

func(self *MissionSetCurrent) ID() uint8 {
	return 41
}

func(self *MissionSetCurrent) Size() uint8 {
	return 4
}

// Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
type MissionCurrent struct {
	Seq	uint16	// Sequence
}

func(self *MissionCurrent) ID() uint8 {
	return 42
}

func(self *MissionCurrent) Size() uint8 {
	return 2
}

// Request the overall list of mission items from the system/component.
type MissionRequestList struct {
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
}

func(self *MissionRequestList) ID() uint8 {
	return 43
}

func(self *MissionRequestList) Size() uint8 {
	return 2
}

// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of MISSIONs.
type MissionCount struct {
	Count	uint16	// Number of mission items in the sequence
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
}

func(self *MissionCount) ID() uint8 {
	return 44
}

func(self *MissionCount) Size() uint8 {
	return 4
}

// Delete all mission items at once.
type MissionClearAll struct {
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
}

func(self *MissionClearAll) ID() uint8 {
	return 45
}

func(self *MissionClearAll) Size() uint8 {
	return 2
}

// A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next MISSION.
type MissionItemReached struct {
	Seq	uint16	// Sequence
}

func(self *MissionItemReached) ID() uint8 {
	return 46
}

func(self *MissionItemReached) Size() uint8 {
	return 2
}

// Ack message during MISSION handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
type MissionAck struct {
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
	Type	uint8	// See MAV_MISSION_RESULT enum
}

func(self *MissionAck) ID() uint8 {
	return 47
}

func(self *MissionAck) Size() uint8 {
	return 3
}

// As local waypoints exist, the global MISSION reference allows to transform between the local coordinate frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings are connected and the MAV should move from in- to outdoor.
type SetGpsGlobalOrigin struct {
	Latitude	int32	// Latitude (WGS84), in degrees * 1E7
	Longitude	int32	// Longitude (WGS84, in degrees * 1E7
	Altitude	int32	// Altitude (WGS84), in meters * 1000 (positive for up)
	TargetSystem	uint8	// System ID
}

func(self *SetGpsGlobalOrigin) ID() uint8 {
	return 48
}

func(self *SetGpsGlobalOrigin) Size() uint8 {
	return 13
}

// Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position
type GpsGlobalOrigin struct {
	Latitude	int32	// Latitude (WGS84), in degrees * 1E7
	Longitude	int32	// Longitude (WGS84), in degrees * 1E7
	Altitude	int32	// Altitude (WGS84), in meters * 1000 (positive for up)
}

func(self *GpsGlobalOrigin) ID() uint8 {
	return 49
}

func(self *GpsGlobalOrigin) Size() uint8 {
	return 12
}

// Set the setpoint for a local position controller. This is the position in local coordinates the MAV should fly to. This message is sent by the path/MISSION planner to the onboard position controller. As some MAVs have a degree of freedom in yaw (e.g. all helicopters/quadrotors), the desired yaw angle is part of the message.
type SetLocalPositionSetpoint struct {
	X	float32	// x position
	Y	float32	// y position
	Z	float32	// z position
	Yaw	float32	// Desired yaw angle
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
	CoordinateFrame	uint8	// Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
}

func(self *SetLocalPositionSetpoint) ID() uint8 {
	return 50
}

func(self *SetLocalPositionSetpoint) Size() uint8 {
	return 19
}

// Transmit the current local setpoint of the controller to other MAVs (collision avoidance) and to the GCS.
type LocalPositionSetpoint struct {
	X	float32	// x position
	Y	float32	// y position
	Z	float32	// z position
	Yaw	float32	// Desired yaw angle
	CoordinateFrame	uint8	// Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
}

func(self *LocalPositionSetpoint) ID() uint8 {
	return 51
}

func(self *LocalPositionSetpoint) Size() uint8 {
	return 17
}

// Transmit the current local setpoint of the controller to other MAVs (collision avoidance) and to the GCS.
type GlobalPositionSetpointInt struct {
	Latitude	int32	// Latitude (WGS84), in degrees * 1E7
	Longitude	int32	// Longitude (WGS84), in degrees * 1E7
	Altitude	int32	// Altitude (WGS84), in meters * 1000 (positive for up)
	Yaw	int16	// Desired yaw angle in degrees * 100
	CoordinateFrame	uint8	// Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
}

func(self *GlobalPositionSetpointInt) ID() uint8 {
	return 52
}

func(self *GlobalPositionSetpointInt) Size() uint8 {
	return 15
}

// Set the current global position setpoint.
type SetGlobalPositionSetpointInt struct {
	Latitude	int32	// Latitude (WGS84), in degrees * 1E7
	Longitude	int32	// Longitude (WGS84), in degrees * 1E7
	Altitude	int32	// Altitude (WGS84), in meters * 1000 (positive for up)
	Yaw	int16	// Desired yaw angle in degrees * 100
	CoordinateFrame	uint8	// Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
}

func(self *SetGlobalPositionSetpointInt) ID() uint8 {
	return 53
}

func(self *SetGlobalPositionSetpointInt) Size() uint8 {
	return 15
}

// Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/MISSIONs to accept and which to reject. Safety areas are often enforced by national or competition regulations.
type SafetySetAllowedArea struct {
	P1x	float32	// x position 1 / Latitude 1
	P1y	float32	// y position 1 / Longitude 1
	P1z	float32	// z position 1 / Altitude 1
	P2x	float32	// x position 2 / Latitude 2
	P2y	float32	// y position 2 / Longitude 2
	P2z	float32	// z position 2 / Altitude 2
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
	Frame	uint8	// Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
}

func(self *SafetySetAllowedArea) ID() uint8 {
	return 54
}

func(self *SafetySetAllowedArea) Size() uint8 {
	return 27
}

// Read out the safety zone the MAV currently assumes.
type SafetyAllowedArea struct {
	P1x	float32	// x position 1 / Latitude 1
	P1y	float32	// y position 1 / Longitude 1
	P1z	float32	// z position 1 / Altitude 1
	P2x	float32	// x position 2 / Latitude 2
	P2y	float32	// y position 2 / Longitude 2
	P2z	float32	// z position 2 / Altitude 2
	Frame	uint8	// Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
}

func(self *SafetyAllowedArea) ID() uint8 {
	return 55
}

func(self *SafetyAllowedArea) Size() uint8 {
	return 25
}

// Set roll, pitch and yaw.
type SetRollPitchYawThrust struct {
	Roll	float32	// Desired roll angle in radians
	Pitch	float32	// Desired pitch angle in radians
	Yaw	float32	// Desired yaw angle in radians
	Thrust	float32	// Collective thrust, normalized to 0 .. 1
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
}

func(self *SetRollPitchYawThrust) ID() uint8 {
	return 56
}

func(self *SetRollPitchYawThrust) Size() uint8 {
	return 18
}

// Set roll, pitch and yaw.
type SetRollPitchYawSpeedThrust struct {
	RollSpeed	float32	// Desired roll angular speed in rad/s
	PitchSpeed	float32	// Desired pitch angular speed in rad/s
	YawSpeed	float32	// Desired yaw angular speed in rad/s
	Thrust	float32	// Collective thrust, normalized to 0 .. 1
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
}

func(self *SetRollPitchYawSpeedThrust) ID() uint8 {
	return 57
}

func(self *SetRollPitchYawSpeedThrust) Size() uint8 {
	return 18
}

// Setpoint in roll, pitch, yaw currently active on the system.
type RollPitchYawThrustSetpoint struct {
	TimeBootMs	uint32	// Timestamp in milliseconds since system boot
	Roll	float32	// Desired roll angle in radians
	Pitch	float32	// Desired pitch angle in radians
	Yaw	float32	// Desired yaw angle in radians
	Thrust	float32	// Collective thrust, normalized to 0 .. 1
}

func(self *RollPitchYawThrustSetpoint) ID() uint8 {
	return 58
}

func(self *RollPitchYawThrustSetpoint) Size() uint8 {
	return 20
}

// Setpoint in rollspeed, pitchspeed, yawspeed currently active on the system.
type RollPitchYawSpeedThrustSetpoint struct {
	TimeBootMs	uint32	// Timestamp in milliseconds since system boot
	RollSpeed	float32	// Desired roll angular speed in rad/s
	PitchSpeed	float32	// Desired pitch angular speed in rad/s
	YawSpeed	float32	// Desired yaw angular speed in rad/s
	Thrust	float32	// Collective thrust, normalized to 0 .. 1
}

func(self *RollPitchYawSpeedThrustSetpoint) ID() uint8 {
	return 59
}

func(self *RollPitchYawSpeedThrustSetpoint) Size() uint8 {
	return 20
}

// Setpoint in the four motor speeds
type SetQuadMotorsSetpoint struct {
	MotorFrontNw	uint16	// Front motor in + configuration, front left motor in x configuration
	MotorRightNe	uint16	// Right motor in + configuration, front right motor in x configuration
	MotorBackSe	uint16	// Back motor in + configuration, back right motor in x configuration
	MotorLeftSw	uint16	// Left motor in + configuration, back left motor in x configuration
	TargetSystem	uint8	// System ID of the system that should set these motor commands
}

func(self *SetQuadMotorsSetpoint) ID() uint8 {
	return 60
}

func(self *SetQuadMotorsSetpoint) Size() uint8 {
	return 9
}

// Setpoint for up to four quadrotors in a group / wing
type SetQuadSwarmRollPitchYawThrust struct {
	Roll	[4]int16	// Desired roll angle in radians +-PI (+-INT16_MAX)
	Pitch	[4]int16	// Desired pitch angle in radians +-PI (+-INT16_MAX)
	Yaw	[4]int16	// Desired yaw angle in radians, scaled to int16 +-PI (+-INT16_MAX)
	Thrust	[4]uint16	// Collective thrust, scaled to uint16 (0..UINT16_MAX)
	Group	uint8	// ID of the quadrotor group (0 - 255, up to 256 groups supported)
	Mode	uint8	// ID of the flight mode (0 - 255, up to 256 modes supported)
}

func(self *SetQuadSwarmRollPitchYawThrust) ID() uint8 {
	return 61
}

func(self *SetQuadSwarmRollPitchYawThrust) Size() uint8 {
	return 34
}

// Outputs of the APM navigation controller. The primary use of this message is to check the response and signs of the controller before actual flight and to assist with tuning controller parameters.
type NavControllerOutput struct {
	NavRoll	float32	// Current desired roll in degrees
	NavPitch	float32	// Current desired pitch in degrees
	AltError	float32	// Current altitude error in meters
	AspdError	float32	// Current airspeed error in meters/second
	XtrackError	float32	// Current crosstrack error on x-y plane in meters
	NavBearing	int16	// Current desired heading in degrees
	TargetBearing	int16	// Bearing to current MISSION/target in degrees
	WpDist	uint16	// Distance to active MISSION in meters
}

func(self *NavControllerOutput) ID() uint8 {
	return 62
}

func(self *NavControllerOutput) Size() uint8 {
	return 26
}

// Setpoint for up to four quadrotors in a group / wing
type SetQuadSwarmLedRollPitchYawThrust struct {
	Roll	[4]int16	// Desired roll angle in radians +-PI (+-INT16_MAX)
	Pitch	[4]int16	// Desired pitch angle in radians +-PI (+-INT16_MAX)
	Yaw	[4]int16	// Desired yaw angle in radians, scaled to int16 +-PI (+-INT16_MAX)
	Thrust	[4]uint16	// Collective thrust, scaled to uint16 (0..UINT16_MAX)
	Group	uint8	// ID of the quadrotor group (0 - 255, up to 256 groups supported)
	Mode	uint8	// ID of the flight mode (0 - 255, up to 256 modes supported)
	LedRed	[4]uint8	// RGB red channel (0-255)
	LedBlue	[4]uint8	// RGB green channel (0-255)
	LedGreen	[4]uint8	// RGB blue channel (0-255)
}

func(self *SetQuadSwarmLedRollPitchYawThrust) ID() uint8 {
	return 63
}

func(self *SetQuadSwarmLedRollPitchYawThrust) Size() uint8 {
	return 46
}

// Corrects the systems state by adding an error correction term to the position and velocity, and by rotating the attitude by a correction angle.
type StateCorrection struct {
	Xerr	float32	// x position error
	Yerr	float32	// y position error
	Zerr	float32	// z position error
	Rollerr	float32	// roll error (radians)
	Pitcherr	float32	// pitch error (radians)
	Yawerr	float32	// yaw error (radians)
	Vxerr	float32	// x velocity
	Vyerr	float32	// y velocity
	Vzerr	float32	// z velocity
}

func(self *StateCorrection) ID() uint8 {
	return 64
}

func(self *StateCorrection) Size() uint8 {
	return 36
}

type RequestDataStream struct {
	ReqMessageRate	uint16	// The requested interval between two messages of this type
	TargetSystem	uint8	// The target requested to send the message stream.
	TargetComponent	uint8	// The target requested to send the message stream.
	ReqStreamId	uint8	// The ID of the requested data stream
	StartStop	uint8	// 1 to start sending, 0 to stop sending.
}

func(self *RequestDataStream) ID() uint8 {
	return 66
}

func(self *RequestDataStream) Size() uint8 {
	return 6
}

type DataStream struct {
	MessageRate	uint16	// The requested interval between two messages of this type
	StreamId	uint8	// The ID of the requested data stream
	OnOff	uint8	// 1 stream is enabled, 0 stream is stopped.
}

func(self *DataStream) ID() uint8 {
	return 67
}

func(self *DataStream) Size() uint8 {
	return 4
}

// This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their 
type ManualControl struct {
	X	int16	// X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
	Y	int16	// Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
	Z	int16	// Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle.
	R	int16	// R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
	Buttons	uint16	// A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
	Target	uint8	// The system to be controlled.
}

func(self *ManualControl) ID() uint8 {
	return 69
}

func(self *ManualControl) Size() uint8 {
	return 11
}

// The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannelsOverride struct {
	Chan1Raw	uint16	// RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan2Raw	uint16	// RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan3Raw	uint16	// RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan4Raw	uint16	// RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan5Raw	uint16	// RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan6Raw	uint16	// RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan7Raw	uint16	// RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan8Raw	uint16	// RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	TargetSystem	uint8	// System ID
	TargetComponent	uint8	// Component ID
}

func(self *RcChannelsOverride) ID() uint8 {
	return 70
}

func(self *RcChannelsOverride) Size() uint8 {
	return 18
}

// Metrics typically displayed on a HUD for fixed wing aircraft
type VfrHud struct {
	Airspeed	float32	// Current airspeed in m/s
	Groundspeed	float32	// Current ground speed in m/s
	Alt	float32	// Current altitude (MSL), in meters
	Climb	float32	// Current climb rate in meters/second
	Heading	int16	// Current heading in degrees, in compass units (0..360, 0=north)
	Throttle	uint16	// Current throttle setting in integer percent, 0 to 100
}

func(self *VfrHud) ID() uint8 {
	return 74
}

func(self *VfrHud) Size() uint8 {
	return 20
}

// Send a command with up to seven parameters to the MAV
type CommandLong struct {
	Param1	float32	// Parameter 1, as defined by MAV_CMD enum.
	Param2	float32	// Parameter 2, as defined by MAV_CMD enum.
	Param3	float32	// Parameter 3, as defined by MAV_CMD enum.
	Param4	float32	// Parameter 4, as defined by MAV_CMD enum.
	Param5	float32	// Parameter 5, as defined by MAV_CMD enum.
	Param6	float32	// Parameter 6, as defined by MAV_CMD enum.
	Param7	float32	// Parameter 7, as defined by MAV_CMD enum.
	Command	uint16	// Command ID, as defined by MAV_CMD enum.
	TargetSystem	uint8	// System which should execute the command
	TargetComponent	uint8	// Component which should execute the command, 0 for all components
	Confirmation	uint8	// 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
}

func(self *CommandLong) ID() uint8 {
	return 76
}

func(self *CommandLong) Size() uint8 {
	return 33
}

// Report status of a command. Includes feedback wether the command was executed.
type CommandAck struct {
	Command	uint16	// Command ID, as defined by MAV_CMD enum.
	Result	uint8	// See MAV_RESULT enum
}

func(self *CommandAck) ID() uint8 {
	return 77
}

func(self *CommandAck) Size() uint8 {
	return 3
}

// Setpoint in roll, pitch, yaw rates and thrust currently active on the system.
type RollPitchYawRatesThrustSetpoint struct {
	TimeBootMs	uint32	// Timestamp in milliseconds since system boot
	RollRate	float32	// Desired roll rate in radians per second
	PitchRate	float32	// Desired pitch rate in radians per second
	YawRate	float32	// Desired yaw rate in radians per second
	Thrust	float32	// Collective thrust, normalized to 0 .. 1
}

func(self *RollPitchYawRatesThrustSetpoint) ID() uint8 {
	return 80
}

func(self *RollPitchYawRatesThrustSetpoint) Size() uint8 {
	return 20
}

// Setpoint in roll, pitch, yaw and thrust from the operator
type ManualSetpoint struct {
	TimeBootMs	uint32	// Timestamp in milliseconds since system boot
	Roll	float32	// Desired roll rate in radians per second
	Pitch	float32	// Desired pitch rate in radians per second
	Yaw	float32	// Desired yaw rate in radians per second
	Thrust	float32	// Collective thrust, normalized to 0 .. 1
	ModeSwitch	uint8	// Flight mode switch position, 0.. 255
	ManualOverrideSwitch	uint8	// Override mode switch position, 0.. 255
}

func(self *ManualSetpoint) ID() uint8 {
	return 81
}

func(self *ManualSetpoint) Size() uint8 {
	return 22
}

// The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNedSystemGlobalOffset struct {
	TimeBootMs	uint32	// Timestamp (milliseconds since system boot)
	X	float32	// X Position
	Y	float32	// Y Position
	Z	float32	// Z Position
	Roll	float32	// Roll
	Pitch	float32	// Pitch
	Yaw	float32	// Yaw
}

func(self *LocalPositionNedSystemGlobalOffset) ID() uint8 {
	return 89
}

func(self *LocalPositionNedSystemGlobalOffset) Size() uint8 {
	return 28
}

// DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
type HilState struct {
	TimeUsec	uint64	// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Roll	float32	// Roll angle (rad)
	Pitch	float32	// Pitch angle (rad)
	Yaw	float32	// Yaw angle (rad)
	Rollspeed	float32	// Body frame roll / phi angular speed (rad/s)
	Pitchspeed	float32	// Body frame pitch / theta angular speed (rad/s)
	Yawspeed	float32	// Body frame yaw / psi angular speed (rad/s)
	Lat	int32	// Latitude, expressed as * 1E7
	Lon	int32	// Longitude, expressed as * 1E7
	Alt	int32	// Altitude in meters, expressed as * 1000 (millimeters)
	Vx	int16	// Ground X Speed (Latitude), expressed as m/s * 100
	Vy	int16	// Ground Y Speed (Longitude), expressed as m/s * 100
	Vz	int16	// Ground Z Speed (Altitude), expressed as m/s * 100
	Xacc	int16	// X acceleration (mg)
	Yacc	int16	// Y acceleration (mg)
	Zacc	int16	// Z acceleration (mg)
}

func(self *HilState) ID() uint8 {
	return 90
}

func(self *HilState) Size() uint8 {
	return 56
}

// Sent from autopilot to simulation. Hardware in the loop control outputs
type HilControls struct {
	TimeUsec	uint64	// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	RollAilerons	float32	// Control output -1 .. 1
	PitchElevator	float32	// Control output -1 .. 1
	YawRudder	float32	// Control output -1 .. 1
	Throttle	float32	// Throttle 0 .. 1
	Aux1	float32	// Aux 1, -1 .. 1
	Aux2	float32	// Aux 2, -1 .. 1
	Aux3	float32	// Aux 3, -1 .. 1
	Aux4	float32	// Aux 4, -1 .. 1
	Mode	uint8	// System mode (MAV_MODE)
	NavMode	uint8	// Navigation mode (MAV_NAV_MODE)
}

func(self *HilControls) ID() uint8 {
	return 91
}

func(self *HilControls) Size() uint8 {
	return 42
}

// Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type HilRcInputsRaw struct {
	TimeUsec	uint64	// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Chan1Raw	uint16	// RC channel 1 value, in microseconds
	Chan2Raw	uint16	// RC channel 2 value, in microseconds
	Chan3Raw	uint16	// RC channel 3 value, in microseconds
	Chan4Raw	uint16	// RC channel 4 value, in microseconds
	Chan5Raw	uint16	// RC channel 5 value, in microseconds
	Chan6Raw	uint16	// RC channel 6 value, in microseconds
	Chan7Raw	uint16	// RC channel 7 value, in microseconds
	Chan8Raw	uint16	// RC channel 8 value, in microseconds
	Chan9Raw	uint16	// RC channel 9 value, in microseconds
	Chan10Raw	uint16	// RC channel 10 value, in microseconds
	Chan11Raw	uint16	// RC channel 11 value, in microseconds
	Chan12Raw	uint16	// RC channel 12 value, in microseconds
	Rssi	uint8	// Receive signal strength indicator, 0: 0%, 255: 100%
}

func(self *HilRcInputsRaw) ID() uint8 {
	return 92
}

func(self *HilRcInputsRaw) Size() uint8 {
	return 33
}

// Optical flow from a flow sensor (e.g. optical mouse sensor)
type OpticalFlow struct {
	TimeUsec	uint64	// Timestamp (UNIX)
	FlowCompMX	float32	// Flow in meters in x-sensor direction, angular-speed compensated
	FlowCompMY	float32	// Flow in meters in y-sensor direction, angular-speed compensated
	GroundDistance	float32	// Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
	FlowX	int16	// Flow in pixels * 10 in x-sensor direction (dezi-pixels)
	FlowY	int16	// Flow in pixels * 10 in y-sensor direction (dezi-pixels)
	SensorId	uint8	// Sensor ID
	Quality	uint8	// Optical flow quality / confidence. 0: bad, 255: maximum quality
}

func(self *OpticalFlow) ID() uint8 {
	return 100
}

func(self *OpticalFlow) Size() uint8 {
	return 26
}

type GlobalVisionPositionEstimate struct {
	Usec	uint64	// Timestamp (microseconds, synced to UNIX time or since system boot)
	X	float32	// Global X position
	Y	float32	// Global Y position
	Z	float32	// Global Z position
	Roll	float32	// Roll angle in rad
	Pitch	float32	// Pitch angle in rad
	Yaw	float32	// Yaw angle in rad
}

func(self *GlobalVisionPositionEstimate) ID() uint8 {
	return 101
}

func(self *GlobalVisionPositionEstimate) Size() uint8 {
	return 32
}

type VisionPositionEstimate struct {
	Usec	uint64	// Timestamp (microseconds, synced to UNIX time or since system boot)
	X	float32	// Global X position
	Y	float32	// Global Y position
	Z	float32	// Global Z position
	Roll	float32	// Roll angle in rad
	Pitch	float32	// Pitch angle in rad
	Yaw	float32	// Yaw angle in rad
}

func(self *VisionPositionEstimate) ID() uint8 {
	return 102
}

func(self *VisionPositionEstimate) Size() uint8 {
	return 32
}

type VisionSpeedEstimate struct {
	Usec	uint64	// Timestamp (microseconds, synced to UNIX time or since system boot)
	X	float32	// Global X speed
	Y	float32	// Global Y speed
	Z	float32	// Global Z speed
}

func(self *VisionSpeedEstimate) ID() uint8 {
	return 103
}

func(self *VisionSpeedEstimate) Size() uint8 {
	return 20
}

type ViconPositionEstimate struct {
	Usec	uint64	// Timestamp (microseconds, synced to UNIX time or since system boot)
	X	float32	// Global X position
	Y	float32	// Global Y position
	Z	float32	// Global Z position
	Roll	float32	// Roll angle in rad
	Pitch	float32	// Pitch angle in rad
	Yaw	float32	// Yaw angle in rad
}

func(self *ViconPositionEstimate) ID() uint8 {
	return 104
}

func(self *ViconPositionEstimate) Size() uint8 {
	return 32
}

// The IMU readings in SI units in NED body frame
type HighresImu struct {
	TimeUsec	uint64	// Timestamp (microseconds, synced to UNIX time or since system boot)
	Xacc	float32	// X acceleration (m/s^2)
	Yacc	float32	// Y acceleration (m/s^2)
	Zacc	float32	// Z acceleration (m/s^2)
	Xgyro	float32	// Angular speed around X axis (rad / sec)
	Ygyro	float32	// Angular speed around Y axis (rad / sec)
	Zgyro	float32	// Angular speed around Z axis (rad / sec)
	Xmag	float32	// X Magnetic field (Gauss)
	Ymag	float32	// Y Magnetic field (Gauss)
	Zmag	float32	// Z Magnetic field (Gauss)
	AbsPressure	float32	// Absolute pressure in millibar
	DiffPressure	float32	// Differential pressure in millibar
	PressureAlt	float32	// Altitude calculated from pressure
	Temperature	float32	// Temperature in degrees celsius
	FieldsUpdated	uint16	// Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
}

func(self *HighresImu) ID() uint8 {
	return 105
}

func(self *HighresImu) Size() uint8 {
	return 62
}

// Optical flow from an omnidirectional flow sensor (e.g. PX4FLOW with wide angle lens)
type OmnidirectionalFlow struct {
	TimeUsec	uint64	// Timestamp (microseconds, synced to UNIX time or since system boot)
	FrontDistanceM	float32	// Front distance in meters. Positive value (including zero): distance known. Negative value: Unknown distance
	Left	[10]int16	// Flow in deci pixels (1 = 0.1 pixel) on left hemisphere
	Right	[10]int16	// Flow in deci pixels (1 = 0.1 pixel) on right hemisphere
	SensorId	uint8	// Sensor ID
	Quality	uint8	// Optical flow quality / confidence. 0: bad, 255: maximum quality
}

func(self *OmnidirectionalFlow) ID() uint8 {
	return 106
}

func(self *OmnidirectionalFlow) Size() uint8 {
	return 54
}

// The IMU readings in SI units in NED body frame
type HilSensor struct {
	TimeUsec	uint64	// Timestamp (microseconds, synced to UNIX time or since system boot)
	Xacc	float32	// X acceleration (m/s^2)
	Yacc	float32	// Y acceleration (m/s^2)
	Zacc	float32	// Z acceleration (m/s^2)
	Xgyro	float32	// Angular speed around X axis in body frame (rad / sec)
	Ygyro	float32	// Angular speed around Y axis in body frame (rad / sec)
	Zgyro	float32	// Angular speed around Z axis in body frame (rad / sec)
	Xmag	float32	// X Magnetic field (Gauss)
	Ymag	float32	// Y Magnetic field (Gauss)
	Zmag	float32	// Z Magnetic field (Gauss)
	AbsPressure	float32	// Absolute pressure in millibar
	DiffPressure	float32	// Differential pressure (airspeed) in millibar
	PressureAlt	float32	// Altitude calculated from pressure
	Temperature	float32	// Temperature in degrees celsius
	FieldsUpdated	uint32	// Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
}

func(self *HilSensor) ID() uint8 {
	return 107
}

func(self *HilSensor) Size() uint8 {
	return 64
}

// Status of simulation environment, if used
type SimState struct {
	Q1	float32	// True attitude quaternion component 1
	Q2	float32	// True attitude quaternion component 2
	Q3	float32	// True attitude quaternion component 3
	Q4	float32	// True attitude quaternion component 4
	Roll	float32	// Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
	Pitch	float32	// Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
	Yaw	float32	// Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
	Xacc	float32	// X acceleration m/s/s
	Yacc	float32	// Y acceleration m/s/s
	Zacc	float32	// Z acceleration m/s/s
	Xgyro	float32	// Angular speed around X axis rad/s
	Ygyro	float32	// Angular speed around Y axis rad/s
	Zgyro	float32	// Angular speed around Z axis rad/s
	Lat	float32	// Latitude in degrees
	Lon	float32	// Longitude in degrees
	Alt	float32	// Altitude in meters
	StdDevHorz	float32	// Horizontal position standard deviation
	StdDevVert	float32	// Vertical position standard deviation
	Vn	float32	// True velocity in m/s in NORTH direction in earth-fixed NED frame
	Ve	float32	// True velocity in m/s in EAST direction in earth-fixed NED frame
	Vd	float32	// True velocity in m/s in DOWN direction in earth-fixed NED frame
}

func(self *SimState) ID() uint8 {
	return 108
}

func(self *SimState) Size() uint8 {
	return 84
}

// Status generated by radio
type RadioStatus struct {
	Rxerrors	uint16	// receive errors
	Fixed	uint16	// count of error corrected packets
	Rssi	uint8	// local signal strength
	Remrssi	uint8	// remote signal strength
	Txbuf	uint8	// how full the tx buffer is as a percentage
	Noise	uint8	// background noise level
	Remnoise	uint8	// remote background noise level
}

func(self *RadioStatus) ID() uint8 {
	return 109
}

func(self *RadioStatus) Size() uint8 {
	return 9
}

// Begin file transfer
type FileTransferStart struct {
	TransferUid	uint64	// Unique transfer ID
	FileSize	uint32	// File size in bytes
	DestPath	[240]byte	// Destination path
	Direction	uint8	// Transfer direction: 0: from requester, 1: to requester
	Flags	uint8	// RESERVED
}

func(self *FileTransferStart) ID() uint8 {
	return 110
}

func(self *FileTransferStart) Size() uint8 {
	return 254
}

// Get directory listing
type FileTransferDirList struct {
	TransferUid	uint64	// Unique transfer ID
	DirPath	[240]byte	// Directory path to list
	Flags	uint8	// RESERVED
}

func(self *FileTransferDirList) ID() uint8 {
	return 111
}

func(self *FileTransferDirList) Size() uint8 {
	return 249
}

// File transfer result
type FileTransferRes struct {
	TransferUid	uint64	// Unique transfer ID
	Result	uint8	// 0: OK, 1: not permitted, 2: bad path / file name, 3: no space left on device
}

func(self *FileTransferRes) ID() uint8 {
	return 112
}

func(self *FileTransferRes) Size() uint8 {
	return 9
}

// The global position, as returned by the Global Positioning System (GPS). This is
//                  NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
type HilGps struct {
	TimeUsec	uint64	// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Lat	int32	// Latitude (WGS84), in degrees * 1E7
	Lon	int32	// Longitude (WGS84), in degrees * 1E7
	Alt	int32	// Altitude (WGS84), in meters * 1000 (positive for up)
	Eph	uint16	// GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
	Epv	uint16	// GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
	Vel	uint16	// GPS ground speed (m/s * 100). If unknown, set to: 65535
	Vn	int16	// GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
	Ve	int16	// GPS velocity in cm/s in EAST direction in earth-fixed NED frame
	Vd	int16	// GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
	Cog	uint16	// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
	FixType	uint8	// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
	SatellitesVisible	uint8	// Number of satellites visible. If unknown, set to 255
}

func(self *HilGps) ID() uint8 {
	return 113
}

func(self *HilGps) Size() uint8 {
	return 36
}

// Simulated optical flow from a flow sensor (e.g. optical mouse sensor)
type HilOpticalFlow struct {
	TimeUsec	uint64	// Timestamp (UNIX)
	FlowCompMX	float32	// Flow in meters in x-sensor direction, angular-speed compensated
	FlowCompMY	float32	// Flow in meters in y-sensor direction, angular-speed compensated
	GroundDistance	float32	// Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
	FlowX	int16	// Flow in pixels in x-sensor direction
	FlowY	int16	// Flow in pixels in y-sensor direction
	SensorId	uint8	// Sensor ID
	Quality	uint8	// Optical flow quality / confidence. 0: bad, 255: maximum quality
}

func(self *HilOpticalFlow) ID() uint8 {
	return 114
}

func(self *HilOpticalFlow) Size() uint8 {
	return 26
}

// Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.
type HilStateQuaternion struct {
	TimeUsec	uint64	// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	AttitudeQuaternion	[4]float32	// Vehicle attitude expressed as normalized quaternion
	Rollspeed	float32	// Body frame roll / phi angular speed (rad/s)
	Pitchspeed	float32	// Body frame pitch / theta angular speed (rad/s)
	Yawspeed	float32	// Body frame yaw / psi angular speed (rad/s)
	Lat	int32	// Latitude, expressed as * 1E7
	Lon	int32	// Longitude, expressed as * 1E7
	Alt	int32	// Altitude in meters, expressed as * 1000 (millimeters)
	Vx	int16	// Ground X Speed (Latitude), expressed as m/s * 100
	Vy	int16	// Ground Y Speed (Longitude), expressed as m/s * 100
	Vz	int16	// Ground Z Speed (Altitude), expressed as m/s * 100
	IndAirspeed	uint16	// Indicated airspeed, expressed as m/s * 100
	TrueAirspeed	uint16	// True airspeed, expressed as m/s * 100
	Xacc	int16	// X acceleration (mg)
	Yacc	int16	// Y acceleration (mg)
	Zacc	int16	// Z acceleration (mg)
}

func(self *HilStateQuaternion) ID() uint8 {
	return 115
}

func(self *HilStateQuaternion) Size() uint8 {
	return 64
}

// Transmitte battery informations for a accu pack.
type BatteryStatus struct {
	CurrentConsumed	int32	// Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
	EnergyConsumed	int32	// Consumed energy, in 100*Joules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
	VoltageCell1	uint16	// Battery voltage of cell 1, in millivolts (1 = 1 millivolt)
	VoltageCell2	uint16	// Battery voltage of cell 2, in millivolts (1 = 1 millivolt), -1: no cell
	VoltageCell3	uint16	// Battery voltage of cell 3, in millivolts (1 = 1 millivolt), -1: no cell
	VoltageCell4	uint16	// Battery voltage of cell 4, in millivolts (1 = 1 millivolt), -1: no cell
	VoltageCell5	uint16	// Battery voltage of cell 5, in millivolts (1 = 1 millivolt), -1: no cell
	VoltageCell6	uint16	// Battery voltage of cell 6, in millivolts (1 = 1 millivolt), -1: no cell
	CurrentBattery	int16	// Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
	AccuId	uint8	// Accupack ID
	BatteryRemaining	int8	// Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
}

func(self *BatteryStatus) ID() uint8 {
	return 147
}

func(self *BatteryStatus) Size() uint8 {
	return 24
}

// Set the 8 DOF setpoint for a controller.
type Setpoint8dof struct {
	Val1	float32	// Value 1
	Val2	float32	// Value 2
	Val3	float32	// Value 3
	Val4	float32	// Value 4
	Val5	float32	// Value 5
	Val6	float32	// Value 6
	Val7	float32	// Value 7
	Val8	float32	// Value 8
	TargetSystem	uint8	// System ID
}

func(self *Setpoint8dof) ID() uint8 {
	return 148
}

func(self *Setpoint8dof) Size() uint8 {
	return 33
}

// Set the 6 DOF setpoint for a attitude and position controller.
type Setpoint6dof struct {
	TransX	float32	// Translational Component in x
	TransY	float32	// Translational Component in y
	TransZ	float32	// Translational Component in z
	RotX	float32	// Rotational Component in x
	RotY	float32	// Rotational Component in y
	RotZ	float32	// Rotational Component in z
	TargetSystem	uint8	// System ID
}

func(self *Setpoint6dof) ID() uint8 {
	return 149
}

func(self *Setpoint6dof) Size() uint8 {
	return 25
}

// Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type MemoryVect struct {
	Address	uint16	// Starting address of the debug variables
	Ver	uint8	// Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
	Type	uint8	// Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
	Value	[32]int8	// Memory contents at specified address
}

func(self *MemoryVect) ID() uint8 {
	return 249
}

func(self *MemoryVect) Size() uint8 {
	return 36
}

type DebugVect struct {
	TimeUsec	uint64	// Timestamp
	X	float32	// x
	Y	float32	// y
	Z	float32	// z
	Name	[10]byte	// Name
}

func(self *DebugVect) ID() uint8 {
	return 250
}

func(self *DebugVect) Size() uint8 {
	return 30
}

// Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type NamedValueFloat struct {
	TimeBootMs	uint32	// Timestamp (milliseconds since system boot)
	Value	float32	// Floating point value
	Name	[10]byte	// Name of the debug variable
}

func(self *NamedValueFloat) ID() uint8 {
	return 251
}

func(self *NamedValueFloat) Size() uint8 {
	return 18
}

// Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type NamedValueInt struct {
	TimeBootMs	uint32	// Timestamp (milliseconds since system boot)
	Value	int32	// Signed integer value
	Name	[10]byte	// Name of the debug variable
}

func(self *NamedValueInt) ID() uint8 {
	return 252
}

func(self *NamedValueInt) Size() uint8 {
	return 18
}

// Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
type Statustext struct {
	Severity	uint8	// Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
	Text	[50]byte	// Status text message, without null termination character
}

func(self *Statustext) ID() uint8 {
	return 253
}

func(self *Statustext) Size() uint8 {
	return 51
}

// Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
type Debug struct {
	TimeBootMs	uint32	// Timestamp (milliseconds since system boot)
	Value	float32	// DEBUG value
	Ind	uint8	// index of debug variable
}

func(self *Debug) ID() uint8 {
	return 254
}

func(self *Debug) Size() uint8 {
	return 9
}
