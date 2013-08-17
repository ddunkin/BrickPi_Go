package brickpi

/*
*  Matthew Richardson
*  matthewrichardson37<at>gmail.com
*  http://mattallen37.wordpress.com/
*  Initial date: June 4, 2013
*  Last updated: July 2, 2013
*
*  You may use this code as you wish, provided you give credit where it's due.
*
*  This is a library of functions for the RPi to communicate with the BrickPi.
 */

// #include <wiringPi.h>

const (
	PORT_A = 0
	PORT_B = 1
	PORT_C = 2
	PORT_D = 3

	PORT_1 = 0
	PORT_2 = 1
	PORT_3 = 2
	PORT_4 = 3

	MASK_D0_M = 0x01
	MASK_D1_M = 0x02
	MASK_9V   = 0x04
	MASK_D0_S = 0x08
	MASK_D1_S = 0x10

	BYTE_MSG_TYPE             = 0 // MSG_TYPE is the first byte.
	MSG_TYPE_CHANGE_ADDR      = 1 // Change the UART address.
	MSG_TYPE_SENSOR_TYPE      = 2 // Change/set the sensor type.
	MSG_TYPE_VALUES           = 3 // Set the motor speed and direction, and return the sesnors and encoders.
	MSG_TYPE_E_STOP           = 4 // Float motors immidately
	MSG_TYPE_TIMEOUT_SETTINGS = 5 // Set the timeout

	// New UART address (MSG_TYPE_CHANGE_ADDR)
	BYTE_NEW_ADDRESS = 1

	// Sensor setup (MSG_TYPE_SENSOR_TYPE)
	BYTE_SENSOR_1_TYPE = 1
	BYTE_SENSOR_2_TYPE = 2

	// Timeout setup (MSG_TYPE_TIMEOUT_SETTINGS)
	BYTE_TIMEOUT = 1

	TYPE_MOTOR_PWM      = 0
	TYPE_MOTOR_SPEED    = 1
	TYPE_MOTOR_POSITION = 2

	TYPE_SENSOR_RAW             = 0 // - 31
	TYPE_SENSOR_LIGHT_OFF       = 0
	TYPE_SENSOR_LIGHT_ON        = (MASK_D0_M | MASK_D0_S)
	TYPE_SENSOR_TOUCH           = 32
	TYPE_SENSOR_ULTRASONIC_CONT = 33
	TYPE_SENSOR_ULTRASONIC_SS   = 34
	TYPE_SENSOR_RCX_LIGHT       = 35 // tested minimally
	TYPE_SENSOR_COLOR_FULL      = 36
	TYPE_SENSOR_COLOR_RED       = 37
	TYPE_SENSOR_COLOR_GREEN     = 38
	TYPE_SENSOR_COLOR_BLUE      = 39
	TYPE_SENSOR_COLOR_NONE      = 40
	TYPE_SENSOR_I2C             = 41
	TYPE_SENSOR_I2C_9V          = 42

	BIT_I2C_MID  = 0x01 // Do one of those funny clock pulses between writing and reading. defined for each device.
	BIT_I2C_SAME = 0x02 // The transmit data, and the number of bytes to read and write isn't going to change. defined for each device.

	INDEX_RED   = 0
	INDEX_GREEN = 1
	INDEX_BLUE  = 2
	INDEX_BLANK = 3
)

type BrickPiStruct struct {
	Address [2]byte // Communication addresses
	Timeout uint32  // Communication timeout (how int32 in ms since the last valid communication before floating the motors). 0 disables the timeout.

	/*
	   Motors
	*/
	MotorSpeed  [4]int32 // Motor speeds, from -255 to 255
	MotorEnable [4]byte  // Motor enable/disable

	/*
	   Encoders
	*/
	EncoderOffset [4]int32 // Encoder offsets (not yet implemented)
	Encoder       [4]int32 // Encoder values

	/*
	   Sensors
	*/
	Sensor         [4]int32    // Primary sensor values
	SensorArray    [4][4]int32 // For more sensor values for the sensor (e.g. for color sensor FULL mode).
	SensorType     [4]byte     // Sensor types
	SensorSettings [4][8]byte  // Sensor settings, used for specifying I2C settings.

	/*
	   I2C
	*/
	SensorI2CDevices [4]byte        // How many I2C devices are on each bus (1 - 8).
	SensorI2CSpeed   [4]byte        // The I2C speed.
	SensorI2CAddr    [4][8]byte     // The I2C address of each device on each bus.
	SensorI2CWrite   [4][8]byte     // How many bytes to write
	SensorI2CRead    [4][8]byte     // How many bytes to read
	SensorI2COut     [4][8][16]byte // The I2C bytes to write
	SensorI2CIn      [4][8][16]byte // The I2C input buffers
}

var Array [256]byte
var BytesReceived byte

func BrickPiChangeAddress(OldAddr byte, NewAddr byte) int32 {
	var i byte = 0
	Array[BYTE_MSG_TYPE] = MSG_TYPE_CHANGE_ADDR
	Array[BYTE_NEW_ADDRESS] = NewAddr
	BrickPiTx(OldAddr, 2, Array)

	if BrickPiRx(&BytesReceived, Array, 5000) {
		return -1
	}
	if !(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_CHANGE_ADDR) {
		return -1
	}

	return 0
}

func BrickPiSetTimeout() int32 {
	var i byte = 0
	for i < 2 {
		Array[BYTE_MSG_TYPE] = MSG_TYPE_TIMEOUT_SETTINGS
		Array[BYTE_TIMEOUT] = (BrickPi.Timeout & 0xFF)
		Array[(BYTE_TIMEOUT + 1)] = ((BrickPi.Timeout / 256) & 0xFF)
		Array[(BYTE_TIMEOUT + 2)] = ((BrickPi.Timeout / 65536) & 0xFF)
		Array[(BYTE_TIMEOUT + 3)] = ((BrickPi.Timeout / 16777216) & 0xFF)
		BrickPiTx(BrickPi.Address[i], 5, Array)
		if BrickPiRx(&BytesReceived, Array, 2500) {
			return -1
		}
		if !(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_TIMEOUT_SETTINGS) {
			return -1
		}
		i++
	}
	return 0
}

var Bit_Offset uint32 = 0

func AddBits(byte_offset byte, bit_offset byte, bits byte, value uint32) {
	var i byte = 0
	for i < bits {
		if value & 0x01 {
			Array[(byte_offset + ((bit_offset + Bit_Offset + i) / 8))] |= (0x01 << ((bit_offset + Bit_Offset + i) % 8))
		}
		value /= 2
		i++
	}
	Bit_Offset += bits
}

func GetBits(byte_offset byte, bit_offset byte, bits byte) uint32 {
	var Result uint32 = 0
	i := bits
	for i {
		Result *= 2
		Result |= ((Array[(byte_offset+((bit_offset+Bit_Offset+(i-1))/8))] >> ((bit_offset + Bit_Offset + (i - 1)) % 8)) & 0x01)
		i--
	}
	Bit_Offset += bits
	return Result
}

func BitsNeeded(value uint32) byte {
	var i byte = 0
	for i < 32 {
		if !value {
			return i
		}
		value /= 2
		i++
	}
	return 31
}

func BrickPiSetupSensors() int32 {
	var i byte = 0
	for i < 2 {
		var ii int32 = 0
		for ii < 256 {
			Array[ii] = 0
			ii++
		}
		Bit_Offset = 0
		Array[BYTE_MSG_TYPE] = MSG_TYPE_SENSOR_TYPE
		Array[BYTE_SENSOR_1_TYPE] = BrickPi.SensorType[PORT_1+(i*2)]
		Array[BYTE_SENSOR_2_TYPE] = BrickPi.SensorType[PORT_2+(i*2)]
		ii = 0
		for ii < 2 {
			var port byte = (i * 2) + ii
			if Array[BYTE_SENSOR_1_TYPE+ii] == TYPE_SENSOR_I2C ||
				Array[BYTE_SENSOR_1_TYPE+ii] == TYPE_SENSOR_I2C_9V {
				AddBits(3, 0, 8, BrickPi.SensorI2CSpeed[port])

				if BrickPi.SensorI2CDevices[port] > 8 {
					BrickPi.SensorI2CDevices[port] = 8
				}

				if BrickPi.SensorI2CDevices[port] == 0 {
					BrickPi.SensorI2CDevices[port] = 1
				}

				AddBits(3, 0, 3, (BrickPi.SensorI2CDevices[port] - 1))

				var device byte = 0
				for device < BrickPi.SensorI2CDevices[port] {
					AddBits(3, 0, 7, (BrickPi.SensorI2CAddr[port][device] >> 1))
					AddBits(3, 0, 2, BrickPi.SensorSettings[port][device])
					if BrickPi.SensorSettings[port][device] & BIT_I2C_SAME {
						AddBits(3, 0, 4, BrickPi.SensorI2CWrite[port][device])
						AddBits(3, 0, 4, BrickPi.SensorI2CRead[port][device])
						var out_byte byte = 0
						for out_byte < BrickPi.SensorI2CWrite[port][device] {
							AddBits(3, 0, 8, BrickPi.SensorI2COut[port][device][out_byte])
							out_byte++
						}
					}
					device++
				}
			}
			ii++
		}
		var UART_TX_BYTES byte = (((Bit_Offset + 7) / 8) + 3)
		BrickPiTx(BrickPi.Address[i], UART_TX_BYTES, Array)
		if BrickPiRx(&BytesReceived, Array, 500000) {
			return -1
		}
		if !(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_SENSOR_TYPE) {
			return -1
		}
		i++
	}
	return 0
}

var Retried byte = 0 // For re-trying a failed update.

func BrickPiUpdateValues() int32 {
	var i byte = 0
	var ii uint32 = 0
	for i < 2 {
		Retried = 0

	__RETRY_COMMUNICATION__:

		ii = 0
		for ii < 256 {
			Array[ii] = 0
			ii++
		}

		Array[BYTE_MSG_TYPE] = MSG_TYPE_VALUES

		Bit_Offset = 0

		//    AddBits(1, 0, 2, 0);     use this to disable encoder offset

		ii = 0 // use this for encoder offset support
		for ii < 2 {
			var port byte = (i * 2) + ii
			if BrickPi.EncoderOffset[port] {
				var Temp_Value int32 = BrickPi.EncoderOffset[port]
				var Temp_ENC_DIR byte
				var Temp_BitsNeeded byte

				AddBits(1, 0, 1, 1)
				for Temp_Value < 0 {
					Temp_ENC_DIR = 1
					Temp_Value *= (-1)
				}
				Temp_BitsNeeded = (BitsNeeded(Temp_Value) + 1)
				AddBits(1, 0, 5, Temp_BitsNeeded)
				Temp_Value *= 2
				Temp_Value |= Temp_ENC_DIR
				AddBits(1, 0, Temp_BitsNeeded, Temp_Value)
			} else {
				AddBits(1, 0, 1, 0)
			}
			ii++
		}

		var speed int32
		var dir byte
		ii = 0
		for ii < 2 {
			var port byte = (i * 2) + ii
			speed = BrickPi.MotorSpeed[port]
			dir = 0
			if speed < 0 {
				dir = 1
				speed *= (-1)
			}
			if speed > 255 {
				speed = 255
			}
			AddBits(1, 0, 10, ((((speed & 0xFF) << 2) | (dir << 1) | (BrickPi.MotorEnable[port] & 0x01)) & 0x3FF))
			ii++
		}

		ii = 0
		for ii < 2 {
			var port byte = (i * 2) + ii
			if BrickPi.SensorType[port] == TYPE_SENSOR_I2C ||
				BrickPi.SensorType[port] == TYPE_SENSOR_I2C_9V {
				var device byte = 0
				for device < BrickPi.SensorI2CDevices[port] {
					if !(BrickPi.SensorSettings[port][device] & BIT_I2C_SAME) {
						AddBits(1, 0, 4, BrickPi.SensorI2CWrite[port][device])
						AddBits(1, 0, 4, BrickPi.SensorI2CRead[port][device])
						var out_byte byte = 0
						for out_byte < BrickPi.SensorI2CWrite[port][device] {
							AddBits(1, 0, 8, BrickPi.SensorI2COut[port][device][out_byte])
							out_byte++
						}
					}
					device++
				}
			}
			ii++
		}

		var UART_TX_BYTES byte = (((Bit_Offset + 7) / 8) + 1)
		BrickPiTx(BrickPi.Address[i], UART_TX_BYTES, Array)

		var result int32 = BrickPiRx(&BytesReceived, Array, 7500)

		if result != -2 { // -2 is the only error that indicates that the BrickPi uC did not properly receive the message
			BrickPi.EncoderOffset[((i * 2) + PORT_A)] = 0
			BrickPi.EncoderOffset[((i * 2) + PORT_B)] = 0
		}

		if result || (Array[BYTE_MSG_TYPE] != MSG_TYPE_VALUES) {
			log.Printf("BrickPiRx error: %d\n", result)
			if Retried < 2 {
				Retried++
				goto __RETRY_COMMUNICATION__
			} else {
				log.Printf("Retry failed.\n")
				return -1
			}
		}

		Bit_Offset = 0

		var Temp_BitsUsed [2]byte // Used for encoder values
		Temp_BitsUsed[0] = GetBits(1, 0, 5)
		Temp_BitsUsed[1] = GetBits(1, 0, 5)
		var Temp_EncoderVal uint32

		ii = 0
		for ii < 2 {
			Temp_EncoderVal = GetBits(1, 0, Temp_BitsUsed[ii])
			if Temp_EncoderVal & 0x01 {
				Temp_EncoderVal /= 2
				BrickPi.Encoder[ii+(i*2)] = Temp_EncoderVal * (-1)
			} else {
				BrickPi.Encoder[ii+(i*2)] = (Temp_EncoderVal / 2)
			}
			ii++
		}

		ii = 0
		for ii < 2 {
			var port byte = ii + (i * 2)
			switch BrickPi.SensorType[port] {
			case TYPE_SENSOR_TOUCH:
				BrickPi.Sensor[port] = GetBits(1, 0, 1)
				break
			case TYPE_SENSOR_ULTRASONIC_CONT:
			case TYPE_SENSOR_ULTRASONIC_SS:
				BrickPi.Sensor[port] = GetBits(1, 0, 8)
				break
			case TYPE_SENSOR_COLOR_FULL:
				BrickPi.Sensor[port] = GetBits(1, 0, 3)
				BrickPi.SensorArray[port][INDEX_BLANK] = GetBits(1, 0, 10)
				BrickPi.SensorArray[port][INDEX_RED] = GetBits(1, 0, 10)
				BrickPi.SensorArray[port][INDEX_GREEN] = GetBits(1, 0, 10)
				BrickPi.SensorArray[port][INDEX_BLUE] = GetBits(1, 0, 10)
				break
			case TYPE_SENSOR_I2C:
			case TYPE_SENSOR_I2C_9V:
				BrickPi.Sensor[port] = GetBits(1, 0, BrickPi.SensorI2CDevices[port])
				var device byte = 0
				for device < BrickPi.SensorI2CDevices[port] {
					if BrickPi.Sensor[port] & (0x01 << device) {
						var in_byte byte = 0
						for in_byte < BrickPi.SensorI2CRead[port][device] {
							BrickPi.SensorI2CIn[port][device][in_byte] = GetBits(1, 0, 8)
							in_byte++
						}
					}
					device++
				}
				break
			case TYPE_SENSOR_LIGHT_OFF:
			case TYPE_SENSOR_LIGHT_ON:
			case TYPE_SENSOR_RCX_LIGHT:
			case TYPE_SENSOR_COLOR_RED:
			case TYPE_SENSOR_COLOR_GREEN:
			case TYPE_SENSOR_COLOR_BLUE:
			case TYPE_SENSOR_COLOR_NONE:
			default:
				BrickPi.Sensor[(ii + (i * 2))] = GetBits(1, 0, 10)
			}
			ii++
		}
		i++
	}
	return 0
}

var UART_file_descriptor int32 = 0

func BrickPiSetup() int32 {
	UART_file_descriptor = serialOpen("/dev/ttyAMA0", 500000)
	if UART_file_descriptor == -1 {
		return -1
	}
	return 0
}

func BrickPiTx(dest byte, ByteCount byte, OutArray []byte) {
	var tx_buffer [256]byte
	tx_buffer[0] = dest
	tx_buffer[1] = dest + ByteCount
	tx_buffer[2] = ByteCount
	var i byte = 0
	for i < ByteCount {
		tx_buffer[1] += OutArray[i]
		tx_buffer[i+3] = OutArray[i]
		i++
	}
	i = 0
	for i < (ByteCount + 3) {
		serialPutchar(UART_file_descriptor, tx_buffer[i])
		i++
	}
}

func BrickPiRx(InBytes *byte, InArray *byte, timeout int32) int32 { // timeout in uS, not mS
	var rx_buffer [256]byte
	var RxBytes byte = 0
	var CheckSum byte = 0
	var i byte = 0
	var result int32
	var OrigionalTick uint32 = CurrentTickUs()
	for serialDataAvail(UART_file_descriptor) <= 0 {
		if timeout && ((CurrentTickUs() - OrigionalTick) >= timeout) {
			return -2
		}
	}

	RxBytes = 0
	for RxBytes < serialDataAvail(UART_file_descriptor) { // If it's been 1 ms since the last data was received, assume it's the end of the message.
		RxBytes = serialDataAvail(UART_file_descriptor)
		usleep(75)
	}

	i = 0
	for i < RxBytes {
		result = serialGetchar(UART_file_descriptor)
		if result >= 0 {
			rx_buffer[i] = result
		} else {
			return -1
		}
		i++
	}

	if RxBytes < 2 {
		return -4
	}

	if RxBytes < (rx_buffer[1] + 2) {
		return -6
	}

	CheckSum = rx_buffer[1]

	i = 0
	for i < (RxBytes - 2) {
		CheckSum += rx_buffer[i+2]
		InArray[i] = rx_buffer[i+2]
		i++
	}

	if CheckSum != rx_buffer[0] {
		return -5
	}

	*InBytes = (RxBytes - 2)

	return 0
}
