package brickpi

/*
*  This is a library of functions for the RPi to communicate with the BrickPi.
*
*  Based on BrickPi_C (Matthew Richardson)
*  matthewrichardson37<at>gmail.com
*  http://mattallen37.wordpress.com/
 */

// #cgo LDFLAGS: -lwiringPi -lrt
// #include <stdlib.h>
// #include <wiringSerial.h>
// #include "tick.h"
import "C"
import "unsafe"

import (
	"log"
	"time"
)

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

var BrickPi = new(BrickPiStruct)

var array [256]byte
var bytesReceived byte

func ChangeAddress(oldAddr byte, newAddr byte) int32 {
	array[BYTE_MSG_TYPE] = MSG_TYPE_CHANGE_ADDR
	array[BYTE_NEW_ADDRESS] = newAddr
	tx(oldAddr, 2, array)

	if rx(&bytesReceived, &array, 5000) != 0 {
		return -1
	}
	if !(bytesReceived == 1 && array[BYTE_MSG_TYPE] == MSG_TYPE_CHANGE_ADDR) {
		return -1
	}

	return 0
}

func SetTimeout() int32 {
	var i byte = 0
	for i < 2 {
		array[BYTE_MSG_TYPE] = MSG_TYPE_TIMEOUT_SETTINGS
		array[BYTE_TIMEOUT] = byte(BrickPi.Timeout & 0xFF)
		array[(BYTE_TIMEOUT + 1)] = byte((BrickPi.Timeout / 256) & 0xFF)
		array[(BYTE_TIMEOUT + 2)] = byte((BrickPi.Timeout / 65536) & 0xFF)
		array[(BYTE_TIMEOUT + 3)] = byte((BrickPi.Timeout / 16777216) & 0xFF)
		tx(BrickPi.Address[i], 5, array)
		if rx(&bytesReceived, &array, 2500) != 0 {
			return -1
		}
		if !(bytesReceived == 1 && array[BYTE_MSG_TYPE] == MSG_TYPE_TIMEOUT_SETTINGS) {
			return -1
		}
		i++
	}
	return 0
}

var bitOffset uint32 = 0

func addBits(byte_offset byte, bit_offset byte, bits byte, value uint32) {
	var i byte = 0
	for i < bits {
		if value&0x01 != 0 {
			array[(uint32(byte_offset) + ((uint32(bit_offset) + bitOffset + uint32(i)) / 8))] |= byte(0x01 << ((uint32(bit_offset) + bitOffset + uint32(i)) % 8))
		}
		value /= 2
		i++
	}
	bitOffset += uint32(bits)
}

func getBits(byte_offset uint32, bit_offset uint32, bits uint32) uint32 {
	var result uint32 = 0
	i := bits
	for i != 0 {
		result *= 2
		result |= ((uint32(array[(byte_offset+((bit_offset+bitOffset+(i-1))/8))]) >> ((bit_offset + bitOffset + (i - 1)) % 8)) & 0x01)
		i--
	}
	bitOffset += bits
	return result
}

func BitsNeeded(value uint32) byte {
	var i byte = 0
	for i < 32 {
		if value == 0 {
			return i
		}
		value /= 2
		i++
	}
	return 31
}

func SetupSensors() int32 {
	var i byte = 0
	for i < 2 {
		var ii byte = 0
		for ii <= 255 {
			array[ii] = 0
			ii++
		}
		bitOffset = 0
		array[BYTE_MSG_TYPE] = MSG_TYPE_SENSOR_TYPE
		array[BYTE_SENSOR_1_TYPE] = BrickPi.SensorType[PORT_1+(i*2)]
		array[BYTE_SENSOR_2_TYPE] = BrickPi.SensorType[PORT_2+(i*2)]
		ii = 0
		for ii < 2 {
			var port byte = (i * 2) + ii
			if array[BYTE_SENSOR_1_TYPE+ii] == TYPE_SENSOR_I2C ||
				array[BYTE_SENSOR_1_TYPE+ii] == TYPE_SENSOR_I2C_9V {
				addBits(3, 0, 8, uint32(BrickPi.SensorI2CSpeed[port]))

				if BrickPi.SensorI2CDevices[port] > 8 {
					BrickPi.SensorI2CDevices[port] = 8
				}

				if BrickPi.SensorI2CDevices[port] == 0 {
					BrickPi.SensorI2CDevices[port] = 1
				}

				addBits(3, 0, 3, uint32(BrickPi.SensorI2CDevices[port]-1))

				var device byte = 0
				for device < BrickPi.SensorI2CDevices[port] {
					addBits(3, 0, 7, uint32(BrickPi.SensorI2CAddr[port][device]>>1))
					addBits(3, 0, 2, uint32(BrickPi.SensorSettings[port][device]))
					if BrickPi.SensorSettings[port][device]&BIT_I2C_SAME != 0 {
						addBits(3, 0, 4, uint32(BrickPi.SensorI2CWrite[port][device]))
						addBits(3, 0, 4, uint32(BrickPi.SensorI2CRead[port][device]))
						var outByte byte = 0
						for outByte < BrickPi.SensorI2CWrite[port][device] {
							addBits(3, 0, 8, uint32(BrickPi.SensorI2COut[port][device][outByte]))
							outByte++
						}
					}
					device++
				}
			}
			ii++
		}
		var txBytes = byte(((bitOffset + 7) / 8) + 3)
		tx(BrickPi.Address[i], txBytes, array)
		if rx(&bytesReceived, &array, 500000) != 0 {
			return -1
		}
		if !(bytesReceived == 1 && array[BYTE_MSG_TYPE] == MSG_TYPE_SENSOR_TYPE) {
			return -1
		}
		i++
	}
	return 0
}

var retried byte = 0 // For re-trying a failed update.

func UpdateValues() int32 {
	var i byte = 0
	var ii byte = 0
	for i < 2 {
		retried = 0

	__RETRY_COMMUNICATION__:

		ii = 0
		for ii <= 255 {
			array[ii] = 0
			ii++
		}

		array[BYTE_MSG_TYPE] = MSG_TYPE_VALUES

		bitOffset = 0

		//    addBits(1, 0, 2, 0);     use this to disable encoder offset

		ii = 0 // use this for encoder offset support
		for ii < 2 {
			var port byte = (i * 2) + ii
			if BrickPi.EncoderOffset[port] != 0 {
				var tempValue = BrickPi.EncoderOffset[port]
				var tempEncDir byte
				var tempBitsNeeded byte

				addBits(1, 0, 1, 1)
				for tempValue < 0 {
					tempEncDir = 1
					tempValue *= (-1)
				}
				tempBitsNeeded = (BitsNeeded(uint32(tempValue)) + 1)
				addBits(1, 0, 5, uint32(tempBitsNeeded))
				tempValue *= 2
				tempValue |= int32(tempEncDir)
				addBits(1, 0, tempBitsNeeded, uint32(tempValue))
			} else {
				addBits(1, 0, 1, 0)
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
			addBits(1, 0, 10, uint32((((speed&0xFF)<<2)|(int32(dir)<<1)|int32(BrickPi.MotorEnable[port]&0x01))&0x3FF))
			ii++
		}

		ii = 0
		for ii < 2 {
			var port byte = (i * 2) + ii
			if BrickPi.SensorType[port] == TYPE_SENSOR_I2C ||
				BrickPi.SensorType[port] == TYPE_SENSOR_I2C_9V {
				var device byte = 0
				for device < BrickPi.SensorI2CDevices[port] {
					if (BrickPi.SensorSettings[port][device] & BIT_I2C_SAME) == 0 {
						addBits(1, 0, 4, uint32(BrickPi.SensorI2CWrite[port][device]))
						addBits(1, 0, 4, uint32(BrickPi.SensorI2CRead[port][device]))
						var outByte byte = 0
						for outByte < BrickPi.SensorI2CWrite[port][device] {
							addBits(1, 0, 8, uint32(BrickPi.SensorI2COut[port][device][outByte]))
							outByte++
						}
					}
					device++
				}
			}
			ii++
		}

		var txBytes = byte(((bitOffset + 7) / 8) + 1)
		tx(BrickPi.Address[i], txBytes, array)

		var result = rx(&bytesReceived, &array, 7500)

		if result != -2 { // -2 is the only error that indicates that the BrickPi uC did not properly receive the message
			BrickPi.EncoderOffset[((i * 2) + PORT_A)] = 0
			BrickPi.EncoderOffset[((i * 2) + PORT_B)] = 0
		}

		if result != 0 || (array[BYTE_MSG_TYPE] != MSG_TYPE_VALUES) {
			log.Printf("rx error: %d\n", result)
			if retried < 2 {
				retried++
				goto __RETRY_COMMUNICATION__
			} else {
				log.Printf("Retry failed.\n")
				return -1
			}
		}

		bitOffset = 0

		var tempBitsUsed [2]byte // Used for encoder values
		tempBitsUsed[0] = byte(getBits(1, 0, 5))
		tempBitsUsed[1] = byte(getBits(1, 0, 5))
		var tempEncoderVal uint32

		ii = 0
		for ii < 2 {
			tempEncoderVal = getBits(1, 0, uint32(tempBitsUsed[ii]))
			if tempEncoderVal&0x01 != 0 {
				tempEncoderVal /= 2
				BrickPi.Encoder[ii+(i*2)] = int32(tempEncoderVal) * (-1)
			} else {
				BrickPi.Encoder[ii+(i*2)] = int32(tempEncoderVal / 2)
			}
			ii++
		}

		ii = 0
		for ii < 2 {
			var port byte = ii + (i * 2)
			switch BrickPi.SensorType[port] {
			case TYPE_SENSOR_TOUCH:
				BrickPi.Sensor[port] = int32(getBits(1, 0, 1))
				break
			case TYPE_SENSOR_ULTRASONIC_CONT:
			case TYPE_SENSOR_ULTRASONIC_SS:
				BrickPi.Sensor[port] = int32(getBits(1, 0, 8))
				break
			case TYPE_SENSOR_COLOR_FULL:
				BrickPi.Sensor[port] = int32(getBits(1, 0, 3))
				BrickPi.SensorArray[port][INDEX_BLANK] = int32(getBits(1, 0, 10))
				BrickPi.SensorArray[port][INDEX_RED] = int32(getBits(1, 0, 10))
				BrickPi.SensorArray[port][INDEX_GREEN] = int32(getBits(1, 0, 10))
				BrickPi.SensorArray[port][INDEX_BLUE] = int32(getBits(1, 0, 10))
				break
			case TYPE_SENSOR_I2C:
			case TYPE_SENSOR_I2C_9V:
				BrickPi.Sensor[port] = int32(getBits(1, 0, uint32(BrickPi.SensorI2CDevices[port])))
				var device byte = 0
				for device < BrickPi.SensorI2CDevices[port] {
					if BrickPi.Sensor[port]&(0x01<<device) != 0 {
						var in_byte byte = 0
						for in_byte < BrickPi.SensorI2CRead[port][device] {
							BrickPi.SensorI2CIn[port][device][in_byte] = byte(getBits(1, 0, 8))
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
				BrickPi.Sensor[(ii + (i * 2))] = int32(getBits(1, 0, 10))
			}
			ii++
		}
		i++
	}
	return 0
}

var uartFileDescriptor C.int = 0

func Setup() int32 {
	serialPath := C.CString("/dev/ttyAMA0")
	defer C.free(unsafe.Pointer(serialPath))
	uartFileDescriptor = C.serialOpen(serialPath, C.int(500000))
	if uartFileDescriptor == -1 {
		return -1
	}
	return 0
}

func tx(dest byte, byteCount byte, outArray [256]byte) {
	var txBuffer [256]byte
	txBuffer[0] = dest
	txBuffer[1] = dest + byteCount
	txBuffer[2] = byteCount
	var i byte = 0
	for i < byteCount {
		txBuffer[1] += outArray[i]
		txBuffer[i+3] = outArray[i]
		i++
	}
	i = 0
	for i < (byteCount + 3) {
		C.serialPutchar(uartFileDescriptor, C.uchar(txBuffer[i]))
		i++
	}
}

func rx(inBytes *byte, inArray *[256]byte, timeout int32) int32 { // timeout in uS, not mS
	var rxBuffer [256]byte
	var rxBytes byte = 0
	var checksum byte = 0
	var i byte = 0
	var result byte
	var origionalTick C.ulong = C.CurrentTickUs()
	for C.serialDataAvail(uartFileDescriptor) <= 0 {
		if timeout != 0 && ((C.CurrentTickUs() - origionalTick) >= C.ulong(timeout)) {
			return -2
		}
	}

	rxBytes = 0
	for rxBytes < byte(C.serialDataAvail(uartFileDescriptor)) { // If it's been 1 ms since the last data was received, assume it's the end of the message.
		rxBytes = byte(C.serialDataAvail(uartFileDescriptor))
		time.Sleep(75 * time.Microsecond)
	}

	i = 0
	for i < rxBytes {
		result = byte(C.serialGetchar(uartFileDescriptor))
		if result >= 0 {
			rxBuffer[i] = result
		} else {
			return -1
		}
		i++
	}

	if rxBytes < 2 {
		return -4
	}

	if rxBytes < (rxBuffer[1] + 2) {
		return -6
	}

	checksum = rxBuffer[1]

	i = 0
	for i < (rxBytes - 2) {
		checksum += rxBuffer[i+2]
		inArray[i] = rxBuffer[i+2]
		i++
	}

	if checksum != rxBuffer[0] {
		return -5
	}

	*inBytes = (rxBytes - 2)

	return 0
}
