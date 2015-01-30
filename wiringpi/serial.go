package wiringpi

// #cgo LDFLAGS: -lwiringPi -lrt
// #include <stdlib.h>
// #include <unistd.h>
// #include <wiringSerial.h>
import "C"

import (
	"errors"
	"unsafe"
)

type Serial struct {
	fd C.int
}

func SerialOpen(path string, baudRate int32) (*Serial, error) {
	cPath := C.CString(path)
	defer C.free(unsafe.Pointer(cPath))
	s := Serial {fd: C.serialOpen(cPath, C.int(baudRate))}
	if s.fd == -1 {
		return nil, errors.New("Could not open device")
	}
	return &s, nil
}

func (s *Serial) Close() {
	C.serialClose(s.fd)
}

func (s *Serial) PutByte(b byte) {
	C.serialPutchar(s.fd, C.uchar(b))
}

func (s *Serial) PutBytes(b []byte) {
	C.write(s.fd, unsafe.Pointer(&b[0]), C.size_t(len(b)))
}

func (s *Serial) DataAvail() int {
	avail := C.serialDataAvail(s.fd)
	if avail == -1 {
		return -1
	}
	return int(avail)
}

func (s *Serial) GetByte() (byte, error) {
	c := C.serialGetchar(s.fd)
	if c == -1 {
		return 0, errors.New("No data")
	}
	return byte(c), nil
}

