package packet

import (
	"encoding/hex"
	"errors"
	"strings"
	"testing"
)

func mustHex(t *testing.T, s string) []byte {
	t.Helper()
	b, err := hex.DecodeString(strings.ReplaceAll(s, " ", ""))
	if err != nil {
		t.Fatal(err)
	}
	return b
}

// Vectors are the precomputed frames from the protocol repository's
// examples/robot-driver.sh, all device 1 command 4 (set wheel speeds).
var driverVectors = []struct {
	name        string
	left, right int32
	frame       string
}{
	{"forward", 100, 100, "01 04 00 00000064 00000064 0000000000000000 D1"},
	{"back", -100, -100, "01 04 00 FFFFFF9C FFFFFF9C 0000000000000000 71"},
	{"left", 0, 100, "01 04 00 00000000 00000064 0000000000000000 8A"},
	{"right", 100, 0, "01 04 00 00000064 00000000 0000000000000000 25"},
	{"stop", 0, 0, "01 04 00 00000000 00000000 0000000000000000 7E"},
}

func TestDriverVectors(t *testing.T) {
	for _, v := range driverVectors {
		t.Run(v.name, func(t *testing.T) {
			want := mustHex(t, v.frame)
			payload := []byte{
				byte(v.left >> 24), byte(v.left >> 16), byte(v.left >> 8), byte(v.left),
				byte(v.right >> 24), byte(v.right >> 16), byte(v.right >> 8), byte(v.right),
			}
			got := New(1, 4, 0, payload).Bytes()
			if string(got[:]) != string(want) {
				t.Fatalf("Bytes() = % X\nwant      % X", got, want)
			}
			p, err := Parse(want)
			if err != nil {
				t.Fatal(err)
			}
			if p.Dev != 1 || p.Cmd != 4 || p.ID != 0 || string(p.Payload[:8]) != string(payload) {
				t.Fatalf("Parse() = %v", p)
			}
		})
	}
}

func TestCRC8CheckValue(t *testing.T) {
	// Standard check value for CRC-8 (poly 0x07, init 0, no reflection).
	if got := CRC8([]byte("123456789")); got != 0xF4 {
		t.Fatalf("CRC8 = %#x, want 0xf4", got)
	}
}

func TestParseErrors(t *testing.T) {
	good := New(5, 4, 9, []byte("hello")).Bytes()
	if _, err := Parse(good[:19]); !errors.Is(err, ErrSize) {
		t.Errorf("short frame: err = %v, want ErrSize", err)
	}
	bad := good
	bad[5] ^= 0x01
	if _, err := Parse(bad[:]); !errors.Is(err, ErrCRC) {
		t.Errorf("corrupt frame: err = %v, want ErrCRC", err)
	}
}

func TestNewPanicsOnLongPayload(t *testing.T) {
	defer func() {
		if recover() == nil {
			t.Fatal("expected panic")
		}
	}()
	New(0, 0, 0, make([]byte, PayloadSize+1))
}
