package ble

import "testing"

func TestParseModel(t *testing.T) {
	for _, tc := range []struct {
		mfr  []byte
		want string
	}{
		{[]byte{0x00, 0x06, 'R', 'T', '1'}, "RT1"},
		{[]byte{0x00, 0x06, 'R', 'T', '0', 0, 0}, "RT0"},
		{[]byte{0x00, 0x06}, ""},
		{nil, ""},
	} {
		if got := parseModel(tc.mfr); got != tc.want {
			t.Errorf("parseModel(% x) = %q, want %q", tc.mfr, got, tc.want)
		}
	}
}
