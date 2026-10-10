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

func TestParseRobotState(t *testing.T) {
	if s := parseRobotState([]byte{0x00, 0x5a}); s == nil || s.Touched() || s.BatteryPercent != 90 {
		t.Fatalf("idle state = %+v", s)
	}
	s := parseRobotState([]byte{0x6A, 0x32}) // cliff, left bump, rear left, front left
	want := RobotState{FrontLeft: true, RearLeft: true, LeftBumper: true, Cliff: true, BatteryPercent: 50}
	if s == nil || *s != want || !s.Touched() {
		t.Fatalf("parseRobotState = %+v, want %+v", s, want)
	}
	if parseRobotState([]byte{1}) != nil {
		t.Fatal("accepted a 1-byte state")
	}
}
