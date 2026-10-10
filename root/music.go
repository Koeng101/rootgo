package root

import (
	"fmt"
	"math"
	"strconv"
	"strings"
)

var noteSemitones = map[byte]int{'C': 0, 'D': 2, 'E': 4, 'F': 5, 'G': 7, 'A': 9, 'B': 11}

// NoteHz returns the equal-tempered frequency (A4 = 440 Hz) of a note
// written as a letter, an optional '#' or 'b', and an octave: "A4", "C#5",
// "Bb3".
func NoteHz(note string) (float64, error) {
	s := strings.TrimSpace(note)
	if len(s) < 2 {
		return 0, fmt.Errorf("root: bad note %q", note)
	}
	semi, ok := noteSemitones[byte(strings.ToUpper(s[:1])[0])]
	if !ok {
		return 0, fmt.Errorf("root: bad note %q", note)
	}
	s = s[1:]
	switch s[0] {
	case '#':
		semi++
		s = s[1:]
	case 'b':
		semi--
		s = s[1:]
	}
	octave, err := strconv.Atoi(s)
	if err != nil || octave < 0 || octave > 9 {
		return 0, fmt.Errorf("root: bad note %q", note)
	}
	midi := (octave+1)*12 + semi
	return 440 * math.Pow(2, float64(midi-69)/12), nil
}
