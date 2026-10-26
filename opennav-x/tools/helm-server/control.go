package helm

import (
	"bufio"
	"context"
	"encoding/json"
	"io"
	"sync"
)

// Control channel with SKAGER
//
// SKAGER starts the Helm Server as a child process and talks to it over stdin
// and stdout, one JSON object per line. When SKAGER exits, stdin closes and
// Helm stops: no Helm server outlives the SKAGER that started it.
//
// SKAGER → Helm commands:
//
//	{"cmd":"pair"}                        open the pairing window (answer: "pairing")
//	{"cmd":"pair_cancel"}
//	{"cmd":"devices"}                     (answer: "devices")
//	{"cmd":"revoke","device":"<id>"}      remove a paired phone, ending its session
//	{"cmd":"disconnect"}                  end the current session
//	{"cmd":"name","name":"<boat>"}
//	{"cmd":"pilot","status":{…}}          autopilot status shown on the phone (read-only)
//	{"cmd":"quit"}
//
// Helm → SKAGER events: ready, pairing, pairing_closed, paired, devices,
// session, screen, input, error (see Event).

type command struct {
	Cmd    string          `json:"cmd"`
	Device string          `json:"device"`
	Name   string          `json:"name"`
	Status json.RawMessage `json:"status"`
}

// EventWriter serializes events as JSON lines.
type EventWriter struct {
	mu sync.Mutex
	w  io.Writer
}

func NewEventWriter(w io.Writer) *EventWriter { return &EventWriter{w: w} }

func (e *EventWriter) Write(ev Event) {
	data, err := json.Marshal(ev)
	if err != nil {
		return
	}
	e.mu.Lock()
	defer e.mu.Unlock()
	_, _ = e.w.Write(append(data, '\n'))
}

// RunControl reads SKAGER's commands until stdin closes or "quit", then cancels.
func RunControl(ctx context.Context, cancel context.CancelFunc, s *Server, in io.Reader, emit func(Event)) {
	defer cancel()
	scanner := bufio.NewScanner(in)
	scanner.Buffer(make([]byte, 64<<10), 64<<10)
	for scanner.Scan() {
		if ctx.Err() != nil {
			return
		}
		var c command
		if json.Unmarshal(scanner.Bytes(), &c) != nil {
			emit(Event{Event: "error", Message: "unreadable command"})
			continue
		}
		switch c.Cmd {
		case "pair":
			ev, err := s.OpenPairing()
			if err != nil {
				emit(Event{Event: "error", Message: err.Error()})
				continue
			}
			emit(ev)
		case "pair_cancel":
			s.ClosePairing()
			emit(Event{Event: "pairing_closed", Reason: "cancelled"})
		case "devices":
			s.EmitDevices()
		case "revoke":
			s.Revoke(c.Device)
		case "disconnect":
			s.EndSession("ended_on_skager")
		case "name":
			s.SetName(c.Name)
		case "pilot":
			if len(c.Status) > 0 && len(c.Status) < 2048 && json.Valid(c.Status) {
				s.SetPilot(c.Status)
			}
		case "quit":
			return
		default:
			emit(Event{Event: "error", Message: "unknown command"})
		}
	}
}
