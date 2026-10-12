package brokerstore

import (
	"errors"
	"reflect"
	"testing"
)

type recordingFreshCreation struct {
	events                                                      []string
	failAt                                                      int
	fault, cleanup                                              error
	reserved, root, candidates, registrations, users, completed bool
}

func (c *recordingFreshCreation) step(name string) error {
	c.events = append(c.events, name)
	if len(c.events) == c.failAt {
		return c.fault
	}
	return nil
}
func (c *recordingFreshCreation) check() error      { return c.step("check") }
func (c *recordingFreshCreation) rootAbsent() error { return c.step("absent") }
func (c *recordingFreshCreation) reserve() error {
	if err := c.step("reserve-sync"); err != nil {
		return err
	}
	c.reserved = true
	return nil
}
func (c *recordingFreshCreation) createRoot() error {
	if !c.reserved {
		panic("root before durable fence")
	}
	if err := c.step("root"); err != nil {
		return err
	}
	c.root = true
	return nil
}
func (c *recordingFreshCreation) createCandidates() error {
	if err := c.step("candidates"); err != nil {
		return err
	}
	c.candidates = true
	return nil
}
func (c *recordingFreshCreation) createRegistrations() error {
	if err := c.step("registrations"); err != nil {
		return err
	}
	c.registrations = true
	return nil
}
func (c *recordingFreshCreation) createUsers() error {
	if err := c.step("users"); err != nil {
		return err
	}
	c.users = true
	return nil
}
func (c *recordingFreshCreation) complete() error {
	if !(c.reserved && c.root && c.candidates && c.registrations && c.users) {
		panic("completion before validated tree")
	}
	if err := c.step("complete-sync"); err != nil {
		return err
	}
	c.completed = true
	return nil
}
func (c *recordingFreshCreation) close() error {
	c.events = append(c.events, "close")
	return c.cleanup
}

func TestFreshCreationReservationAndCompletionOrdering(t *testing.T) {
	c := &recordingFreshCreation{}
	if err := initializeFresh(c); err != nil {
		t.Fatal(err)
	}
	want := []string{"check", "absent", "check", "reserve-sync", "check", "root", "check", "candidates", "check", "registrations", "check", "users", "check", "complete-sync", "check", "close"}
	if !reflect.DeepEqual(c.events, want) {
		t.Fatalf("operations = %v; want %v", c.events, want)
	}
	if !c.completed {
		t.Fatal("completion missing")
	}
}

func TestFreshCreationEveryFaultStopsAndRetainsStateAndCleanupError(t *testing.T) {
	fault := errors.New("injected operation failure")
	cleanup := errors.New("uncertain handle cleanup")
	for failure := 1; failure <= 15; failure++ {
		c := &recordingFreshCreation{failAt: failure, fault: fault, cleanup: cleanup}
		err := initializeFresh(c)
		if !errors.Is(err, fault) || !errors.Is(err, cleanup) {
			t.Fatalf("fault %d lost evidence: %v", failure, err)
		}
		if len(c.events) != failure+1 || c.events[len(c.events)-1] != "close" {
			t.Fatalf("fault %d continued: %v", failure, c.events)
		}
		if c.reserved != (failure > 4) || c.root != (failure > 6) || c.candidates != (failure > 8) || c.registrations != (failure > 10) || c.users != (failure > 12) || c.completed != (failure > 14) {
			t.Fatalf("fault %d erased or prematurely committed durable state: %+v", failure, c)
		}
	}
	c := &recordingFreshCreation{cleanup: cleanup}
	if err := initializeFresh(c); !errors.Is(err, cleanup) || !c.completed {
		t.Fatalf("successful creation hid cleanup uncertainty: %v %+v", err, c)
	}
}
