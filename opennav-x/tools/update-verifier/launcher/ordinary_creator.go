package launcher

import (
	"errors"
	"io"
)

// These interfaces describe the production ordinary creator's custody boundary,
// not wait/progress children or the supervisor's health-lifetime protocol.
type ordinaryImage interface {
	io.Closer
	recheck() error
	create(processSpec) (ordinaryChild, error)
}

type ordinaryChild interface {
	io.Closer
	corroborate() error
	resume() (uint32, error)
	abortSuspended() error
}

func startOrdinary(spec processSpec, acquire func(processSpec) (ordinaryImage, error)) error {
	if !hashPattern.MatchString(spec.expectedImageHash) || len(spec.input) != 0 || spec.limit != 0 {
		return errors.New("ordinary application requires a managed image hash")
	}
	image, err := acquire(spec)
	if err != nil {
		return err
	}
	defer image.Close()
	if err := image.recheck(); err != nil {
		return err
	}
	// There is exactly one native create attempt. An error or uncertainty never
	// authorizes another creator, another path, or an ordinary fallback.
	child, err := image.create(spec)
	if err != nil {
		return err
	}
	defer child.Close()
	if err := child.corroborate(); err != nil {
		return errors.Join(err, child.abortSuspended())
	}
	if err := image.recheck(); err != nil {
		return errors.Join(err, child.abortSuspended())
	}
	// resume marks its attempt before entering the OS. Never terminate or retry
	// after that call, even if the result cannot establish successful execution.
	count, err := child.resume()
	if err != nil || count != 1 {
		return errors.New("ordinary application resume was not confirmed; no retry")
	}
	return nil
}
