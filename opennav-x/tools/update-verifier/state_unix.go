//go:build !windows

package verifier

import (
	"errors"
	"golang.org/x/sys/unix"
	"os"
	"path/filepath"
	"syscall"
)

func makePrivateStateDirectory(path string) error { return os.Mkdir(path, 0700) }
func assertPrivateStateDirectory(path string) error {
	s, err := os.Lstat(path)
	if err != nil {
		return err
	}
	u, ok := s.Sys().(*syscall.Stat_t)
	if !ok || !s.IsDir() || s.Mode().Perm() != 0700 || u.Uid != uint32(os.Getuid()) {
		return errors.New("update directory must be owned and private")
	}
	return nil
}
func rejectStateReparse(string) error { return nil }
func lockState(directory string) (func(), error) {
	p := filepath.Join(directory, "state.lock")
	f, err := os.OpenFile(p, os.O_CREATE|os.O_RDWR|syscall.O_NOFOLLOW, 0600)
	if err != nil {
		return nil, err
	}
	if err := unix.Flock(int(f.Fd()), unix.LOCK_EX|unix.LOCK_NB); err != nil {
		f.Close()
		return nil, errors.New("update state is busy")
	}
	return func() { _ = unix.Flock(int(f.Fd()), unix.LOCK_UN); _ = f.Close() }, nil
}
func replaceStateFile(from, to string) error { return os.Rename(from, to) }
func syncStateDirectory(path string) error {
	f, err := os.Open(path)
	if err != nil {
		return err
	}
	defer f.Close()
	return f.Sync()
}
