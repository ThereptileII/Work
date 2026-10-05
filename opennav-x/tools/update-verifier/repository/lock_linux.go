//go:build linux

package repository

import (
	"errors"
	"os"
	"path/filepath"

	"golang.org/x/sys/unix"
)

func lock(directory string) (func(), error) {
	if e := plain(directory); e != nil {
		return nil, e
	}
	var owner unix.Stat_t
	if e := unix.Stat(directory, &owner); e != nil || owner.Uid != uint32(os.Getuid()) || owner.Mode&0077 != 0 {
		return nil, errors.New("operator directory must be private and owned by this user")
	}
	fd, e := unix.Open(filepath.Join(directory, "operator.lock"), unix.O_RDWR|unix.O_CREAT|unix.O_NOFOLLOW|unix.O_CLOEXEC, 0600)
	if e != nil {
		return nil, errors.New("operator lock unavailable")
	}
	f := os.NewFile(uintptr(fd), "operator-lock")
	st, e := f.Stat()
	if e != nil || !st.Mode().IsRegular() || st.Mode().Perm()&0077 != 0 {
		f.Close()
		return nil, errors.New("unsafe operator lock")
	}
	var fileOwner unix.Stat_t
	if e = unix.Fstat(fd, &fileOwner); e != nil || fileOwner.Uid != uint32(os.Getuid()) || fileOwner.Nlink != 1 {
		f.Close()
		return nil, errors.New("operator lock ownership refused")
	}
	if e = unix.Flock(fd, unix.LOCK_EX|unix.LOCK_NB); e != nil {
		f.Close()
		return nil, errors.New("another publisher owns the operator lock")
	}
	return func() { _ = unix.Flock(fd, unix.LOCK_UN); _ = f.Close() }, nil
}
