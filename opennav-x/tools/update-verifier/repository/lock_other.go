//go:build !linux

package repository

import "errors"

func lock(string) (func(), error) { return nil, errors.New("operator mutation requires Linux") }
