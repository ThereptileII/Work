//go:build !windows

package brokerhost

import (
	"context"
	"errors"
)

func Run(context.Context) error { return errors.New("native Windows broker service required") }
