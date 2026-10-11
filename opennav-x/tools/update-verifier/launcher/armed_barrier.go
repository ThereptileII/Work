package launcher

import (
	"context"
	"errors"
)

const armedUpdateMarker = "signed-update-armed.jsonl"

var errArmedUpdateBarrier = errors.New("armed update barrier prevents startup")

func requireUnarmed(p platform, root string) error {
	if err := p.armedUpdateAbsent(root); err != nil {
		return errors.Join(errArmedUpdateBarrier, err)
	}
	return nil
}

// The fixed marker is a denial signal only. Its bytes and claimed owner never
// authorize startup, recovery or trust initialization. Proven absence is a
// point-in-time observation, not a pinned absence or an enabled signed broker.
func waitUnarmed(ctx context.Context, p platform, root string, spec processSpec) (processResult, error) {
	if err := requireUnarmed(p, root); err != nil {
		return processResult{}, err
	}
	return p.wait(ctx, spec)
}

func startUnarmed(p platform, root string, spec processSpec) error {
	if err := requireUnarmed(p, root); err != nil {
		return err
	}
	return p.start(spec)
}

// Managed native ordinary creation retains the existing image/child custody
// state machine. Check after its held-image recheck both before create and
// before resume, where refusal can still abort an owned suspended child.
type unarmedOrdinaryImage struct {
	ordinaryImage
	platform platform
	root     string
}

func (image unarmedOrdinaryImage) recheck() error {
	if err := image.ordinaryImage.recheck(); err != nil {
		return err
	}
	return requireUnarmed(image.platform, image.root)
}
