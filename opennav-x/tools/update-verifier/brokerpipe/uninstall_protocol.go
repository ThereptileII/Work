package brokerpipe

import (
	"context"
	"encoding/json"
	"errors"
	"sync"
	"sync/atomic"
	"time"
)

const protectedUninstallTimeout = 90 * time.Second
const protectedUninstallExchangeTimeout = 30 * time.Second

// The interim result is comparison data from the authenticated live broker.
// It permits the client callback to restore original shortcuts and drain its
// child handles, never SCM effects or file retirement by itself.
func validUninstallFenceResult(r MaintenanceResult, q MaintenanceRequest) bool {
	if !validMaintenanceRequest(q) || q.Operation != PrepareUninstall || r.State != "uninstall-fenced" || r.RetirementSHA256 != "" {
		return false
	}
	final := r
	final.State = "uninstall-prepared"
	final.RetirementSHA256 = "0000000000000000000000000000000000000000000000000000000000000000"
	return validDecommissionResult(final, q)
}

func decodeUninstallFenceResult(raw []byte, q MaintenanceRequest) (MaintenanceResult, error) {
	if len(raw) > maxFrame {
		return MaintenanceResult{}, ErrRefused
	}
	fields := []string{"schema", "requestId", "state", "generation", "completionSha256", "selectedHeadSha256", "clientSid", "registrationSha256", "currentProfileSha256", "currentProfileIdentitySha256", "fenceSha256"}
	if _, err := exactObject(raw, fields, false); err != nil {
		return MaintenanceResult{}, ErrRefused
	}
	var r MaintenanceResult
	if json.Unmarshal(raw, &r) != nil || !validUninstallFenceResult(r, q) {
		return MaintenanceResult{}, ErrRefused
	}
	return r, nil
}

func sameUninstallFence(final, interim MaintenanceResult) bool {
	return final.Schema == interim.Schema && final.RequestID == interim.RequestID &&
		final.Generation == interim.Generation && final.CompletionSHA256 == interim.CompletionSHA256 &&
		final.SelectedHeadSHA256 == interim.SelectedHeadSHA256 && final.ClientSID == interim.ClientSID &&
		final.RegistrationSHA256 == interim.RegistrationSHA256 &&
		final.CurrentProfileSHA256 == interim.CurrentProfileSHA256 &&
		final.CurrentProfileIdentitySHA256 == interim.CurrentProfileIdentitySHA256 &&
		final.FenceSHA256 == interim.FenceSHA256
}

func uninstallContinue(q MaintenanceRequest, fence string) []byte {
	b, _ := json.Marshal(struct {
		Schema    int    `json:"schema"`
		Kind      string `json:"kind"`
		RequestID string `json:"requestId"`
		Fence     string `json:"fenceSha256"`
	}{1, "uninstall-continue", q.RequestID, fence})
	return b
}

func validUninstallContinue(raw []byte, q MaintenanceRequest, fence string) bool {
	if len(raw) > maxFrame || !validMaintenanceRequest(q) || q.Operation != PrepareUninstall {
		return false
	}
	if _, err := exactObject(raw, []string{"schema", "kind", "requestId", "fenceSha256"}, false); err != nil {
		return false
	}
	var v struct {
		Schema    int    `json:"schema"`
		Kind      string `json:"kind"`
		RequestID string `json:"requestId"`
		Fence     string `json:"fenceSha256"`
	}
	return json.Unmarshal(raw, &v) == nil && v.Schema == 1 && v.Kind == "uninstall-continue" &&
		v.RequestID == q.RequestID && v.Fence == fence && hex64.MatchString(fence)
}

// The native caller retains the physical pipe/process owners. These helpers
// enforce the same bounded sequence in portable tests without minting native
// authority. check must inspect the original owners directly: the host may hold
// the peer capability mutex throughout its handler and publishFence callback.
func invokeProtectedUninstallHandler(ctx context.Context, peer Peer, q MaintenanceRequest, handler ProtectedUninstallHandler, connection executionConnection, check func(context.Context) error, incoming *atomic.Bool) (MaintenanceResult, error) {
	if ctx == nil || ctx.Err() != nil || !validMaintenanceRequest(q) || q.Operation != PrepareUninstall || handler == nil || connection == nil || check == nil || incoming == nil {
		return MaintenanceResult{}, ErrRefused
	}
	ctx, cancel := context.WithCancel(ctx)
	defer cancel()
	var mu sync.Mutex
	var active atomic.Bool
	active.Store(true)
	published := false
	var interim MaintenanceResult
	var failure error
	publish := func(useCtx context.Context, result MaintenanceResult) error {
		mu.Lock()
		defer mu.Unlock()
		fail := func(err error) error {
			failure = errors.Join(ErrRefused, failure, err)
			cancel()
			return failure
		}
		if !active.Load() || published || failure != nil || useCtx == nil || !validUninstallFenceResult(result, q) || result.ClientSID != peer.SID {
			return fail(ErrRefused)
		}
		published = true // Consume the attempt even when publication/read fails.
		if err := errors.Join(ctx.Err(), useCtx.Err(), check(ctx)); err != nil {
			return fail(err)
		}
		exchange, stop := context.WithTimeout(ctx, protectedUninstallExchangeTimeout)
		defer stop()
		// A derived handler context may shorten but never extend the owner.
		stopUse := context.AfterFunc(useCtx, stop)
		defer stopUse()
		raw, err := json.Marshal(result)
		if err != nil {
			return fail(err)
		}
		incoming.Store(true) // Arm before the client can observe its exact fence.
		defer incoming.Store(false)
		if err := connection.write(exchange, raw); err != nil {
			return fail(err)
		}
		raw, err = connection.read(exchange)
		incoming.Store(false)
		if err != nil || !validUninstallContinue(raw, q, result.FenceSHA256) {
			return fail(errors.Join(ErrRefused, err, exchange.Err()))
		}
		if err := errors.Join(exchange.Err(), useCtx.Err(), check(exchange)); err != nil {
			return fail(err)
		}
		interim = result
		return nil
	}
	// Revoke publication on all exits, including a handler panic. Cancellation
	// releases any concurrent bounded read before we wait for its state lock.
	defer func() {
		active.Store(false)
		cancel()
		mu.Lock() // Join any publisher before native owners can unwind.
		mu.Unlock()
	}()
	result, err := handler(ctx, peer, q, publish)
	active.Store(false)
	// An asynchronously retained publisher cannot outlive the handler. Cancel
	// its reader before joining; an in-progress exchange withholds any final.
	if !mu.TryLock() {
		cancel()
		mu.Lock()
	}
	complete := published && failure == nil && validUninstallFenceResult(interim, q)
	stateErr := failure
	mu.Unlock()
	if err != nil || !complete || !validDecommissionResult(result, q) || !sameUninstallFence(result, interim) {
		return MaintenanceResult{}, errors.Join(ErrRefused, err, stateErr)
	}
	if err := errors.Join(ctx.Err(), check(ctx)); err != nil {
		return MaintenanceResult{}, errors.Join(ErrRefused, err)
	}
	return result, nil
}

type uninstallFenceBackend struct{ validate func(context.Context) error }

func (b uninstallFenceBackend) check(ctx context.Context) error { return b.validate(ctx) }
func (uninstallFenceBackend) close() error                      { return ErrRefused }

func callProtectedUninstallConversation(ctx context.Context, q MaintenanceRequest, expectedSID string, pid uint32, creation uint64, connection executionConnection, check func(context.Context) error, incoming *atomic.Bool, restoreAndDrain func(context.Context, *UninstallFenceWitness) error) (MaintenanceResult, error) {
	if ctx == nil || ctx.Err() != nil || q.Operation != PrepareUninstall || connection == nil || check == nil || incoming == nil || restoreAndDrain == nil {
		return MaintenanceResult{}, ErrRefused
	}
	raw, err := encodeMaintenanceRequest(q)
	if err != nil || check(ctx) != nil {
		return MaintenanceResult{}, ErrRefused
	}
	incoming.Store(true)
	defer incoming.Store(false)
	handshake, stop := context.WithTimeout(ctx, handshakeTimeout)
	err = connection.write(handshake, raw)
	stop()
	if err != nil {
		return MaintenanceResult{}, errors.Join(ErrRefused, err)
	}
	raw, err = connection.read(ctx)
	incoming.Store(false)
	if err != nil {
		return MaintenanceResult{}, errors.Join(ErrRefused, err)
	}
	interim, err := decodeUninstallFenceResult(raw, q)
	if err != nil || interim.ClientSID != expectedSID || check(ctx) != nil {
		return MaintenanceResult{}, ErrRefused
	}
	callbackCtx, cancel := context.WithTimeout(ctx, protectedUninstallExchangeTimeout)
	defer cancel()
	witness, err := newUninstallFenceWitness(callbackCtx, q, interim, pid, creation, uninstallFenceBackend{check})
	if err != nil {
		return MaintenanceResult{}, err
	}
	err = func() error {
		defer witness.revoke()
		if err := witness.Check(callbackCtx, pid, creation); err != nil {
			return err
		}
		return errors.Join(restoreAndDrain(callbackCtx, witness), witness.Check(callbackCtx, pid, creation))
	}()
	if err != nil || callbackCtx.Err() != nil || check(ctx) != nil {
		return MaintenanceResult{}, errors.Join(ErrRefused, err, callbackCtx.Err())
	}
	incoming.Store(true)
	err = connection.write(callbackCtx, uninstallContinue(q, interim.FenceSHA256))
	cancel()
	if err != nil {
		return MaintenanceResult{}, errors.Join(ErrRefused, err)
	}
	raw, err = connection.read(ctx)
	incoming.Store(false)
	if err != nil {
		return MaintenanceResult{}, errors.Join(ErrRefused, err)
	}
	result, err := decodeMaintenanceResult(raw, q)
	if err != nil || !sameUninstallFence(result, interim) || check(ctx) != nil || ctx.Err() != nil {
		return MaintenanceResult{}, ErrRefused
	}
	return result, nil
}
