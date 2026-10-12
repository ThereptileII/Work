package brokerhost

import (
	"context"
	"errors"
)

var errBrokerEndpointStopped = errors.New("broker endpoint stopped unexpectedly")
var errBrokerEndpointPanic = errors.New("broker endpoint failed")

// Both authenticated endpoints share one Store and engine operation lock. A
// terminal endpoint cancels the other, but returning never releases that Store
// until BOTH serving scopes have drained. The outer SCM shutdown deadline may
// exit the own-process service without releasing owners if either is stuck.
func serveBrokerEndpoints(ctx context.Context, staging, execution func(context.Context) error) error {
	if ctx == nil || staging == nil || execution == nil {
		return errors.New("live broker context and both endpoints required")
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	operation, cancel := context.WithCancel(ctx)
	defer cancel()
	results := make(chan error, 2)
	serve := func(endpoint func(context.Context) error) {
		var result error
		defer func() {
			if recover() != nil {
				result = errBrokerEndpointPanic
			}
			results <- result
		}()
		result = endpoint(operation)
	}
	go serve(staging)
	go serve(execution)
	first := <-results
	// An unsolicited successful return is still a lost protected endpoint.
	if first == nil && ctx.Err() == nil {
		first = errBrokerEndpointStopped
	}
	cancel()
	second := <-results
	// Only exact context cancellation is benign. A joined cancellation plus a
	// cleanup/custody error must survive so SCM cannot report a clean shutdown.
	benign := func(err error) error {
		if err == context.Canceled || (ctx.Err() != nil && err == ctx.Err()) {
			return nil
		}
		return err
	}
	if err := errors.Join(benign(first), benign(second)); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	return errBrokerEndpointStopped
}
