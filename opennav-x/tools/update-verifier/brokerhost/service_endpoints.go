package brokerhost

import (
	"context"
	"errors"
)

var errBrokerEndpointStopped = errors.New("broker endpoint stopped unexpectedly")
var errBrokerEndpointPanic = errors.New("broker endpoint failed")

// Authenticated endpoints share one Store and engine operation lock. A terminal
// endpoint cancels every peer, but returning never releases that Store until ALL
// serving scopes drain. The SCM deadline may exit the own-process service without
// releasing owners if a handler remains stuck.
func serveBrokerEndpoints(ctx context.Context, staging, execution func(context.Context) error, additional ...func(context.Context) error) error {
	if ctx == nil || staging == nil || execution == nil {
		return errors.New("live broker context and both endpoints required")
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	endpoints := append([]func(context.Context) error{staging, execution}, additional...)
	for _, endpoint := range endpoints {
		if endpoint == nil {
			return errors.New("all protected endpoints required")
		}
	}
	operation, cancel := context.WithCancel(ctx)
	defer cancel()
	results := make(chan error, len(endpoints))
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
	for _, endpoint := range endpoints {
		go serve(endpoint)
	}
	first := <-results
	// An unsolicited successful return is still a lost protected endpoint.
	if first == nil && ctx.Err() == nil {
		first = errBrokerEndpointStopped
	}
	cancel()
	// Only exact context cancellation is benign. A joined cancellation plus a
	// cleanup/custody error must survive so SCM cannot report a clean shutdown.
	benign := func(err error) error {
		if err == context.Canceled || (ctx.Err() != nil && err == ctx.Err()) {
			return nil
		}
		return err
	}
	result := benign(first)
	for range len(endpoints) - 1 {
		result = errors.Join(result, benign(<-results))
	}
	if err := result; err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	return errBrokerEndpointStopped
}
