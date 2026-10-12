package brokerhost

import (
	"context"
	"errors"
	"time"

	"example.com/opennav-update-verifier/brokerregistration"
)

// registerClientServiceStart handles only the explicit stopped SCM operation.
// Its callbacks are the fixed native protected request reader and registration
// coordinator, never pipe/client supplied authority. Normal and initialization
// starts continue to the existing dispatcher. No path is accepted as an argument.
func registerClientServiceStart(ctx context.Context, args []string,
	read func(context.Context, string, string) ([]byte, error),
	register func(context.Context, []byte, string) error) (bool, error) {
	if len(args) < 2 || args[1] != "register-client" {
		return false, nil
	}
	if ctx == nil || ctx.Err() != nil || len(args) != 4 || args[0] != ServiceName ||
		!brokerregistration.IsCanonicalUserSID(args[2]) || !hashPattern.MatchString(args[3]) || read == nil || register == nil {
		return true, errors.New("exact approved stopped client registration arguments required")
	}
	raw, err := read(ctx, args[2], args[3])
	if err != nil {
		return true, err
	}
	if err = ctx.Err(); err != nil {
		return true, err
	}
	request, err := brokerregistration.ParseApprovedRequest(raw, args[3], time.Now().UTC())
	if err != nil || request.ClientSID != args[2] {
		return true, errors.New("protected client request differs from SCM approval")
	}
	if err = ctx.Err(); err != nil {
		return true, err
	}
	// The coordinator revalidates these owned original bytes before mutation. Once
	// it reports successful durable completion, a late cancellation is not rollback.
	return true, register(ctx, request.OriginalBytes(), args[3])
}
