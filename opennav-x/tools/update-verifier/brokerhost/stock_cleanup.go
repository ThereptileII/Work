package brokerhost

import "errors"

// ErrStockCleanup reports uncertain release of retained stock image/namespace
// custody. It is not ordinary observation failure or permission to retry close.
var ErrStockCleanup = errors.New("stock observation owner cleanup uncertain")

func stockCleanup(err error) error {
	if err == nil {
		return nil
	}
	return errors.Join(ErrStockCleanup, err)
}

// Real native callers supply their actual probe check and CloseHandle. Keep the
// close outcome even when observation failed, so a temporary pin cannot silently
// survive an otherwise successful validation. Private callbacks allow inert tests.
func checkAndCloseStockProbe(check, close func() error) (resultErr error) {
	defer func() { resultErr = errors.Join(resultErr, stockCleanup(close())) }()
	return check()
}
