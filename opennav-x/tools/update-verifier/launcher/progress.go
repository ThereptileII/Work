package launcher

import (
	"context"
	"errors"
	"time"
)

// downloadProgress owns only the inert progress prompt. Closing it must never
// terminate an installer or supervisor. done closes after the prompt exits.
type downloadProgress interface {
	done() <-chan struct{}
	finish(context.Context) error
	close()
}

// Preparation remains cancellable until the progress prompt has acknowledged
// our EOF with exit zero. Only an owned, uncancelled result crosses this gate;
// NSIS starts later and is deliberately outside this cancellation scope.
func prepareWithProgress(ctx context.Context, p platform, app string, prepare func(context.Context) (preparation, error)) (preparation, error) {
	downloadCtx, cancel := context.WithTimeout(ctx, 30*time.Minute)
	defer cancel()
	if err := downloadCtx.Err(); err != nil {
		return nil, err
	}
	progress, err := p.progress(app+"/skager-update-prompt.exe", app)
	if err != nil {
		return nil, errors.New("preparation progress unavailable")
	}
	defer progress.close()
	select {
	case <-progress.done():
		return nil, errors.New("preparation progress exited before preparation")
	default:
	}
	stop := make(chan struct{})
	stopped := make(chan struct{})
	go func() {
		defer close(stopped)
		select {
		case <-progress.done():
			cancel()
		case <-stop:
		}
	}()
	prepared, err := prepare(downloadCtx)
	close(stop)
	<-stopped
	// Stop and join the watcher before intentionally closing stdin. An exit
	// concurrent with preparation completion is also rejected by finish.
	if err == nil {
		err = downloadCtx.Err()
	}
	if err == nil && prepared == nil {
		err = errors.New("preparation returned no artifact")
	}
	if err == nil {
		finishCtx, finishCancel := context.WithTimeout(downloadCtx, 5*time.Second)
		err = progress.finish(finishCtx)
		finishCancel()
	}
	if err == nil {
		err = downloadCtx.Err()
	}
	if err != nil {
		if prepared != nil {
			_ = prepared.close()
		}
		return nil, errors.New("update preparation cancelled or failed")
	}
	return prepared, nil
}
