package main

import (
	"context"
	"fmt"
	"io"
	"os"

	"example.com/opennav-update-verifier/launcher"
)

func startup(ctx context.Context, args []string, run func(context.Context, []string) error, stderr io.Writer, reportFailure func()) int {
	if err := run(ctx, args); err != nil {
		fmt.Fprintln(stderr, "SKAGER startup did not complete:", err)
		// The installer's exact unattended command must return its failure to
		// the owning process. Ordinary and malformed startup retain guidance.
		if len(args) != 1 || args[0] != "--initialize-trust" {
			reportFailure()
		}
		return 1
	}
	return 0
}

func main() {
	os.Exit(startup(context.Background(), os.Args[1:], launcher.Run, os.Stderr, launcher.ReportFailure))
}
