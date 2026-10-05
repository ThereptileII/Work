package main

import (
	"context"
	"fmt"
	"os"

	"example.com/opennav-update-verifier/launcher"
)

func main() {
	if err := launcher.Run(context.Background(), os.Args[1:]); err != nil {
		fmt.Fprintln(os.Stderr, "SKAGER startup did not complete:", err)
		launcher.ReportFailure()
		os.Exit(1)
	}
}
