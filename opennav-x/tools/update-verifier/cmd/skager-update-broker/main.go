package main

import (
	"context"
	"fmt"
	"os"

	"example.com/opennav-update-verifier/brokerhost"
)

func main() {
	if len(os.Args) != 1 || brokerhost.Run(context.Background()) != nil {
		fmt.Fprintln(os.Stderr, "SKAGER update broker unavailable.")
		os.Exit(1)
	}
}
