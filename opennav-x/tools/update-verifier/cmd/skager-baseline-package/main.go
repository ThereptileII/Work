// skager-baseline-package prepares an operator-reviewable, exact initial
// baseline handoff. Its output SHA does not approve its own client request.
package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"strings"

	"example.com/opennav-update-verifier/baselinepackage"
)

func main() {
	result, err := run(context.Background(), os.Args[1:])
	if err != nil {
		fmt.Fprintln(os.Stderr, "SKAGER baseline handoff refused:", err)
		os.Exit(1)
	}
	fmt.Println("baseline:", result.ArchivePath)
	fmt.Println("client request:", result.RequestPath)
	fmt.Println("request SHA-256 for separate operator approval:", result.RequestSHA256)
}

func run(ctx context.Context, args []string) (baselinepackage.Result, error) {
	flags := flag.NewFlagSet("skager-baseline-package", flag.ContinueOnError)
	flags.SetOutput(io.Discard)
	in := baselinepackage.Input{}
	flags.StringVar(&in.SourceArchive, "source-archive", "", "exact original portable recovery ZIP")
	flags.StringVar(&in.BaselineTree, "baseline-tree", "", "extracted original portable recovery tree")
	flags.StringVar(&in.TrustConfig, "trust-config", "", "independently approved public trust JSON")
	flags.StringVar(&in.StockPath, "stock-path", "", "actual original OpenCPN executable")
	flags.StringVar(&in.OutputDirectory, "output-directory", "", "existing protected acquisition directory")
	flags.StringVar(&in.ClientSID, "client-sid", "", "actual intended Windows account SID")
	flags.StringVar(&in.Generation, "generation", "", "approved 32-character baseline generation")
	flags.StringVar(&in.Version, "version", "", "approved application version")
	flags.StringVar(&in.Commit, "commit", "", "approved 40-character application commit")
	flags.StringVar(&in.ApprovedSourceSHA256, "approved-source-sha256", "", "independent original recovery ZIP SHA-256")
	flags.StringVar(&in.ApprovedPackageSHA256, "approved-package-sha256", "", "independent installer/package.json SHA-256")
	flags.StringVar(&in.ApprovedBuildSHA256, "approved-build-sha256", "", "independent docs/PRODUCT_BUILD.json SHA-256")
	flags.StringVar(&in.ApprovedExecutableSHA256, "approved-executable-sha256", "", "independent app/opencpn.exe SHA-256")
	flags.StringVar(&in.ApprovedInventorySHA256, "approved-inventory-sha256", "", "independent FILE_SHA256.json SHA-256")
	flags.StringVar(&in.ApprovedTrustSHA256, "approved-trust-sha256", "", "independent public trust SHA-256")
	flags.StringVar(&in.ApprovedStockSHA256, "approved-stock-sha256", "", "independent supported stock OpenCPN SHA-256")
	seen := map[string]bool{}
	for i := 0; i < len(args); i++ {
		if !strings.HasPrefix(args[i], "--") {
			return baselinepackage.Result{}, errors.New("named operator inputs required")
		}
		name, _, inline := strings.Cut(strings.TrimPrefix(args[i], "--"), "=")
		if flags.Lookup(name) == nil || seen[name] {
			return baselinepackage.Result{}, errors.New("unknown or duplicate operator input")
		}
		seen[name] = true
		if !inline {
			i++
			if i >= len(args) || strings.HasPrefix(args[i], "--") {
				return baselinepackage.Result{}, errors.New("operator input value required")
			}
		}
	}
	if len(seen) != 16 || flags.Parse(args) != nil || flags.NArg() != 0 {
		return baselinepackage.Result{}, errors.New("complete exact operator input set required")
	}
	return baselinepackage.Produce(ctx, in)
}
