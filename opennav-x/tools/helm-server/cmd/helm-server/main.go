// skager-helm: the SKAGER Helm Server. SKAGER starts it (Settings → SKAGER Helm)
// with -control stdio; it can also be run by hand for development.
package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"path/filepath"
	"strconv"
	"strings"

	helm "skager.app/helm"
)

func main() {
	addr := flag.String("addr", ":"+strconv.Itoa(helm.DefaultPort), "listen address (all interfaces by default, so phones on the boat Wi-Fi can connect)")
	name := flag.String("name", "SKAGER", "boat name shown on the phone")
	stateDir := flag.String("state-dir", "", "directory for paired devices (required; SKAGER passes its profile)")
	window := flag.String("window", "", "window to show (Windows HWND or X11 window id); empty = primary screen")
	source := flag.String("source", "screen", "screen, or test for the labelled test pattern (development only)")
	testSize := flag.String("test-size", "1280x800", "test pattern size")
	control := flag.String("control", "none", "stdio when started by SKAGER")
	pair := flag.Bool("pair", false, "open a pairing window at start and print it (development)")
	inputLog := flag.String("input-log", "", "test pattern only: append received input to this file")
	version := flag.Bool("version", false, "print the version")
	flag.Parse()
	if *version {
		fmt.Println("skager-helm", helm.Version)
		return
	}
	logger := log.New(os.Stderr, "skager-helm: ", log.LstdFlags)
	if *stateDir == "" {
		logger.Fatal("-state-dir is required")
	}
	store, err := helm.OpenStore(filepath.Join(*stateDir, "helm-devices.json"))
	if err != nil {
		logger.Fatalf("device store: %v", err)
	}
	var display helm.Display
	switch *source {
	case "screen":
		display, err = helm.OpenDisplay(*window)
		if err != nil {
			logger.Fatalf("display: %v", err)
		}
	case "test":
		var w, h int
		if _, err := fmt.Sscanf(*testSize, "%dx%d", &w, &h); err != nil || w < 320 || h < 240 || w > 4096 || h > 4096 {
			logger.Fatal("invalid -test-size")
		}
		tp := helm.NewTestPattern(w, h)
		if *inputLog != "" {
			f, err := os.OpenFile(*inputLog, os.O_CREATE|os.O_APPEND|os.O_WRONLY, 0o600)
			if err != nil {
				logger.Fatal(err)
			}
			defer f.Close()
			tp.OnInput = func(line string) { fmt.Fprintln(f, line) }
		}
		display = tp
		logger.Print("TEST PATTERN source: this is not SKAGER's screen")
	default:
		logger.Fatal("-source must be screen or test")
	}
	defer display.Close()

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt)
	go func() { <-sig; cancel() }()

	var emit func(helm.Event)
	stdio := *control == "stdio"
	if stdio {
		w := helm.NewEventWriter(os.Stdout)
		emit = w.Write
	} else {
		emit = func(ev helm.Event) {
			switch ev.Event {
			case "pairing":
				printPairing(ev)
			case "session", "paired", "pairing_closed":
				device := ""
				if ev.Device != nil {
					device = ev.Device.Name
				}
				logger.Printf("%s %s %s %s", ev.Event, ev.State, device, ev.Reason)
			}
		}
	}
	server, err := helm.NewServer(helm.Config{ListenAddr: *addr, Name: *name, Store: store, Display: display,
		Events: emit, Logf: logger.Printf})
	if err != nil {
		logger.Fatal(err)
	}
	listener, err := server.Listen()
	if err != nil {
		emit(helm.Event{Event: "error", Message: "Helm could not open port: " + err.Error()})
		logger.Fatal(err)
	}
	emit(helm.Event{Event: "ready", ServerID: store.ServerID(), Name: server.Name(), Port: server.Port(),
		Addresses: helm.LANAddresses(server.Port())})
	server.EmitDevices()
	logger.Printf("listening on port %d (version %s)", server.Port(), helm.Version)
	if stdio {
		go helm.RunControl(ctx, cancel, server, os.Stdin, emit)
	}
	if *pair {
		ev, err := server.OpenPairing()
		if err != nil {
			logger.Fatal(err)
		}
		emit(ev)
	}
	if err := server.Serve(ctx, listener); err != nil {
		logger.Fatal(err)
	}
}

// printPairing shows the pairing QR code in a terminal (development).
func printPairing(ev helm.Event) {
	var b strings.Builder
	b.WriteString("\nScan with the SKAGER Helm app, or enter the key " + ev.Key + "\n")
	b.WriteString("Addresses: " + strings.Join(ev.Addresses, ", ") + "\n\n")
	if ev.QR != nil {
		rows := append([]string{strings.Repeat("0", ev.QR.Size)}, ev.QR.Rows...)
		rows = append(rows, strings.Repeat("0", ev.QR.Size))
		for y := 0; y < len(rows); y += 2 {
			b.WriteString("  ")
			for x := -1; x <= ev.QR.Size; x++ {
				top, bottom := module(rows, x, y), module(rows, x, y+1)
				switch {
				case top && bottom:
					b.WriteString(" ")
				case top:
					b.WriteString("▄")
				case bottom:
					b.WriteString("▀")
				default:
					b.WriteString("█")
				}
			}
			b.WriteString("\n")
		}
	}
	fmt.Fprintln(os.Stderr, b.String())
}

func module(rows []string, x, y int) bool {
	if y >= len(rows) || x < 0 || x >= len(rows[y]) {
		return false
	}
	return rows[y][x] == '1'
}
