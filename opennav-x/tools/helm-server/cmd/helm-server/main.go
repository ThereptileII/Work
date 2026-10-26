package main

import (
	"context"
	"flag"
	"log"
	"os"
	"time"

	helm "example.com/skager-helm"
)

func main() {
	addr := flag.String("addr", "127.0.0.1:43872", "LAN listen address")
	serverID := flag.String("server-id", "skager-pc", "stable SKAGER PC identity")
	displayName := flag.String("display-name", "SKAGER", "name shown during pairing")
	pairingCode := flag.String("pairing-code", "", "short-lived code provisioned by the SKAGER UI")
	certFile := flag.String("tls-cert", "", "TLS certificate for non-loopback LAN signalling")
	keyFile := flag.String("tls-key", "", "TLS private key for non-loopback LAN signalling")
	h264File := flag.String("h264-file", "", "developer-only Annex-B H264 access unit to publish repeatedly")
	flag.Parse()
	if *pairingCode == "" {
		log.Fatal("-pairing-code is required; do not put it in source or a release manifest")
	}
	s, err := helm.NewServer(helm.ServerConfig{ListenAddr: *addr, ServerID: *serverID, DisplayName: *displayName, PairingCode: *pairingCode, MaxClients: 1, Media: helm.DefaultMediaPolicy(), TLSCertFile: *certFile, TLSKeyFile: *keyFile})
	if err != nil {
		log.Fatal(err)
	}
	if *h264File != "" {
		data, readErr := os.ReadFile(*h264File)
		if readErr != nil {
			log.Fatal(readErr)
		}
		publisher := helm.NewH264Publisher(s)
		go func() {
			ticker := time.NewTicker(time.Second / 30)
			defer ticker.Stop()
			for range ticker.C {
				if err := publisher.PublishAccessUnit(helm.AnnexBNALUnits(data), uint32(time.Now().UnixNano()/1e6*90)); err != nil {
					log.Printf("H264 test publisher: %v", err)
				}
			}
		}()
		log.Printf("developer H264 publisher enabled from %s", *h264File)
	}
	log.Printf("SKAGER Helm listening on %s; pairing code is intentionally not logged", *addr)
	if err := s.Serve(context.Background()); err != nil && err.Error() != "http: Server closed" {
		log.Print(err)
		os.Exit(1)
	}
}
