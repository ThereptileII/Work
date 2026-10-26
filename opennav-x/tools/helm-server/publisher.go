package helm

import (
	"errors"
	"sync"

	"github.com/pion/rtp"
	"github.com/pion/rtp/codecs"
)

var errNoNALUnits = errors.New("H264 frame has no NAL units")

// AnnexBNALUnits splits a bounded Annex-B H264 access unit. It deliberately
// does not attempt to decode or reinterpret video; the platform capture/codec
// boundary remains responsible for producing a valid access unit.
func AnnexBNALUnits(data []byte) [][]byte {
	var out [][]byte
	start := -1
	for i := 0; i+3 < len(data); {
		prefix := 0
		if i+4 <= len(data) && data[i] == 0 && data[i+1] == 0 && data[i+2] == 0 && data[i+3] == 1 {
			prefix = 4
		} else if data[i] == 0 && data[i+1] == 0 && data[i+2] == 1 {
			prefix = 3
		}
		if prefix == 0 {
			i++
			continue
		}
		if start >= 0 && start < i {
			out = append(out, append([]byte(nil), data[start:i]...))
		}
		start = i + prefix
		i += prefix
	}
	if start >= 0 && start < len(data) {
		out = append(out, append([]byte(nil), data[start:]...))
	}
	return out
}

type H264Publisher struct {
	server *Server
	mu     sync.Mutex
	seq    uint16
	ssrc   uint32
}

func NewH264Publisher(server *Server) *H264Publisher {
	return &H264Publisher{server: server, ssrc: 0x534b4752}
}

// PublishAccessUnit packetizes one H264 access unit for the server's WebRTC
// track. It is intended for platform capture adapters and test publishers.
func (p *H264Publisher) PublishAccessUnit(nals [][]byte, timestamp uint32) error {
	if p == nil || p.server == nil || len(nals) == 0 {
		return errNoNALUnits
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	payloader := codecs.H264Payloader{}
	for _, nal := range nals {
		if len(nal) == 0 {
			continue
		}
		for _, payload := range payloader.Payload(1200, nal) {
			p.seq++
			packet := &rtp.Packet{Header: rtp.Header{Version: 2, PayloadType: 102, SequenceNumber: p.seq, Timestamp: timestamp, SSRC: p.ssrc}, Payload: payload}
			if err := p.server.PublishRTP(packet); err != nil {
				return err
			}
		}
	}
	return nil
}
