package helm

import (
	"context"
	"encoding/json"
	"net/http/httptest"
	"net/url"
	"strings"
	"testing"
	"time"

	"github.com/gorilla/websocket"
	"github.com/pion/webrtc/v4"
)

func TestInfoPairHTTPContract(t *testing.T) {
	s, err := NewServer(ServerConfig{ServerID: "pc-1", DisplayName: "SKAGER", PairingCode: "123456", MaxClients: 1, Media: DefaultMediaPolicy()})
	if err != nil {
		t.Fatal(err)
	}
	h := httptest.NewServer(s.Handler())
	defer h.Close()
	resp, err := h.Client().Get(h.URL + "/helm/v1/info")
	if err != nil {
		t.Fatal(err)
	}
	defer resp.Body.Close()
	var info PairingInfo
	if err := json.NewDecoder(resp.Body).Decode(&info); err != nil {
		t.Fatal(err)
	}
	if info.ServerID != "pc-1" || len(info.Capabilities) == 0 {
		t.Fatalf("bad info response: %+v", info)
	}
	form := `{"pairing_code":"123456","client_name":"test-browser"}`
	resp, err = h.Client().Post(h.URL+"/helm/v1/pair", "application/json", strings.NewReader(form))
	if err != nil {
		t.Fatal(err)
	}
	defer resp.Body.Close()
	var cred SessionCredentials
	if err := json.NewDecoder(resp.Body).Decode(&cred); err != nil {
		t.Fatal(err)
	}
	if cred.SessionID == "" || cred.Token == "" {
		t.Fatalf("empty credentials: %+v", cred)
	}
}

func TestNonLoopbackServerRequiresTLS(t *testing.T) {
	s, err := NewServer(ServerConfig{ListenAddr: "0.0.0.0:43872", ServerID: "pc-1", DisplayName: "SKAGER", PairingCode: "123456", MaxClients: 1, Media: DefaultMediaPolicy()})
	if err != nil {
		t.Fatal(err)
	}
	if err := s.Serve(context.Background()); err == nil {
		t.Fatal("non-loopback server started without TLS")
	}
}

func TestWebRTCOfferAnswerAndInputAck(t *testing.T) {
	s, err := NewServer(ServerConfig{ServerID: "pc-1", DisplayName: "SKAGER", PairingCode: "123456", MaxClients: 1, Media: DefaultMediaPolicy()})
	if err != nil {
		t.Fatal(err)
	}
	h := httptest.NewServer(s.Handler())
	defer h.Close()
	cred, err := s.Registry().Pair("123456", "test-client", time.Now())
	if err != nil {
		t.Fatal(err)
	}
	peer, err := webrtc.NewPeerConnection(webrtc.Configuration{})
	if err != nil {
		t.Fatal(err)
	}
	defer peer.Close()
	input, err := peer.CreateDataChannel("input", nil)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := peer.AddTransceiverFromKind(webrtc.RTPCodecTypeVideo); err != nil {
		t.Fatal(err)
	}
	opened := make(chan struct{})
	input.OnOpen(func() { close(opened) })
	trackReceived := make(chan *webrtc.TrackRemote, 1)
	peer.OnTrack(func(track *webrtc.TrackRemote, _ *webrtc.RTPReceiver) { trackReceived <- track })
	answerReady := make(chan webrtc.SessionDescription, 1)
	wsURL, _ := url.Parse(h.URL)
	wsURL.Scheme = "ws"
	wsURL.Path = "/helm/v1/signal"
	query := wsURL.Query()
	query.Set("token", cred.Token)
	wsURL.RawQuery = query.Encode()
	ws, _, err := websocket.DefaultDialer.Dial(wsURL.String(), nil)
	if err != nil {
		t.Fatal(err)
	}
	defer ws.Close()
	go func() {
		var env signalEnvelope
		if err := ws.ReadJSON(&env); err == nil && env.Type == "answer" {
			answerReady <- webrtc.SessionDescription{Type: webrtc.SDPTypeAnswer, SDP: env.SDP}
		}
	}()
	offer, err := peer.CreateOffer(nil)
	if err != nil {
		t.Fatal(err)
	}
	gather := webrtc.GatheringCompletePromise(peer)
	if err := peer.SetLocalDescription(offer); err != nil {
		t.Fatal(err)
	}
	<-gather
	if err := ws.WriteJSON(signalEnvelope{Type: "offer", SDP: peer.LocalDescription().SDP}); err != nil {
		t.Fatal(err)
	}
	select {
	case answer := <-answerReady:
		if err := peer.SetRemoteDescription(answer); err != nil {
			t.Fatal(err)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("timed out waiting for WebRTC answer")
	}
	select {
	case <-opened:
	case <-time.After(5 * time.Second):
		t.Fatal("data channel did not open")
	}
	publisher := NewH264Publisher(s)
	if err := publisher.PublishAccessUnit([][]byte{{0x65, 0, 0, 1, 2, 3}}, 90000); err != nil {
		t.Fatal(err)
	}
	select {
	case track := <-trackReceived:
		if _, _, err := track.ReadRTP(); err != nil {
			t.Fatal(err)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("timed out waiting for WebRTC video track")
	}
	if err := input.SendText(`{"type":"input","event":{"type":"pointer_down","x":0.5,"y":0.5,"sequence":1}}`); err != nil {
		t.Fatal(err)
	}
}
