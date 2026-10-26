package helm

import (
	"errors"
	"net"
	"net/url"
	"sort"
	"strconv"
	"strings"

	"rsc.io/qr"
)

// AppScheme is the SKAGER Helm app's link scheme. Scanning the pairing QR code
// with the phone's camera opens the app directly with the link.
const AppScheme = "app.skager.helm"

// PairingLink is the content of the pairing QR code:
//
//	app.skager.helm://pair?v=1&id=<server id>&n=<boat>&a=<ip:port,...>&k=<pairing key>
func PairingLink(serverID, name string, addresses []string, key string) string {
	q := url.Values{}
	q.Set("v", "1")
	q.Set("id", serverID)
	q.Set("n", name)
	q.Set("a", strings.Join(addresses, ","))
	q.Set("k", key)
	return AppScheme + "://pair?" + q.Encode()
}

// QRMatrix is a QR code as rows of '1' (dark) and '0' (light) modules, without
// the quiet zone. SKAGER draws it natively, so no image crosses the pipe.
type QRMatrix struct {
	Size int      `json:"size"`
	Rows []string `json:"rows"`
}

func EncodeQR(text string) (QRMatrix, error) {
	code, err := qr.Encode(text, qr.M)
	if err != nil {
		return QRMatrix{}, err
	}
	m := QRMatrix{Size: code.Size, Rows: make([]string, code.Size)}
	row := make([]byte, code.Size)
	for y := 0; y < code.Size; y++ {
		for x := 0; x < code.Size; x++ {
			row[x] = '0'
			if code.Black(x, y) {
				row[x] = '1'
			}
		}
		m.Rows[y] = string(row)
	}
	return m, nil
}

// LANAddresses lists the private IPv4 addresses the phone may reach this PC on,
// most likely first (192.168/16 and 10/8 before 172.16/12 and CGNAT/Tailscale).
func LANAddresses(port int) []string {
	ifaces, err := net.Interfaces()
	if err != nil {
		return nil
	}
	type candidate struct {
		addr string
		rank int
	}
	var found []candidate
	for _, iface := range ifaces {
		if iface.Flags&net.FlagUp == 0 || iface.Flags&net.FlagLoopback != 0 {
			continue
		}
		addrs, _ := iface.Addrs()
		for _, a := range addrs {
			ipnet, ok := a.(*net.IPNet)
			if !ok {
				continue
			}
			ip := ipnet.IP.To4()
			if ip == nil {
				continue
			}
			rank := addressRank(ip)
			if rank < 0 {
				continue
			}
			found = append(found, candidate{net.JoinHostPort(ip.String(), strconv.Itoa(port)), rank})
		}
	}
	sort.SliceStable(found, func(i, j int) bool { return found[i].rank < found[j].rank })
	out := make([]string, 0, len(found))
	for _, c := range found {
		if len(out) == 4 {
			break
		}
		out = append(out, c.addr)
	}
	return out
}

func addressRank(ip net.IP) int {
	switch {
	case ip[0] == 192 && ip[1] == 168:
		return 0
	case ip[0] == 10:
		return 1
	case ip[0] == 172 && ip[1]&0xf0 == 16:
		return 2
	case ip[0] == 100 && ip[1]&0xc0 == 64: // CGNAT, e.g. Tailscale
		return 3
	case ip.IsLinkLocalUnicast():
		return -1
	case ip.IsPrivate():
		return 2
	}
	return -1
}

// ParsedLink is a pairing link as read by a client.
type ParsedLink struct {
	ServerID  string
	Name      string
	Addresses []string
	Secret    []byte
}

func ParsePairingLink(link string) (ParsedLink, error) {
	u, err := url.Parse(link)
	if err != nil || u.Scheme != AppScheme || u.Host != "pair" {
		return ParsedLink{}, errors.New("not a SKAGER Helm pairing link")
	}
	q := u.Query()
	if q.Get("v") != "1" {
		return ParsedLink{}, errors.New("unsupported pairing link version")
	}
	secret, err := ParsePairingKey(q.Get("k"))
	if err != nil {
		return ParsedLink{}, err
	}
	var addrs []string
	for _, a := range strings.Split(q.Get("a"), ",") {
		if _, _, err := net.SplitHostPort(a); err == nil {
			addrs = append(addrs, a)
		}
	}
	return ParsedLink{ServerID: q.Get("id"), Name: q.Get("n"), Addresses: addrs, Secret: secret}, nil
}
