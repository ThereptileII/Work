package brokerstore

import (
	"bytes"
	"encoding/json"
	"errors"
)

const acquisitionDirectory = ".acquisition"
const acquisitionJournal = "lease.jsonl"
const maxAcquisitionJournal = 64 << 10
const maxAcquisitionEvents = 24

type diskIdentity struct {
	Volume  uint64
	FileID  string
	Created uint64
}

func (i diskIdentity) valid() bool {
	return validHex(i.FileID, 32) && i.FileID != "00000000000000000000000000000000"
}

type acquisitionEvent struct {
	Schema    int
	Sequence  int
	State     string
	StageID   string
	ClientSID string
	Directory diskIdentity
	Parent    diskIdentity
	Plan      *AcquisitionPlan `json:",omitempty"`
	Slot      ArtifactSlot     `json:",omitempty"`
	Identity  *diskIdentity    `json:",omitempty"`
}
type acquisitionIndex struct {
	events     []acquisitionEvent
	plan       *AcquisitionPlan
	states     map[ArtifactSlot]string
	identities map[ArtifactSlot]diskIdentity
	complete   bool
}

func newAcquisitionIndex() *acquisitionIndex {
	return &acquisitionIndex{states: map[ArtifactSlot]string{}, identities: map[ArtifactSlot]diskIdentity{}}
}
func (j *acquisitionIndex) apply(e acquisitionEvent) error {
	if e.Schema != 1 || e.Sequence != len(j.events)+1 || len(j.events) >= maxAcquisitionEvents || !validHex(e.StageID, 32) || !e.Directory.valid() || !e.Parent.valid() || e.ClientSID == "" || len(e.ClientSID) > 184 || j.complete {
		return errors.New("invalid acquisition journal identity/order/bounds")
	}
	if len(j.events) == 0 && e.State != "lease" {
		return errors.New("acquisition journal must originate with lease")
	}
	if len(j.events) > 0 {
		first := j.events[0]
		if first.StageID != e.StageID || first.ClientSID != e.ClientSID || first.Directory != e.Directory || first.Parent != e.Parent {
			return errors.New("acquisition journal binding changed")
		}
	}
	noSlot := e.Slot == "" && e.Identity == nil
	switch e.State {
	case "lease":
		if len(j.events) != 0 || !noSlot || e.Plan != nil {
			return errors.New("invalid lease origin")
		}
	case "bound":
		if len(j.events) != 1 || !noSlot || e.Plan == nil || !e.Plan.valid() {
			return errors.New("invalid acquisition plan binding")
		}
	case "creating":
		if j.plan == nil || e.Plan != nil || e.Identity != nil || (e.Slot != RecoverySlot && e.Slot != InstallerSlot) || j.states[e.Slot] != "" {
			return errors.New("invalid artifact creation intent")
		}
	case "owned":
		if e.Plan != nil || e.Identity == nil || !e.Identity.valid() || j.states[e.Slot] != "creating" {
			return errors.New("artifact lacks exact creation identity")
		}
	case "deleting":
		if e.Plan != nil || e.Identity == nil || j.states[e.Slot] != "owned" || j.identities[e.Slot] != *e.Identity {
			return errors.New("invalid artifact delete intent")
		}
	case "deleted":
		if e.Plan != nil || e.Identity == nil || j.states[e.Slot] != "deleting" || j.identities[e.Slot] != *e.Identity {
			return errors.New("invalid artifact deletion completion")
		}
	case "complete":
		if !noSlot || e.Plan != nil {
			return errors.New("invalid lease completion")
		}
		for _, state := range j.states {
			if state != "deleted" {
				return errors.New("incomplete artifact cannot complete lease")
			}
		}
	default:
		return errors.New("unknown acquisition journal transition")
	}
	j.events = append(j.events, e)
	if e.State == "bound" {
		p := *e.Plan
		j.plan = &p
	}
	if e.Slot != "" {
		j.states[e.Slot] = e.State
		if e.Identity != nil {
			j.identities[e.Slot] = *e.Identity
		}
	}
	j.complete = e.State == "complete"
	return nil
}
func parseAcquisitionIndex(data []byte) (*acquisitionIndex, error) {
	if len(data) == 0 || len(data) > maxAcquisitionJournal || data[len(data)-1] != '\n' {
		return nil, errors.New("uncertain or oversized acquisition journal")
	}
	j := newAcquisitionIndex()
	for _, line := range bytes.Split(data[:len(data)-1], []byte{'\n'}) {
		var e acquisitionEvent
		if err := json.Unmarshal(line, &e); err != nil {
			return nil, err
		}
		canonical, err := json.Marshal(e)
		if err != nil || !bytes.Equal(line, canonical) {
			return nil, errors.New("noncanonical, duplicate or unknown acquisition journal fields")
		}
		if err = j.apply(e); err != nil {
			return nil, err
		}
	}
	return j, nil
}
func slotFilename(slot ArtifactSlot) string {
	switch slot {
	case RecoverySlot:
		return "recovery.zip"
	case InstallerSlot:
		return "setup.exe"
	}
	return ""
}
