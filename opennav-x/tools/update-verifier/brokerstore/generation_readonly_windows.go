package brokerstore

import (
	"context"
	"errors"
	"sort"

	"golang.org/x/sys/windows"
)

// reopenGenerationReadOnly never duplicates a writer FILE_OBJECT. Its metadata
// anchor has no data-read/write/delete access; sharing exclusions do not turn it
// into a write fence. Historical at-creation security and the retained physical
// parent namespace are the write fence across writer Close -> readonly reopen.
// Only broker/SYSTEM had mutation rights from birth; the Store serializes its
// own writers. The new independent file object has READ, no WRITE or DELETE,
// permits loader read/execute opens and denies subsequent writers/deleters.
func reopenGenerationReadOnly(ctx context.Context, old *nativeBaselineFile, length int64) (reader *nativeBaselineFile, consumed bool, resultErr error) {
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	if old == nil || !old.identity.valid() {
		return nil, false, errors.New("original retained generation file required")
	}
	if err := old.check(length); err != nil {
		return nil, false, err
	}
	name := old.pin.name[len(old.parent.name)+1:]
	anchor, err := openAcquisitionFile(old.parent, name, old.pin.security, windows.FILE_READ_ATTRIBUTES|windows.READ_CONTROL, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, windows.OPEN_EXISTING)
	if err != nil {
		return nil, false, err
	}
	defer func() { resultErr = errors.Join(resultErr, anchor.Close()) }()
	anchorID, err := fileIdentity(anchor.pin.handle)
	if err != nil || anchorID != old.identity {
		return nil, false, errors.Join(errors.New("metadata transition anchor identity differs"), err)
	}
	if err = anchor.check(length); err != nil {
		return nil, false, err
	}
	if err = ctx.Err(); err != nil {
		return nil, false, err
	}
	consumed = true
	if err = old.Close(); err != nil {
		return nil, true, err
	}
	// Do not restart or adopt after any refusal. Keeping the metadata anchor and
	// protected parent pin prevents silently replacing a member during this gap.
	if err = ctx.Err(); err != nil {
		return nil, true, err
	}
	next, err := baselineFile(old.parent, name, old.pin.security, false)
	if err != nil {
		return nil, true, err
	}
	reader = next
	if next.identity != anchorID {
		return reader, true, errors.New("readonly generation file identity differs")
	}
	if err = next.check(length); err != nil {
		return reader, true, err
	}
	if err = anchor.check(length); err != nil {
		return reader, true, err
	}
	return reader, true, nil
}

func (g *nativeInstalledGeneration) transitionReadOnly(ctx context.Context, files map[string]candidateFile, members map[string]baselineMember) error {
	if !g.completed || g.closed || g.readOnlyAttempted {
		return errors.New("one live completed native generation transition required")
	}
	g.readOnlyAttempted = true
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := g.check(); err != nil {
		return err
	}
	if err := g.checkHeldDigests(); err != nil {
		return err
	}
	names := make([]string, 0, len(members))
	for name := range members {
		names = append(names, name)
	}
	sort.Strings(names)
	if len(files) != len(members) || len(g.members) != len(members) {
		return errors.New("exact retained generation writer set required")
	}
	for _, name := range names {
		old, ok := files[name].(*nativeBaselineFile)
		m := members[name]
		if !ok || old == nil || old != g.members[name] || old.identity != m.Identity {
			return errors.New("actual native original generation writer required")
		}
		reader, consumed, err := reopenGenerationReadOnly(ctx, old, m.Length)
		if consumed {
			delete(files, name)
			delete(g.members, name)
			if reader != nil {
				files[name] = &generationReadOnlyFile{candidateFile: reader, reader: reader.File}
				g.members[name] = reader
			}
		}
		if err != nil {
			return err
		}
		if err = g.check(); err != nil {
			return err
		}
	}
	// Private parent/reservation/completion writers also become readonly. This
	// preserves original byte/physical custody and permits authenticated readers
	// to inspect completed creation history while this owner remains live.
	private := map[*nativeBaselineFile]int64{}
	for _, held := range g.held {
		if held.file == nil {
			return errors.New("original private generation file missing")
		}
		private[held.file] = held.length
	}
	if g.marker != nil {
		private[g.marker] = g.markerLength
	}
	if g.record == nil {
		return errors.New("completed private generation record missing")
	}
	ordered := make([]*nativeBaselineFile, 0, len(private))
	for file := range private {
		ordered = append(ordered, file)
	}
	sort.Slice(ordered, func(i, j int) bool { return ordered[i].pin.name < ordered[j].pin.name })
	for _, old := range ordered {
		reader, consumed, err := reopenGenerationReadOnly(ctx, old, private[old])
		if consumed {
			for i := range g.held {
				if g.held[i].file == old {
					g.held[i].file = reader
				}
			}
			if g.marker == old {
				g.marker = reader
			}
			if g.record == old {
				g.record = reader
			}
		}
		if err != nil {
			return err
		}
		if err = g.check(); err != nil {
			return err
		}
	}
	return g.checkHeldDigests()
}
