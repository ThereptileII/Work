package brokerstore

import (
	"errors"
	"sort"
	"strings"
)

type decommissionRootInventory struct {
	names                        []string
	deployed                     bool
	executions, profiles, stages []string
}

// Classify the entire bounded root, never arbitrary SID-prefix tolerance. Exact
// pair completeness is only syntax; native original readers establish custody.
func classifyDecommissionInventory(names []string, sid, baseline string) (decommissionRootInventory, error) {
	result := decommissionRootInventory{}
	if len(names) < 2 || len(names) > maxExecutionNamespaceFiles {
		return result, errors.New("bounded singleton registration inventory required")
	}
	seen := map[string]bool{}
	groups := map[string]map[string]map[string]bool{"execution": {}, "profile": {}, "stage": {}}
	for _, name := range names {
		if seen[name] {
			return result, errors.New("duplicate protected root entry")
		}
		seen[name] = true
		switch name {
		case sid + ".once", sid + ".json":
			continue
		case sid + ".launch-parents.once", sid + ".launch-parents.json", sid + ".generations":
			result.deployed = true
			continue
		case sid + "." + baseline + ".baseline.once":
			if !validHex(baseline, 32) {
				return result, errors.New("original registered baseline identity required")
			}
			result.deployed = true
			continue
		}
		matched := false
		for kind, ids := range groups {
			prefix := sid + "." + kind + "."
			if !strings.HasPrefix(name, prefix) {
				continue
			}
			id, suffix, found := strings.Cut(strings.TrimPrefix(name, prefix), ".")
			wanted := "journal"
			if kind == "profile" {
				wanted = "backup"
			}
			if kind == "stage" {
				wanted = "json"
			}
			if !found || !validHex(id, 32) || (suffix != "once" && suffix != wanted) {
				return result, errors.New("noncanonical protected evidence entry")
			}
			if ids[id] == nil {
				ids[id] = map[string]bool{}
			}
			ids[id][suffix] = true
			matched = true
			result.deployed = true
			break
		}
		if !matched {
			return result, errors.New("unknown, aliased or other-client protected root entry")
		}
	}
	if !seen[sid+".once"] || !seen[sid+".json"] {
		return result, errors.New("complete original registration pair required")
	}
	if result.deployed {
		for _, name := range []string{sid + ".launch-parents.once", sid + ".launch-parents.json", sid + ".generations", sid + "." + baseline + ".baseline.once"} {
			if !seen[name] {
				return result, errors.New("complete original protected deployment parents required")
			}
		}
	}
	for kind, ids := range groups {
		if len(ids) > maxExecutionJournals {
			return result, errors.New("protected evidence capacity exceeded")
		}
		for id, pair := range ids {
			if len(pair) != 2 {
				return result, errors.New("partial protected evidence pair")
			}
			switch kind {
			case "execution":
				result.executions = append(result.executions, id)
			case "profile":
				result.profiles = append(result.profiles, id)
			case "stage":
				result.stages = append(result.stages, id)
			}
		}
	}
	sort.Strings(result.executions)
	sort.Strings(result.profiles)
	sort.Strings(result.stages)
	result.names = append([]string(nil), names...)
	sort.Strings(result.names)
	return result, nil
}
