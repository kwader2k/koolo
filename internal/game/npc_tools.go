package game

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
)

func IsActBoss(m data.Monster) bool {
	switch m.Name {
	case npc.Andariel:
	case npc.Duriel:
	case npc.Mephisto:
	case npc.Diablo:
	case npc.BaalCrab:
		return true
	}
	return false
}

// TODO: Remove as the helper now lives in d2go
func IsMonsterSealElite(m data.Monster) bool {
	return m.IsSealElite()
}

func IsQuestEnemy(m data.Monster) bool {
	if IsActBoss(m) {
		return true
	}
	if m.IsSealElite() {
		return true
	}
	switch m.Name {
	case npc.BloodRaven:
	case npc.Radament:
	case npc.Summoner:
	case npc.CouncilMember:
	case npc.CouncilMember2:
	case npc.CouncilMember3:
	case npc.Izual:
	case npc.Hephasto:
	case npc.Nihlathak:
		return true
	}
	return false
}
