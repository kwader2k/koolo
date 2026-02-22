package character

import (
	"log/slog"
	"time"

	"github.com/hectorgimenez/koolo/internal/action"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Smiter struct {
	BaseCharacter
}

func (s Smiter) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

func (f Smiter) CheckKeyBindings() []skill.ID {
	requireKeybindings := []skill.ID{skill.Smite, skill.HolyShield, skill.TomeOfTownPortal, skill.Fanaticism, skill.Vigor, skill.ResistLightning}
	missingKeybindings := make([]skill.ID, 0)

	for _, cskill := range requireKeybindings {
		if _, found := f.Data.KeyBindings.KeyBindingForSkill(cskill); !found {
			missingKeybindings = append(missingKeybindings, cskill)
		}
	}

	if len(missingKeybindings) > 0 {
		f.Logger.Debug("There are missing required key bindings.", slog.Any("Bindings", missingKeybindings))
	}

	return missingKeybindings
}

const (
	smiterMaxAttacksLoop = 30
)

func (f Smiter) PerformSmiteAttack(monsterID data.UnitID) {
	ctx := context.Get()
	ctx.PauseIfNotPriority()
	monster, found := f.Data.Monsters.FindByID(monsterID)
	if !found {
		return
	}

	smiteKey, found := f.Data.KeyBindings.KeyBindingForSkill(skill.Smite)
	if found && f.Data.PlayerUnit.LeftSkill != skill.Smite {
		ctx.HID.PressKeyBinding(smiteKey)
		utils.Sleep(50)
	}

	screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(monster.Position.X, monster.Position.Y)
	ctx.HID.Click(game.LeftButton, screenX, screenY)
}

func (f Smiter) KillMonsterSequence(monsterSelector func(d game.Data) (data.UnitID, bool), skipOnImmunities []stat.Resist) error {
	completedAttackLoops := 0
	previousUnitID := 0
	ctx := context.Get()

	for {
		ctx.PauseIfNotPriority()

		id, found := monsterSelector(*f.Data)
		if !found {
			return nil
		}
		if previousUnitID != int(id) {
			completedAttackLoops = 0
		}

		if !f.preBattleChecks(id, skipOnImmunities) {
			return nil
		}

		monster, found := f.Data.Monsters.FindByID(id)
		if !found {
			return nil
		}

		if monster.Stats[stat.Life] <= 0 {
			return nil
		}

		if completedAttackLoops >= smiterMaxAttacksLoop {
			completedAttackLoops = 0
			continue
		}

		var aura skill.ID
		if monster.Name == npc.UberMephisto {
			aura = f.getUberMephAura()
		} else {
			aura = skill.Fanaticism
		}

		if aura != 0 {
			if kb, found := f.Data.KeyBindings.KeyBindingForSkill(aura); found {
				if f.Data.PlayerUnit.RightSkill != aura {
					ctx.HID.PressKeyBinding(kb)
					utils.Sleep(50)
				}
			}
		}

		f.PerformSmiteAttack(id)

		completedAttackLoops++
		previousUnitID = int(id)
	}
}

func (f Smiter) getUberMephAura() skill.ID {
	ctx := context.Get()
	selectedAura := ctx.CharacterCfg.Character.Smiter.UberMephAura

	var preferredAura skill.ID
	switch selectedAura {
	case "fanaticism":
		preferredAura = skill.Fanaticism
	case "salvation":
		preferredAura = skill.Salvation
	case "resist_lightning", "":
		preferredAura = skill.ResistLightning
	default:
		preferredAura = skill.ResistLightning
	}

	if preferredAura != 0 {
		if _, found := f.Data.KeyBindings.KeyBindingForSkill(preferredAura); found {
			return preferredAura
		}
	}

	if _, found := f.Data.KeyBindings.KeyBindingForSkill(skill.ResistLightning); found {
		return skill.ResistLightning
	}

	return skill.Fanaticism
}

func (f Smiter) BuffSkills() []skill.ID {
	if _, found := f.Data.KeyBindings.KeyBindingForSkill(skill.HolyShield); found {
		return []skill.ID{skill.HolyShield}
	}
	return make([]skill.ID, 0)
}

func (f Smiter) PreCTABuffSkills() []skill.ID {
	return make([]skill.ID, 0)
}

func (f Smiter) killBoss(npc npc.ID, t data.MonsterType) error {
	return f.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		b, found := d.Monsters.FindOne(npc, t)
		if found && b.Stats[stat.Life] > 0 {
			return b.UnitID, true
		}
		return 0, false
	}, nil)
}

func (f Smiter) KillCountess() error {
	return f.killBoss(npc.DarkStalker, data.MonsterTypeSuperUnique)
}

func (f Smiter) KillAndariel() error {
	return f.killBoss(npc.Andariel, data.MonsterTypeUnique)
}

func (f Smiter) KillSummoner() error {
	return f.killBoss(npc.Summoner, data.MonsterTypeUnique)
}

func (f Smiter) KillDuriel() error {
	return f.killBoss(npc.Duriel, data.MonsterTypeUnique)
}

func (f Smiter) KillCouncil() error {
	// Disable item pickup while killing council members
	context.Get().DisableItemPickup()
	defer context.Get().EnableItemPickup()

	err := f.killAllCouncilMembers()
	if err != nil {
		return err
	}

	// Wait a moment for items to settle
	time.Sleep(300 * time.Millisecond)

	// Re-enable item pickup and do a final pickup pass
	err = action.ItemPickup(40)
	if err != nil {
		f.Logger.Warn("Error during final item pickup after council", "error", err)
	}

	return nil
}

func (f Smiter) killAllCouncilMembers() error {
	for {
		if !f.anyCouncilMemberAlive() {
			return nil
		}

		err := f.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			for _, m := range d.Monsters.Enemies() {
				if (m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3) && m.Stats[stat.Life] > 0 {
					return m.UnitID, true
				}
			}
			return 0, false
		}, nil)

		if err != nil {
			return err
		}
	}
}

func (f Smiter) anyCouncilMemberAlive() bool {
	for _, m := range f.Data.Monsters.Enemies() {
		if (m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3) && m.Stats[stat.Life] > 0 {
			return true
		}

	}
	return false
}

func (f Smiter) KillMephisto() error {
	return f.killBoss(npc.Mephisto, data.MonsterTypeUnique)
}

func (f Smiter) KillIzual() error {
	return f.killBoss(npc.Izual, data.MonsterTypeUnique)
}

func (f Smiter) KillDiablo() error {
	timeout := time.Second * 20
	startTime := time.Now()
	diabloFound := false

	for {
		if time.Since(startTime) > timeout && !diabloFound {
			f.Logger.Error("Diablo was not found, timeout reached")
			return nil
		}

		diablo, found := f.Data.Monsters.FindOne(npc.Diablo, data.MonsterTypeUnique)
		if !found || diablo.Stats[stat.Life] <= 0 {
			if diabloFound {
				return nil
			}
			time.Sleep(200 * time.Millisecond)
			continue
		}

		diabloFound = true
		f.Logger.Info("Diablo detected, attacking")

		return f.killBoss(npc.Diablo, data.MonsterTypeUnique)
	}
}

func (f Smiter) KillPindle() error {
	return f.killBoss(npc.DefiledWarrior, data.MonsterTypeSuperUnique)
}

func (f Smiter) KillNihlathak() error {
	return f.killBoss(npc.Nihlathak, data.MonsterTypeSuperUnique)
}

func (f Smiter) KillBaal() error {
	return f.killBoss(npc.BaalCrab, data.MonsterTypeUnique)
}

func (f Smiter) KillUberDuriel() error {
	return f.killBoss(npc.UberDuriel, data.MonsterTypeUnique)
}

func (f Smiter) KillUberIzual() error {
	return f.killBoss(npc.UberIzual, data.MonsterTypeUnique)
}

func (f Smiter) KillLilith() error {
	return f.killBoss(npc.Lilith, data.MonsterTypeUnique)
}

func (f Smiter) KillUberMephisto() error {
	return f.killBoss(npc.UberMephisto, data.MonsterTypeUnique)
}

func (f Smiter) KillUberDiablo() error {
	return f.killBoss(npc.UberDiablo, data.MonsterTypeUnique)
}

func (f Smiter) KillUberBaal() error {
	return f.killBoss(npc.UberBaal, data.MonsterTypeUnique)
}

func (f Smiter) InitialCharacterConfigSetup() {
	ctx := context.Get()
	ctx.CharacterCfg.Inventory.BeltColumns = [4]string{"healing", "healing", "mana", "fullrejuvenation"}
	ctx.CharacterCfg.Inventory.HealingPotionCount = 0
	ctx.CharacterCfg.Inventory.RejuvPotionCount = 0
	ctx.CharacterCfg.Inventory.ManaPotionCount = 0

	ctx.CharacterCfg.Health.ChickenAt = 20
	ctx.CharacterCfg.Health.HealingPotionAt = 70
	ctx.CharacterCfg.Health.RejuvPotionAtLife = 55
	ctx.CharacterCfg.Health.ManaPotionAt = 30

	ctx.CharacterCfg.CubeRecipes.Enabled = false
}
