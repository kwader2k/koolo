package run

import (
	"errors"
	"fmt"
	"strings"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// Positions adapted from kolbot baal.js
var throneMainPos = data.Position{X: 15095, Y: 5042}
var throneCenterPos = data.Position{X: 15093, Y: 5029}
var casterPrecastPos = data.Position{X: 15094, Y: 5027}
var hammerPrecastPos = data.Position{X: 15094, Y: 5029}
var forwardPrecastPos = data.Position{X: 15116, Y: 5026}

type Baal struct {
	ctx                *context.Status
	clearMonsterFilter data.MonsterFilter // Used to clear area (basically TZ)
	nextPreAttackAt    time.Time          // Gate pre-attack casts to avoid spamming.
	lastDecoyAt        time.Time          // Prevents decoy spam.
}

func NewBaal(clearMonsterFilter data.MonsterFilter) *Baal {
	return &Baal{
		ctx:                context.Get(),
		clearMonsterFilter: clearMonsterFilter,
	}
}

func (s Baal) Name() string {
	return string(config.BaalRun)
}

func (a Baal) CheckConditions(parameters *RunParameters) SequencerResult {
	farmingRun := IsFarmingRun(parameters)
	if !a.ctx.Data.Quests[quest.Act5RiteOfPassage].Completed() {
		if farmingRun {
			return SequencerSkip
		}
		return SequencerStop
	}
	questCompleted := a.ctx.Data.Quests[quest.Act5EveOfDestruction].Completed()
	if (farmingRun && !questCompleted) || (!farmingRun && questCompleted) {
		return SequencerSkip
	}
	return SequencerOk
}
func (s *Baal) Run(parameters *RunParameters) error {
	// Set filter
	filter := data.MonsterAnyFilter()
	if s.ctx.CharacterCfg.Game.Baal.OnlyElites {
		filter = data.MonsterEliteFilter()
	}
	if s.clearMonsterFilter != nil {
		filter = s.clearMonsterFilter
	}

	err := action.WayPoint(area.TheWorldStoneKeepLevel2)
	if err != nil {
		return err
	}

	if s.ctx.CharacterCfg.Game.Baal.ClearFloors || s.clearMonsterFilter != nil {
		action.ClearCurrentLevel(false, filter)
	}

	err = action.MoveToArea(area.TheWorldStoneKeepLevel3)
	if err != nil {
		return err
	}

	if s.ctx.CharacterCfg.Game.Baal.ClearFloors || s.clearMonsterFilter != nil {
		action.ClearCurrentLevel(false, filter)
	}

	err = action.MoveToArea(area.ThroneOfDestruction)
	if err != nil {
		return err
	}
	err = action.MoveToCoords(throneMainPos)
	if err != nil {
		return err
	}
	if s.checkForSoulsOrDolls() {
		return errors.New("souls or dolls detected, skipping")
	}

	// Let's move to a safe area and open the portal in companion mode
	if s.ctx.CharacterCfg.Companion.Leader {
		action.MoveToCoords(data.Position{X: 15116, Y: 5071})
		action.OpenTPIfLeader()
	}

	err = action.ClearAreaAroundPlayer(50, data.MonsterAnyFilter())
	if err != nil {
		return err
	}

	// Force rebuff before waves
	action.Buff()

	// Come back to previous position
	err = action.MoveToCoords(throneMainPos)
	if err != nil {
		return err
	}

	// Process waves until Baal leaves throne
	s.ctx.Logger.Info("Starting Baal waves...")
	waveTimeout := time.Now().Add(7 * time.Minute)

	lastWaveDetected := false
	isWaitingForPortal := false
	_, isLevelingChar := s.ctx.Char.(context.LevelingCharacter)

	for !s.hasBaalLeftThrone() && time.Now().Before(waveTimeout) {
		s.ctx.PauseIfNotPriority()
		s.ctx.RefreshGameData()

		// Detect last wave for logging
		if _, found := s.ctx.Data.Monsters.FindOne(npc.BaalsMinion, data.MonsterTypeMinion); found {
			if !lastWaveDetected {
				s.ctx.Logger.Info("Last wave (Baal's Minion) detected")
				lastWaveDetected = true
			}
		} else if lastWaveDetected {
			if !s.ctx.CharacterCfg.Game.Baal.KillBaal && !isLevelingChar {
				s.ctx.Logger.Info("Waves cleared, skipping Baal kill (Fast Exit).")
				return nil
			}

			if !isWaitingForPortal {
				s.ctx.Logger.Info("Waves cleared, moving to portal position to wait...")
				action.MoveToCoords(data.Position{X: 15090, Y: 5008})
				isWaitingForPortal = true
			}

			utils.Sleep(500)
			continue
		}

		if !isWaitingForPortal {
			action.ClearAreaAroundPosition(throneMainPos, 50, data.MonsterAnyFilter())
			s.preAttackBaalWaves()
		}

		utils.Sleep(60) // Prevent excessive checking
	}

	if !s.hasBaalLeftThrone() {
		return errors.New("baal waves timeout - portal never appeared")
	}

	// Baal has entered the chamber
	s.ctx.Logger.Info("Baal has entered the Worldstone Chamber")

	// Kill Baal Logic
	if s.ctx.CharacterCfg.Game.Baal.KillBaal || isLevelingChar {
		action.Buff()

		s.ctx.Logger.Info("Waiting for Baal portal...")
		var baalPortal data.Object
		found := false

		for i := 0; i < 15; i++ {
			baalPortal, found = s.ctx.Data.Objects.FindOne(object.BaalsPortal)
			if found {
				break
			}
			utils.Sleep(300)
		}

		if !found {
			return errors.New("baal portal not found after waves completed")
		}

		s.ctx.Logger.Info("Entering Baal portal...")

		// Enter portal
		err = action.InteractObject(baalPortal, func() bool {
			return s.ctx.Data.PlayerUnit.Area == area.TheWorldstoneChamber
		})

		// Verify entry
		if s.ctx.Data.PlayerUnit.Area == area.TheWorldstoneChamber {
			s.ctx.Logger.Info("Successfully entered Worldstone Chamber")
		} else if err != nil {
			return fmt.Errorf("failed to enter baal portal: %w", err)
		}

		// Move to Baal (may fail due to tentacles)
		s.ctx.Logger.Info("Moving to Baal...")
		moveErr := action.MoveToCoords(data.Position{X: 15136, Y: 5943})
		if moveErr != nil {
			if strings.Contains(moveErr.Error(), "path could not be calculated") {
				s.ctx.Logger.Info("Path blocked by tentacles, attacking from current position")
			} else {
				s.ctx.Logger.Warn("Failed to move to Baal", "error", moveErr)
			}
		}

		if err := s.ctx.Char.KillBaal(); err != nil {
			return err
		}

		action.ItemPickup(30)

		return nil
	}

	return nil
}

// hasBaalLeftThrone checks if Baal has left the throne and entered the Worldstone Chamber
func (s *Baal) hasBaalLeftThrone() bool {
	_, found := s.ctx.Data.Monsters.FindOne(npc.BaalThrone, data.MonsterTypeNone)
	return !found
}

func (s Baal) checkForSoulsOrDolls() bool {
	var npcIds []npc.ID

	if s.ctx.CharacterCfg.Game.Baal.DollQuit {
		npcIds = append(npcIds, npc.UndeadStygianDoll2, npc.UndeadSoulKiller2)
	}
	if s.ctx.CharacterCfg.Game.Baal.SoulQuit {
		npcIds = append(npcIds, npc.BlackSoul2, npc.BurningSoul2)
	}

	for _, id := range npcIds {
		if _, found := s.ctx.Data.Monsters.FindOne(id, data.MonsterTypeNone); found {
			return true
		}
	}

	return false
}

func (s *Baal) preAttackBaalWaves() {
	// Switch to Cleansing aura if poisoned.
	if s.ctx.Data.PlayerUnit.States.HasState(state.Poison) && !s.ctx.Data.PlayerUnit.States.HasState(state.Cleansing) {
		if kb, found := s.ctx.Data.KeyBindings.KeyBindingForSkill(skill.Cleansing); found && s.ctx.Data.PlayerUnit.RightSkill != skill.Cleansing {
			s.ctx.HID.PressKeyBinding(kb)
			utils.Sleep(60)         // Allow at least 1 D2R tick
			s.ctx.RefreshGameData() // Update player skills after the key press.
		}
	}
	player := s.ctx.Data.PlayerUnit

	// Pre-attack staging position.
	preAttackPosition := throneMainPos
	if player.Skills[skill.BlessedHammer].Level > 1 && (player.States.HasState(state.Meditation) || player.Class != data.Paladin) {
		// Blessed Hammer should be cast from the monster spawn area.
		preAttackPosition = hammerPrecastPos
	}
	action.MoveToCoords(preAttackPosition)

	// Pre-attack cooldown gate.
	if time.Now().Before(s.nextPreAttackAt) {
		return
	}
	originalNextPreAttackAt := s.nextPreAttackAt
	defer func() {
		// Apply the player's cast duration unless a handler set the next window.
		if originalNextPreAttackAt.Equal(s.nextPreAttackAt) {
			s.nextPreAttackAt = time.Now().Add(s.ctx.Data.PlayerCastDuration())
		}
	}()

	// Amazon
	if player.Skills[skill.Decoy].Level > 0 {
		const decoyCooldown = 10 * time.Second
		if s.lastDecoyAt.IsZero() || time.Since(s.lastDecoyAt) > decoyCooldown {
			decoyPos := data.Position{X: 15092, Y: 5028}
			if step.CastAtPosition(skill.Decoy, false, decoyPos) {
				s.lastDecoyAt = time.Now()
				return
			}
		}
	}
	// Assassin
	if player.Skills[skill.LightningSentry].Level > 0 {
		castIssued := false
		for i := 0; i < 3; i++ {
			if step.CastAtPosition(skill.LightningSentry, true, throneCenterPos) {
				castIssued = true
				utils.Sleep(int(s.ctx.Data.PlayerCastDuration().Milliseconds()))
			}
		}
		if castIssued {
			return
		}
	}
	if player.Skills[skill.DeathSentry].Level > 0 {
		castIssued := false
		for i := 0; i < 2; i++ {
			if step.CastAtPosition(skill.DeathSentry, true, throneCenterPos) {
				castIssued = true
				utils.Sleep(int(s.ctx.Data.PlayerCastDuration().Milliseconds()))
			}
		}
		if castIssued {
			return
		}
	}
	if player.Skills[skill.ShockWeb].Level > 0 {
		if step.CastAtPosition(skill.ShockWeb, true, throneCenterPos) {
			return
		}
	}
	// Druid
	if player.Skills[skill.Tornado].Level > 0 {
		if step.CastAtPosition(skill.Tornado, true, throneCenterPos) {
			return
		}
	}
	if player.Skills[skill.Fissure].Level > 0 {
		if step.CastAtPosition(skill.Fissure, true, forwardPrecastPos) {
			return
		}
	}
	if player.Skills[skill.Volcano].Level > 0 {
		if step.CastAtPosition(skill.Volcano, true, forwardPrecastPos) {
			return
		}
	}
	// Paladin
	// Paladin pre-casts are only worth it with Meditation active.
	// Still check non-paladins in case Paladin skills are available via items.
	if player.States.HasState(state.Meditation) || player.Class != data.Paladin {
		if player.Skills[skill.BlessedHammer].Level > 1 {
			// Switch to Concentration if not under Cleansing or no longer poisoned.
			if kb, found := s.ctx.Data.KeyBindings.KeyBindingForSkill(skill.Concentration); found && player.RightSkill != skill.Concentration && (player.RightSkill != skill.Cleansing || !player.States.HasState(state.Poison)) {
				s.ctx.HID.PressKeyBinding(kb)
				utils.Sleep(60) // Allow at least 1 D2R tick
			}
			if step.CastAtPosition(skill.BlessedHammer, true, hammerPrecastPos) {
				return
			}
		}
		if player.Skills[skill.HolyBolt].Level > 1 {
			if step.CastAtPosition(skill.HolyBolt, true, hammerPrecastPos) {
				return
			}
		}
	}
	// Necromancer
	if player.Skills[skill.PoisonNova].Level > 0 {
		if step.CastAtPosition(skill.PoisonNova, true, player.Position) {
			return
		}
	}
	if player.Skills[skill.DimVision].Level > 0 {
		if step.CastAtPosition(skill.DimVision, true, casterPrecastPos) {
			return
		}
	}
	// Sorceress
	if player.Skills[skill.Blizzard].Level > 0 {
		if step.CastAtPosition(skill.Blizzard, true, casterPrecastPos) {
			return
		}
	}
	if player.Skills[skill.Meteor].Level > 0 {
		if step.CastAtPosition(skill.Meteor, true, casterPrecastPos) {
			return
		}
	}
	if player.Skills[skill.FrozenOrb].Level > 0 {
		if step.CastAtPosition(skill.FrozenOrb, true, casterPrecastPos) {
			return
		}
	}

	s.nextPreAttackAt = time.Now().Add(40 * time.Millisecond) // Short cooldown to re-check when no pre-attack fired.
}
