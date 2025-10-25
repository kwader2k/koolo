package run

import (
	"errors"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

var baalThronePosition = data.Position{
	X: 15095,
	Y: 5042,
}

type Baal struct {
	ctx                *context.Status
	clearMonsterFilter data.MonsterFilter // Used to clear area (basically TZ)
	preAtkLast         time.Time
	decoyLast          time.Time
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

func (s *Baal) Run() error {
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
	err = action.MoveToCoords(baalThronePosition)
	if err != nil {
		return err
	}
	if s.checkForSoulsOrDolls() {
		return errors.New("souls or dolls detected, skipping")
	}

	// Let's move to a safe area and open the portal in companion mode
	if s.ctx.CharacterCfg.Companion.Leader {
		action.MoveToCoords(data.Position{
			X: 15116,
			Y: 5071,
		})
		action.OpenTPIfLeader()
	}

	err = action.ClearAreaAroundPlayer(50, data.MonsterAnyFilter())
	if err != nil {
		return err
	}

	// Force rebuff before waves
	action.Buff()

	// Come back to previous position
	err = action.MoveToCoords(baalThronePosition)
	if err != nil {
		return err
	}

	lastWave := false
	waveTimeout := time.Now().Add(7 * time.Minute)
	noMonstersCount := 0                           

	for !lastWave && time.Now().Before(waveTimeout) {
		if _, found := s.ctx.Data.Monsters.FindOne(npc.BaalsMinion, data.MonsterTypeMinion); found {
			lastWave = true
		}

		if baalPortal, foundPortal := s.ctx.Data.Objects.FindOne(object.BaalsPortal); foundPortal {
			if baalPortal.Selectable {
				lastWave = true
			}
		}

		
		monstersNearby := []data.Monster{}
		for _, m := range s.ctx.Data.Monsters.Enemies(data.MonsterAnyFilter()) {
			distance := s.ctx.PathFinder.DistanceFromMe(m.Position)
			if distance <= 40 { // Only check monsters within 40 units
				monstersNearby = append(monstersNearby, m)
			}
		}

		if len(monstersNearby) == 0 {
			noMonstersCount++
			// If no monsters detected for 3 consecutive checks, assume wave is complete
			if noMonstersCount >= 3 {
				utils.Sleep(2000) // Wait a bit for next wave to spawn
				// Check one more time for portal or minions
				if baalPortal, foundPortal := s.ctx.Data.Objects.FindOne(object.BaalsPortal); foundPortal {
					if baalPortal.Selectable {
						lastWave = true
						break
					}
				}
				if _, found := s.ctx.Data.Monsters.FindOne(npc.BaalsMinion, data.MonsterTypeMinion); found {
					lastWave = true
					break
				}
			}
		} else {
			noMonstersCount = 0
		}

		// Return to throne position between waves
		err = action.ClearAreaAroundPosition(baalThronePosition, 50, data.MonsterAnyFilter())
		if err != nil {
			return err
		}

		action.MoveToCoords(baalThronePosition)

		// Preattack between waves (inspired by kolbot baal.js)
		s.preAttackBaalWaves()
	}

	// Check if we timed out
	if !lastWave {
		return errors.New("baal waves timeout - portal never appeared or minions never detected")
	}

	// Let's be sure everything is dead
	if err = action.ClearAreaAroundPosition(baalThronePosition, 50, data.MonsterAnyFilter()); err != nil {
		return err
	}

	_, isLevelingChar := s.ctx.Char.(context.LevelingCharacter)
	if s.ctx.CharacterCfg.Game.Baal.KillBaal || isLevelingChar {
		utils.Sleep(15000)
		action.Buff()
		// Exception: Baal portal has no destination in memory
		baalPortal, _ := s.ctx.Data.Objects.FindOne(object.BaalsPortal)
		err = action.InteractObject(baalPortal, func() bool {
			return s.ctx.Data.PlayerUnit.Area == area.TheWorldstoneChamber
		})
		if err != nil {
			return err
		}

		_ = action.MoveToCoords(data.Position{X: 15136, Y: 5943})

		return s.ctx.Char.KillBaal()
	}

	return nil
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
	// Positions adapted from kolbot baal.js preattack
	blizzPos := data.Position{X: 15094, Y: 5027}
	hammerPos := data.Position{X: 15094, Y: 5029}
	throneCenter := data.Position{X: 15093, Y: 5029}
	forwardPos := data.Position{X: 15116, Y: 5026}

	// Simple global cooldown between preattacks to avoid spam
	const preAtkCooldown = 1500 * time.Millisecond
	if !s.preAtkLast.IsZero() && time.Since(s.preAtkLast) < preAtkCooldown {
		return
	}

	if s.ctx.Data.PlayerUnit.Skills[skill.Blizzard].Level > 0 {
		step.CastAtPosition(skill.Blizzard, true, blizzPos)
		s.preAtkLast = time.Now()
		return
	}

	if s.ctx.Data.PlayerUnit.Skills[skill.Meteor].Level > 0 {
		step.CastAtPosition(skill.Meteor, true, blizzPos)
		s.preAtkLast = time.Now()
		return
	}
	if s.ctx.Data.PlayerUnit.Skills[skill.FrozenOrb].Level > 0 {
		step.CastAtPosition(skill.FrozenOrb, true, blizzPos)
		s.preAtkLast = time.Now()
		return
	}

	if s.ctx.Data.PlayerUnit.Skills[skill.BlessedHammer].Level > 0 {
		if kb, found := s.ctx.Data.KeyBindings.KeyBindingForSkill(skill.Concentration); found {
			s.ctx.HID.PressKeyBinding(kb)
		}
		step.CastAtPosition(skill.BlessedHammer, true, hammerPos)
		s.preAtkLast = time.Now()
		return
	}

	if s.ctx.Data.PlayerUnit.Skills[skill.Decoy].Level > 0 {
		const decoyCooldown = 10 * time.Second
		if s.decoyLast.IsZero() || time.Since(s.decoyLast) > decoyCooldown {
			decoyPos := data.Position{X: 15092, Y: 5028}
			step.CastAtPosition(skill.Decoy, false, decoyPos)
			s.decoyLast = time.Now()
			s.preAtkLast = time.Now()
			return
		}
	}

	if s.ctx.Data.PlayerUnit.Skills[skill.PoisonNova].Level > 0 {
		step.CastAtPosition(skill.PoisonNova, true, s.ctx.Data.PlayerUnit.Position)
		s.preAtkLast = time.Now()
		return
	}
	if s.ctx.Data.PlayerUnit.Skills[skill.DimVision].Level > 0 {
		step.CastAtPosition(skill.DimVision, true, blizzPos)
		s.preAtkLast = time.Now()
		return
	}

	// Druid:
	if s.ctx.Data.PlayerUnit.Skills[skill.Tornado].Level > 0 {
		step.CastAtPosition(skill.Tornado, true, throneCenter)
		s.preAtkLast = time.Now()
		return
	}
	if s.ctx.Data.PlayerUnit.Skills[skill.Fissure].Level > 0 {
		step.CastAtPosition(skill.Fissure, true, forwardPos)
		s.preAtkLast = time.Now()
		return
	}
	if s.ctx.Data.PlayerUnit.Skills[skill.Volcano].Level > 0 {
		step.CastAtPosition(skill.Volcano, true, forwardPos)
		s.preAtkLast = time.Now()
		return
	}

	// Assassin:
	if s.ctx.Data.PlayerUnit.Skills[skill.LightningSentry].Level > 0 {
		for i := 0; i < 3; i++ {
			step.CastAtPosition(skill.LightningSentry, true, throneCenter)
			utils.Sleep(80)
		}
		s.preAtkLast = time.Now()
		return
	}
	if s.ctx.Data.PlayerUnit.Skills[skill.DeathSentry].Level > 0 {
		for i := 0; i < 2; i++ {
			step.CastAtPosition(skill.DeathSentry, true, throneCenter)
			utils.Sleep(80)
		}
		s.preAtkLast = time.Now()
		return
	}
	if s.ctx.Data.PlayerUnit.Skills[skill.ShockWeb].Level > 0 {
		step.CastAtPosition(skill.ShockWeb, true, throneCenter)
		s.preAtkLast = time.Now()
		return
	}
}
