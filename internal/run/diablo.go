package run

import (
	"fmt"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

var diabloSpawnPosition = data.Position{X: 7792, Y: 5294}
var diabloFightPosition = data.Position{X: 7788, Y: 5292}
var chaosNavToPosition = data.Position{X: 7732, Y: 5292} //into path towards vizier

type Diablo struct {
	ctx *context.Status
}

func NewDiablo() *Diablo {
	return &Diablo{
		ctx: context.Get(),
	}
}

func (d *Diablo) Name() string {
	return string(config.DiabloRun)
}

func (d *Diablo) Run() error {
	// Just to be sure we always re-enable item pickup after the run
	defer func() {
		d.ctx.EnableItemPickup()
	}()

	if err := action.WayPoint(area.RiverOfFlame); err != nil {
		return err
	}

	_, isLevelingChar := d.ctx.Char.(context.LevelingCharacter)

	if err := action.MoveToArea(area.ChaosSanctuary); err != nil {
		return err
	}

	if isLevelingChar {
		action.Buff()
	}

	// We move directly to Diablo spawn position if StartFromStar is enabled, not clearing the path
	d.ctx.Logger.Debug(fmt.Sprintf("StartFromStar value: %t", d.ctx.CharacterCfg.Game.Diablo.StartFromStar))
	if d.ctx.CharacterCfg.Game.Diablo.StartFromStar {
		if d.ctx.Data.CanTeleport() {
			if err := action.MoveToCoords(diabloSpawnPosition, step.WithIgnoreMonsters()); err != nil {
				return err
			}
		} else {
			//move to star
			if err := action.MoveToCoords(diabloSpawnPosition, step.WithMonsterFilter(d.getMonsterFilter())); err != nil {
				return err
			}
		}
		//open portal if leader
		if d.ctx.CharacterCfg.Companion.Leader {
			action.OpenTPIfLeader()
			action.Buff()
			action.ClearAreaAroundPlayer(30, data.MonsterAnyFilter())
		}

		if !d.ctx.Data.CanTeleport() {
			d.ctx.Logger.Debug("Non-teleporting character detected, clearing path to Vizier from star")
			err := action.MoveToCoords(chaosNavToPosition, step.WithClearPathOverride(30), step.WithMonsterFilter(d.getMonsterFilter()))
			if err != nil {
				d.ctx.Logger.Error(fmt.Sprintf("Failed to clear path to Vizier from star: %v", err))
				return err
			}
			d.ctx.Logger.Debug("Successfully cleared path to Vizier from star")
		}
	} else {
		//open portal in entrance
		if d.ctx.CharacterCfg.Companion.Leader {
			action.OpenTPIfLeader()
			action.Buff()
			action.ClearAreaAroundPlayer(30, data.MonsterAnyFilter())
		}
		//path through towards vizier
		err := action.MoveToCoords(chaosNavToPosition, step.WithClearPathOverride(30), step.WithMonsterFilter(d.getMonsterFilter()))
		if err != nil {
			return err
		}
	}

	d.ctx.RefreshGameData()
	sealGroups := map[string][]object.Name{
		"Vizier":       {object.DiabloSeal4, object.DiabloSeal5}, // Vizier
		"Lord De Seis": {object.DiabloSeal3},                     // Lord De Seis
		"Infector":     {object.DiabloSeal1, object.DiabloSeal2}, // Infector
	}

	// Thanks Go for the lack of ordered maps
	for _, bossName := range []string{"Vizier", "Lord De Seis", "Infector"} {
		d.ctx.Logger.Debug(fmt.Sprint("Heading to ", bossName))

		for _, sealID := range sealGroups[bossName] {
			seal, found := d.ctx.Data.Objects.FindOne(sealID)
			if !found {
				return fmt.Errorf("seal not found: %d", sealID)
			}

			err := action.MoveToCoords(seal.Position, step.WithClearPathOverride(20), step.WithMonsterFilter(d.getMonsterFilter()))
			if err != nil {
				return err
			}

			// Handle the special case for DiabloSeal3
			if sealID == object.DiabloSeal3 && seal.Position.X == 7773 && seal.Position.Y == 5155 {
				if err = action.MoveToCoords(data.Position{X: 7768, Y: 5160}, step.WithClearPathOverride(20), step.WithMonsterFilter(d.getMonsterFilter())); err != nil {
					return fmt.Errorf("failed to move to bugged seal position: %w", err)
				}
			}

			// Clear everything around the seal
			action.ClearAreaAroundPlayer(10, d.ctx.Data.MonsterFilterAnyReachable())

			//Buff refresh before Infector
			if object.DiabloSeal1 == sealID || isLevelingChar {
				action.Buff()
			}

			// Smart Diablo Kill: Go to town before opening last seal (DiabloSeal2)
			// This ensures potions are refilled BEFORE Diablo spawns
			if sealID == object.DiabloSeal2 && isLevelingChar &&
				d.ctx.CharacterCfg.Game.Difficulty == difficulty.Normal &&
				d.ctx.CharacterCfg.Character.SorceressLeveling.UseSmartDiabloKill {
				d.ctx.Logger.Debug("Smart Diablo Kill: Going to town before opening last seal")
				action.InRunReturnTownRoutine()
			}

			maxAttemptsToOpenSeal := 3
			attempts := 0

			for attempts < maxAttemptsToOpenSeal {
				seal, _ = d.ctx.Data.Objects.FindOne(sealID)

				if !seal.Selectable {
					break
				}

				if err = action.InteractObject(seal, func() bool {
					seal, _ = d.ctx.Data.Objects.FindOne(sealID)
					return !seal.Selectable
				}); err != nil {
					d.ctx.Logger.Error(fmt.Sprintf("Attempt %d to interact with seal %d: %v failed", attempts+1, sealID, err))
					d.ctx.PathFinder.RandomMovement()
					utils.Sleep(200)
				}

				attempts++
			}

			seal, _ = d.ctx.Data.Objects.FindOne(sealID)
			if seal.Selectable {
				d.ctx.Logger.Error(fmt.Sprintf("Failed to open seal %d after %d attempts", sealID, maxAttemptsToOpenSeal))
				return fmt.Errorf("failed to open seal %d after %d attempts", sealID, maxAttemptsToOpenSeal)
			}

			// Infector spawns when first seal is enabled
			if object.DiabloSeal1 == sealID {
				if err = d.killSealElite(bossName); err != nil {
					return err
				}
			}
		}

		// Skip Infector boss because was already killed
		if bossName != "Infector" {
			// Wait for the boss to spawn and kill it.
			// Lord De Seis sometimes it's far, and we can not detect him, but we will kill him anyway heading to the next seal
			if err := d.killSealElite(bossName); err != nil && bossName != "Lord De Seis" {
				return err
			}
		}

	}

	if d.ctx.CharacterCfg.Game.Diablo.KillDiablo {

		originalClearPathDistCfg := d.ctx.CharacterCfg.Character.ClearPathDist
		d.ctx.CharacterCfg.Character.ClearPathDist = 0

		defer func() {
			d.ctx.CharacterCfg.Character.ClearPathDist = originalClearPathDistCfg

		}()

		action.Buff()

		// Skip town visit if smart kill already handled it before last seal
		shouldDoTownVisit := isLevelingChar &&
			d.ctx.CharacterCfg.Game.Difficulty == difficulty.Normal &&
			!d.ctx.CharacterCfg.Character.SorceressLeveling.UseSmartDiabloKill

		if shouldDoTownVisit {
			action.MoveToCoords(diabloSpawnPosition)
			action.InRunReturnTownRoutine()
			step.MoveTo(diabloFightPosition, step.WithIgnoreMonsters())
		} else {
			action.MoveToCoords(diabloSpawnPosition)
		}

		// Check if we should disable item pickup for Diablo
		if d.ctx.CharacterCfg.Game.Diablo.DisableItemPickupDuringBosses {
			d.ctx.DisableItemPickup()
		}

		return d.ctx.Char.KillDiablo()

	}

	return nil
}

func (d *Diablo) killSealElite(boss string) error {
	d.ctx.Logger.Debug(fmt.Sprintf("Starting kill sequence for %s", boss))
	startTime := time.Now()

	timeout := 15 * time.Second

	_, isLevelingChar := d.ctx.Char.(context.LevelingCharacter)
	sealElite := data.Monster{}
	for time.Since(startTime) < timeout {
		d.ctx.PauseIfNotPriority()
		d.ctx.RefreshGameData()

		for _, m := range d.ctx.Data.Monsters.Enemies(d.ctx.Data.MonsterFilterAnyReachable()) {
			if action.IsMonsterSealElite(m) {
				sealElite = m
				//d.ctx.Logger.Debug(fmt.Sprintf("Seal elite found: %v at position X: %d, Y: %d", m.Name, m.Position.X, m.Position.Y))
				break
			}
		}

		if sealElite.UnitID != 0 {
			//Seal elite found, stop detection loop
			break
		}

		//Reset time
		if d.ctx.Data.PlayerUnit.Area.IsTown() {
			startTime = time.Now()
		}

		utils.Sleep(250)
	}

	utils.Sleep(500)

	killSealEliteAttempts := 0
	if sealElite.UnitID != 0 {
		for killSealEliteAttempts <= 5 {
			d.ctx.PauseIfNotPriority()
			d.ctx.RefreshGameData()
			m, found := d.ctx.Data.Monsters.FindByID(sealElite.UnitID)

			//If in town, wait until back to battlefield
			if d.ctx.Data.PlayerUnit.Area.IsTown() {
				utils.Sleep(100)
				continue
			}

			if !found {
				if _, corpseFound := d.ctx.Data.Corpses.FindByID(sealElite.UnitID); corpseFound {
					d.ctx.Logger.Debug(fmt.Sprintf("Successfully killed seal elite %s after %d attempts", boss, killSealEliteAttempts))
					return nil
				} else {
					return fmt.Errorf("seal elite %s not found after first detection ", boss)
				}
			}

			killSealEliteAttempts++
			sealElite = m

			var clearRadius int
			if d.ctx.Data.CanTeleport() {
				clearRadius = 30
			} else {
				clearRadius = 40
			}

			//d.ctx.Logger.Debug(fmt.Sprintf("Clearing area around seal elite with radius %d", clearRadius))

			err := action.ClearAreaAroundPosition(sealElite.Position, clearRadius, func(monsters data.Monsters) (filteredMonsters []data.Monster) {
				if isLevelingChar {
					filteredMonsters = append(filteredMonsters, monsters...)
				} else {
					filteredMonsters = append(filteredMonsters, sealElite)
				}
				return filteredMonsters
			})

			if err != nil {
				d.ctx.Logger.Error(fmt.Sprintf("Failed to clear area around seal elite %s: %v", boss, err))
				continue
			}

			utils.Sleep(250)
		}
	} else {
		return fmt.Errorf("no seal elite found for %s within %v seconds", boss, timeout.Seconds())
	}

	return fmt.Errorf("failed to kill seal elite %s after %d attempts", boss, killSealEliteAttempts)
}

func (d *Diablo) getMonsterFilter() data.MonsterFilter {
	return func(monsters data.Monsters) (filteredMonsters []data.Monster) {
		for _, m := range monsters {
			if !d.ctx.Data.AreaData.IsWalkable(m.Position) {
				continue
			}

			// If FocusOnElitePacks is enabled, only return elite monsters and seal bosses
			if d.ctx.CharacterCfg.Game.Diablo.FocusOnElitePacks {
				if m.IsElite() || action.IsMonsterSealElite(m) {
					filteredMonsters = append(filteredMonsters, m)
				}
			} else {
				filteredMonsters = append(filteredMonsters, m)
			}
		}

		return filteredMonsters
	}
}
