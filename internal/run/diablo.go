package run

import (
	"errors"
	"fmt"
	"slices"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

var diabloSpawnPosition = data.Position{X: 7792, Y: 5294}
var diabloFightPosition = data.Position{X: 7788, Y: 5292}
var chaosNavToPosition = data.Position{X: 7732, Y: 5292} // into path towards vizier

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

func (d Diablo) CheckConditions(parameters *RunParameters) SequencerResult {
	farmingRun := IsFarmingRun(parameters)
	questCompleted := d.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed()
	if farmingRun && !questCompleted {
		return SequencerSkip
	}
	if !farmingRun && questCompleted {
		if slices.Contains(d.ctx.Data.PlayerUnit.AvailableWaypoints, area.Harrogath) || d.ctx.Data.PlayerUnit.Area.Act() == 5 {
			return SequencerSkip
		}
		// Workaround AvailableWaypoints only filled when wp menu has been opened on act page
		// Check if any act 5 quest is started or completed
		if action.HasAnyQuestStartedOrCompleted(quest.Act5SiegeOnHarrogath, quest.Act5EveOfDestruction) {
			return SequencerSkip
		}
	}
	return SequencerOk
}

func (d *Diablo) Run(parameters *RunParameters) error {
	if IsQuestRun(parameters) && d.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed() {
		if err := d.goToAct5(); err != nil {
			return err
		}
		return nil
	}

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
			// Removed large clearAreaAroundPlayer(50) - too slow & risky
		}

		if !d.ctx.Data.CanTeleport() {
			d.ctx.Logger.Debug("Non-teleporting character detected, clearing path to Vizier from star")
			err := action.MoveToCoords(chaosNavToPosition,
				step.WithClearPathOverride(20), // ← REDUCED from 30/40
				step.WithMonsterFilter(d.getMonsterFilter()))
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
			// Removed large clearAreaAroundPlayer(50)
		}

		//path through towards vizier
		err := action.MoveToCoords(chaosNavToPosition,
			step.WithClearPathOverride(20), // ← REDUCED from 40
			step.WithMonsterFilter(d.getMonsterFilter()))
		if err != nil {
			return err
		}
	}

	d.ctx.RefreshGameData()

	sealGroups := map[string][]object.Name{
		"Vizier":      {object.DiabloSeal4, object.DiabloSeal5},
		"Lord De Seis": {object.DiabloSeal3},
		"Infector":    {object.DiabloSeal1, object.DiabloSeal2},
	}

	for _, bossName := range []string{"Vizier", "Lord De Seis", "Infector"} {
		d.ctx.Logger.Debug(fmt.Sprint("Heading to ", bossName))

		for _, sealID := range sealGroups[bossName] {
			seal, found := d.ctx.Data.Objects.FindOne(sealID)
			if !found {
				return fmt.Errorf("seal not found: %d", sealID)
			}

			err := action.MoveToCoords(seal.Position,
				step.WithClearPathOverride(25), // ← REDUCED from 35
				step.WithMonsterFilter(d.getMonsterFilter()))
			if err != nil {
				return err
			}

			// Handle the special case for DiabloSeal3
			if sealID == object.DiabloSeal3 && seal.Position.X == 7773 && seal.Position.Y == 5155 {
				if err = action.MoveToCoords(data.Position{X: 7768, Y: 5160},
					step.WithClearPathOverride(25),
					step.WithMonsterFilter(d.getMonsterFilter())); err != nil {
					return fmt.Errorf("failed to move to bugged seal position: %w", err)
				}
			}

			// Smaller clear - faster & less exposure
			action.ClearAreaAroundPlayer(15, d.ctx.Data.MonsterFilterAnyReachable())

			// Buff refresh before Infector only
			if object.DiabloSeal1 == sealID {
				action.Buff()
			}

			maxAttemptsToOpenSeal := 3 // ← REDUCED from 5
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
					utils.PingSleep(utils.Light, 50) // ← FASTER from 100
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
			if err := d.killSealElite(bossName); err != nil && bossName != "Lord De Seis" {
				return err
			}
		}
	}

	if d.ctx.CharacterCfg.Game.Diablo.KillDiablo {
		// Buff BEFORE setting ClearPathDist to 0
		action.Buff()

		originalClearPathDistCfg := d.ctx.CharacterCfg.Character.ClearPathDist
		d.ctx.CharacterCfg.Character.ClearPathDist = 0
		defer func() {
			d.ctx.CharacterCfg.Character.ClearPathDist = originalClearPathDistCfg
		}()

		if isLevelingChar && d.ctx.CharacterCfg.Game.Difficulty == difficulty.Normal {
			action.MoveToCoords(diabloSpawnPosition)
			action.InRunReturnTownRoutine()
			step.MoveTo(diabloFightPosition, step.WithIgnoreMonsters())
		} else {
			action.MoveToCoords(diabloSpawnPosition)
		}

		if d.ctx.CharacterCfg.Game.Diablo.DisableItemPickupDuringBosses {
			d.ctx.DisableItemPickup()
		}

		if err := d.ctx.Char.KillDiablo(); err != nil {
			return err
		}

		action.ItemPickup(30)

		if IsQuestRun(parameters) {
			if err := d.goToAct5(); err != nil {
				return err
			}
		}
	}

	return nil
}

func (d *Diablo) killSealElite(boss string) error {
	d.ctx.Logger.Debug(fmt.Sprintf("Aggro kill %s", boss))
	startTime := time.Now()
	timeout := 8 * time.Second // ← REDUCED from 12s

	_, isLevelingChar := d.ctx.Char.(context.LevelingCharacter)
	sealElite := data.Monster{}
	sealEliteAlreadyDead := false
	sealEliteDetected := false

	var bossNPCID npc.ID
	switch boss {
	case "Vizier":
		bossNPCID = npc.StormCaster
	case "Lord De Seis":
		bossNPCID = npc.OblivionKnight
	case "Infector":
		bossNPCID = npc.VenomLord
	}

	for time.Since(startTime) < timeout {
		d.ctx.PauseIfNotPriority()
		d.ctx.RefreshGameData()

		for _, m := range d.ctx.Data.Monsters.Enemies(d.ctx.Data.MonsterFilterAnyReachable()) {
			if action.IsMonsterSealElite(m) && m.Name == bossNPCID {
				sealElite = m
				sealEliteDetected = true
				goto killLoop
			}
		}

		if sealElite.UnitID == 0 {
			for _, corpse := range d.ctx.Data.Corpses {
				if action.IsMonsterSealElite(corpse) && corpse.Name == bossNPCID {
					sealEliteAlreadyDead = true
					break
				}
			}
		}

		if sealElite.UnitID != 0 || sealEliteAlreadyDead {
			break
		}

		utils.PingSleep(utils.Light, 50) // ← FASTER from 150
	}

	if sealEliteAlreadyDead {
		return nil
	}

	if sealElite.UnitID == 0 {
		action.ClearAreaAroundPlayer(25, data.MonsterAnyFilter()) // Quick fallback
		d.ctx.RefreshGameData()

		for _, corpse := range d.ctx.Data.Corpses {
			if action.IsMonsterSealElite(corpse) && corpse.Name == bossNPCID {
				return nil
			}
		}

		if boss == "Lord De Seis" {
			d.ctx.Logger.Debug("Lord De Seis not found but this is acceptable, continuing")
			return nil
		}
		return fmt.Errorf("no seal elite found for %s within %v seconds", boss, timeout)
	}

killLoop:
	killSealEliteAttempts := 0
	for killSealEliteAttempts <= 4 { // ← REDUCED from 8
		d.ctx.PauseIfNotPriority()
		d.ctx.RefreshGameData()

		m, found := d.ctx.Data.Monsters.FindByID(sealElite.UnitID)
		if d.ctx.Data.PlayerUnit.Area.IsTown() {
			utils.PingSleep(utils.Light, 50)
			continue
		}

		if !found {
			// Re-detect
			for _, monster := range d.ctx.Data.Monsters.Enemies(d.ctx.Data.MonsterFilterAnyReachable()) {
				if action.IsMonsterSealElite(monster) && monster.Name == bossNPCID {
					sealElite = monster
					found = true
					break
				}
			}

			if !found {
				for _, corpse := range d.ctx.Data.Corpses {
					if action.IsMonsterSealElite(corpse) && corpse.Name == bossNPCID {
						d.ctx.Logger.Debug(fmt.Sprintf("Successfully killed seal elite %s (found in corpses)", boss))
						return nil
					}
				}
				if killSealEliteAttempts > 2 {
					return fmt.Errorf("seal elite %s not found after first detection", boss)
				}
				utils.PingSleep(utils.Light, 50)
				continue
			}
		}

		killSealEliteAttempts++
		sealElite = m

		// HP Safety check
		if d.ctx.Data.PlayerUnit.HPPercent() < 50 {
			action.UsePotion()
		}

		clearRadius := 30 // Fixed aggressive value

		err := action.ClearAreaAroundPosition(sealElite.Position, clearRadius, func(monsters data.Monsters) (filteredMonsters []data.Monster) {
			if isLevelingChar {
				filteredMonsters = append(filteredMonsters, monsters...)
			} else {
				filteredMonsters = append(filteredMonsters, sealElite) // Focus boss
			}
			return filteredMonsters
		})

		if err != nil {
			d.ctx.Logger.Error(fmt.Sprintf("Failed to clear area around seal elite %s: %v", boss, err))
			continue
		}

		d.ctx.RefreshGameData()

		corpseFound := false
		for _, corpse := range d.ctx.Data.Corpses {
			if action.IsMonsterSealElite(corpse) && corpse.Name == bossNPCID {
				d.ctx.Logger.Debug(fmt.Sprintf("Successfully killed seal elite %s after %d attempts (found in corpses)", boss, killSealEliteAttempts))
				return nil
			}
		}

		bossStillAlive := false
		for _, m := range d.ctx.Data.Monsters.Enemies(d.ctx.Data.MonsterFilterAnyReachable()) {
			if action.IsMonsterSealElite(m) && m.Name == bossNPCID {
				bossStillAlive = true
				break
			}
		}

		if sealEliteDetected && !bossStillAlive && !corpseFound {
			d.ctx.Logger.Debug(fmt.Sprintf("Successfully killed seal elite %s after %d attempts (corpse likely shattered)", boss, killSealEliteAttempts))
			return nil
		}

		utils.PingSleep(utils.Light, 50) // ← FASTER
	}

	return fmt.Errorf("failed to kill seal elite %s after %d attempts", boss, killSealEliteAttempts)
}

func (d *Diablo) getMonsterFilter() data.MonsterFilter {
	return func(monsters data.Monsters) (filteredMonsters []data.Monster) {
		for _, m := range monsters {
			if !d.ctx.Data.AreaData.IsWalkable(m.Position) {
				continue
			}

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

func (d *Diablo) goToAct5() error {
	err := action.WayPoint(area.ThePandemoniumFortress)
	if err != nil {
		return err
	}

	err = action.InteractNPC(npc.Tyrael2)
	if err != nil {
		return err
	}

	//Choose travel to harrogath option
	d.ctx.HID.KeySequence(win.VK_DOWN, win.VK_RETURN)
	utils.Sleep(1000)
	d.ctx.RefreshGameData()
	utils.Sleep(1000)
	d.trySkipCinematic()

	if d.ctx.Data.PlayerUnit.Area.Act() != 5 {
		harrogathPortal, found := d.ctx.Data.Objects.FindOne(object.LastLastPortal)
		if found {
			err = action.InteractObject(harrogathPortal, func() bool {
				utils.Sleep(100)
				ctx := context.Get()
				return !ctx.Manager.InGame() || d.ctx.Data.PlayerUnit.Area.Act() == 5
			})
			if err != nil {
				return err
			}
			d.trySkipCinematic()
		}
		return errors.New("failed to go to act 5")
	}
	return nil
}

func (d Diablo) trySkipCinematic() {
	if !d.ctx.Manager.InGame() {
		utils.Sleep(2000)
		action.HoldKey(win.VK_SPACE, 2000)
		utils.Sleep(2000)
		action.HoldKey(win.VK_SPACE, 2000)
		utils.Sleep(2000)
	}
}