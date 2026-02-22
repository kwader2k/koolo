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

		//Workaround AvailableWaypoints only filled when wp menu has been opened on act page
		//Check if any act 5 quest is started or completed
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
					utils.PingSleep(utils.Medium, 200)
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

		// Buff BEFORE setting ClearPathDist to 0, so bot can defend itself during buff
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

		// Check if we should disable item pickup for Diablo
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

func (d *Diablo) killSealElite(sealEliteName string) error {
	d.ctx.Logger.Debug(fmt.Sprintf("Starting kill sequence for %s", sealEliteName))
	startTime := time.Now()

	timeout := 20 * time.Second

	_, isLevelingChar := d.ctx.Char.(context.LevelingCharacter)
	sealElite := data.Monster{}
	sealEliteAlreadyDead := false
	sealEliteDetected := false // Track if we ever detected the boss alive

	// Map seal elite name to NPC ID for corpse checking
	var sealEliteID npc.ID
	switch sealEliteName {
	case "Vizier":
		sealEliteID = npc.StormCaster
	case "Lord De Seis":
		sealEliteID = npc.OblivionKnight
	case "Infector":
		sealEliteID = npc.VenomLord
	}

	for time.Since(startTime) < timeout {
		d.ctx.PauseIfNotPriority()
		d.ctx.RefreshGameData()

		// Check for living seal elite
		for _, m := range d.ctx.Data.Monsters.Enemies(d.ctx.Data.MonsterFilterAnyReachable()) {
			if action.IsMonsterSealElite(m) && m.Name == sealEliteID {
				sealElite = m
				sealEliteDetected = true // Mark as detected
				break
			}
		}

		// If not found alive, check if already dead in corpses
		if sealElite.UnitID == 0 {
			for _, corpse := range d.ctx.Data.Corpses {
				if action.IsMonsterSealElite(corpse) && corpse.Name == sealEliteID {
					sealEliteAlreadyDead = true
					break
				}
			}
		}

		if sealElite.UnitID != 0 || sealEliteAlreadyDead {
			// Seal elite found (alive or dead), stop detection loop
			break
		}

		// Reset time
		if d.ctx.Data.PlayerUnit.Area.IsTown() {
			startTime = time.Now()
		}

		utils.PingSleep(utils.Light, 250)
	}

	// If seal elite was already dead, no need to kill it
	if sealEliteAlreadyDead {
		d.ctx.Logger.Debug(fmt.Sprintf("%s is already dead, continuing", sealEliteName))
		return nil
	}

	// If we didn't find the boss at all after timeout, it might have spawned far away or died before we could detect it
	// For Lord De Seis this is acceptable (he can be far), but for others it's suspicious
	if sealElite.UnitID == 0 {
		// Try one more time to check corpses after clearing nearby area
		d.ctx.Logger.Debug(fmt.Sprintf("%s not found, attempting to clear before checking again", sealEliteName))
		action.ClearAreaAroundPlayer(40, data.MonsterAnyFilter())
		d.ctx.RefreshGameData()

		for _, corpse := range d.ctx.Data.Corpses {
			if action.IsMonsterSealElite(corpse) && corpse.Name == sealEliteID {
				d.ctx.Logger.Debug(fmt.Sprintf("%s is now dead, continuing", sealEliteName))
				return nil
			}
		}

		// If it's Lord De Seis, this is acceptable (he spawns far sometimes)
		if sealEliteName == "Lord De Seis" {
			d.ctx.Logger.Debug("Lord De Seis not found but this is acceptable, continuing")
			return nil
		}

		return fmt.Errorf("no seal elite found for %s within %v", sealEliteName, timeout)
	}

	utils.PingSleep(utils.Medium, 500)

	killSealEliteAttempts := 0
	if sealElite.UnitID != 0 {
		for killSealEliteAttempts <= 5 {
			d.ctx.PauseIfNotPriority()
			d.ctx.RefreshGameData()
			m, found := d.ctx.Data.Monsters.FindByID(sealElite.UnitID)

			// If in town, wait until back to battlefield
			if d.ctx.Data.PlayerUnit.Area.IsTown() {
				utils.PingSleep(utils.Light, 100)
				continue
			}

			if !found {
				// Boss UnitID lost, try to re-detect by checking all seal elites
				for _, monster := range d.ctx.Data.Monsters.Enemies(d.ctx.Data.MonsterFilterAnyReachable()) {
					if action.IsMonsterSealElite(monster) && monster.Name == sealEliteID {
						sealElite = monster
						found = true
						break
					}
				}

				if !found {
					// Check corpses - look for the specific boss by name
					for _, corpse := range d.ctx.Data.Corpses {
						if action.IsMonsterSealElite(corpse) && corpse.Name == sealEliteID {
							d.ctx.Logger.Debug(fmt.Sprintf("Successfully killed seal elite %s (found in corpses)", sealEliteName))
							return nil
						}
					}

					// Still not found - only fail after multiple attempts (not first iteration)
					if killSealEliteAttempts > 2 {
						return fmt.Errorf("seal elite %s not found after first detection", sealEliteName)
					}

					// Continue loop to retry
					utils.PingSleep(utils.Light, 250)
					continue
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
			d.ctx.Logger.Debug(fmt.Sprintf("Clearing area around seal elite %s with radius %d (attempts #%d)", sealEliteName, clearRadius, killSealEliteAttempts))

			err := action.ClearAreaAroundPosition(sealElite.Position, clearRadius, func(monsters data.Monsters) (filteredMonsters []data.Monster) {
				if isLevelingChar {
					filteredMonsters = append(filteredMonsters, monsters...)
				} else {
					filteredMonsters = append(filteredMonsters, sealElite)
				}
				return filteredMonsters
			})

			if err != nil {
				d.ctx.Logger.Error(fmt.Sprintf("Failed to clear area around seal elite %s: %v", sealEliteName, err))
				continue
			}
			d.ctx.Logger.Debug(fmt.Sprintf("Successfully cleared area around seal elite %s (attempts #%d)", sealEliteName, killSealEliteAttempts))

			// After clearing, check if the seal elite was killed
			d.ctx.RefreshGameData()

			// First check corpses (if not shattered)
			for _, corpse := range d.ctx.Data.Corpses {
				if action.IsMonsterSealElite(corpse) && corpse.Name == sealEliteID {
					d.ctx.Logger.Debug(fmt.Sprintf("Successfully killed seal elite %s after %d attempts (found in corpses)", sealEliteName, killSealEliteAttempts))
					return nil
				}
			}

			// If corpse not found, check if seal elite is still alive
			sealEliteStillAlive := false
			for _, m := range d.ctx.Data.Monsters.Enemies(d.ctx.Data.MonsterFilterAnyReachable()) {
				if action.IsMonsterSealElite(m) && m.Name == sealEliteID {
					d.ctx.Logger.Debug(fmt.Sprintf("Seal elite %s is still alive (attempts #%d)", sealEliteName, killSealEliteAttempts))
					sealEliteStillAlive = true
					break
				}
			}

			// If we detected the seal elite earlier but now it's gone (not alive, not in corpses)
			// Trust the detection flag - seal elite was killed, corpse likely destroyed/shattered
			if sealEliteDetected && !sealEliteStillAlive {
				d.ctx.Logger.Debug(fmt.Sprintf("Successfully killed seal elite %s after %d attempts (corpse destroyed/shattered)", sealEliteName, killSealEliteAttempts))
				return nil
			}

			utils.PingSleep(utils.Light, 250)
		}
	} else {
		return fmt.Errorf("no seal elite found for %s within %v", sealEliteName, timeout)
	}

	return fmt.Errorf("failed to kill seal elite %s after %d attempts", sealEliteName, killSealEliteAttempts)
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

	d.ctx.RefreshGameData()
	if d.ctx.Data.PlayerUnit.Area.Act() != 5 {
		harrogathPortal, found := d.ctx.Data.Objects.FindOne(object.LastLastPortal)
		if found {
			if err = action.InteractObject(harrogathPortal, func() bool {
				utils.Sleep(100)
				ctx := context.Get()
				isInCinematic := ctx.Data.OpenMenus.Cinematic
				isInAct5 := ctx.Data.PlayerUnit.Area.Act() == 5
				return isInCinematic || isInAct5
			}); err != nil {
				return err
			}
		} else {
			return errors.New("failed to find Harrogath portal")
		}

	}

	timeout := time.Second * 20
	startTime := time.Now()
	for time.Since(startTime) < timeout {
		d.ctx.RefreshGameData()

		if d.ctx.Data.OpenMenus.Cinematic {
			d.skipCinematic()
			continue
		}

		break
	}

	utils.Sleep(2000)

	// final check
	d.ctx.RefreshGameData()
	if d.ctx.Data.OpenMenus.Cinematic {
		return errors.New("failed to skip act 5 cinematic")
	}

	return nil
}

func (d Diablo) skipCinematic() {
	utils.Sleep(2000)
	action.HoldKey(win.VK_SPACE, 2000)
	utils.Sleep(2000)
	action.HoldKey(win.VK_SPACE, 2000)
	utils.Sleep(2000)
}
