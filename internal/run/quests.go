package run

import (
	"errors" // NEW: Import errors package
	"fmt"    // NEW: Import fmt for error formatting
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

type Quests struct {
	ctx *context.Status
}

func NewQuests() *Quests {
	return &Quests{
		ctx: context.Get(),
	}
}

func (a Quests) Name() string {
	return string(config.QuestsRun)
}

func (a Quests) CheckConditions(parameters *RunParameters) SequencerResult {
	return SequencerError
}

func (a Quests) Run(parameters *RunParameters) error {
	if a.ctx.CharacterCfg.Game.Quests.ClearDen && !a.ctx.Data.Quests[quest.Act1DenOfEvil].Completed() {
		a.clearDenQuest()
	}

	if a.ctx.CharacterCfg.Game.Quests.RescueCain && !a.ctx.Data.Quests[quest.Act1TheSearchForCain].Completed() {
		a.rescueCainQuest()
	}

	if a.ctx.CharacterCfg.Game.Quests.RetrieveHammer && !a.ctx.Data.Quests[quest.Act1ToolsOfTheTrade].Completed() {
		a.retrieveHammerQuest()
	}

	if a.ctx.CharacterCfg.Game.Quests.KillRadament && !a.ctx.Data.Quests[quest.Act2RadamentsLair].Completed() {
		a.killRadamentQuest()
	}

	if a.ctx.CharacterCfg.Game.Quests.GetCube {
		_, found := a.ctx.Data.Inventory.Find("HoradricCube", item.LocationInventory, item.LocationStash)
		if !found {
			a.getHoradricCube()
		}
	}

	if a.ctx.CharacterCfg.Game.Quests.RetrieveBook && !a.ctx.Data.Quests[quest.Act3LamEsensTome].Completed() {
		a.retrieveBookQuest()
	}

	if a.ctx.CharacterCfg.Game.Quests.KillIzual && !a.ctx.Data.Quests[quest.Act4TheFallenAngel].Completed() {
		a.killIzualQuest()
	}

	if a.ctx.CharacterCfg.Game.Quests.KillShenk && !a.ctx.Data.Quests[quest.Act5SiegeOnHarrogath].Completed() {
		a.killShenkQuest()
	}

	if a.ctx.CharacterCfg.Game.Quests.RescueAnya && !a.ctx.Data.Quests[quest.Act5PrisonOfIce].Completed() {
		a.rescueAnyaQuest()
	}
	if a.ctx.CharacterCfg.Game.Quests.KillAncients && !a.ctx.Data.Quests[quest.Act5RiteOfPassage].Completed() {
		a.killAncientsQuest()
	}

	return nil
}

func (a Quests) clearDenQuest() error {
	a.ctx.Logger.Info("Starting Den of Evil Quest...")

	err := action.MoveToArea(area.BloodMoor)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.DenOfEvil)
	if err != nil {
		return err
	}

	a.ctx.CharacterCfg.Character.ClearPathDist = 20
	if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
		a.ctx.Logger.Error("Failed to save character configuration: %s", err.Error())
	}

	if err := action.ClearCurrentLevel(false, data.MonsterAnyFilter()); err != nil {
		return err
	}

	_, isLevelingChar := a.ctx.Char.(context.LevelingCharacter)
	if !isLevelingChar {

		err = action.ReturnTown()
		if err != nil {
			return err
		}

		err = action.InteractNPC(npc.Akara)
		if err != nil {
			return err
		}

		step.CloseAllMenus()

	}

	return nil
}

func (a Quests) rescueCainQuest() error {
	a.ctx.Logger.Info("Starting Rescue Cain Quest...")

	// --- Navigation to the Dark Wood and a safe zone near the Inifuss Tree ---
	err := action.WayPoint(area.RogueEncampment)
	if err != nil {
		return err
	}

	a.ctx.CharacterCfg.Character.ClearPathDist = 20
	if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
		a.ctx.Logger.Error("Failed to save character configuration: %s", err.Error())
	}

	err = action.WayPoint(area.DarkWood)
	if err != nil {
		return err
	}

	a.ctx.CharacterCfg.Character.ClearPathDist = 30
	if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
		a.ctx.Logger.Error("Failed to save character configuration: %s", err.Error())
	}

	// Find the Inifuss Tree position.
	var inifussTreePos data.Position
	var foundTree bool
	for _, o := range a.ctx.Data.Objects {
		if o.Name == object.InifussTree {
			inifussTreePos = o.Position
			foundTree = true
			break
		}
	}
	if !foundTree {
		a.ctx.Logger.Error("InifussTree not found, aborting quest.")
		return errors.New("InifussTree not found")
	}

	// Get the player's current position.
	playerPos := a.ctx.Data.PlayerUnit.Position

	// --- New segmented approach to clear the path to the Inifuss Tree ---
	// Start 55 units away and move closer in 10-unit increments.

	clearRadius := 20
	for distance := 55; distance > 0; distance -= 5 {
		a.ctx.Logger.Info(fmt.Sprintf("Moving to position %d units away from the Inifuss Tree to clear the area.", distance))

		// Calculate the new position based on the current distance.
		safePos := atDistance(inifussTreePos, playerPos, distance)

		// Move to the calculated position.
		err = action.MoveToCoords(safePos)
		if err != nil {
			return err
		}

		// Clear a large area around the new position.
		a.ctx.Logger.Info(fmt.Sprintf("Clearing a %d unit radius around the current position...", clearRadius))
		if err := action.ClearAreaAroundPlayer(clearRadius, data.MonsterAnyFilter()); err != nil {
			return err
		}
	}

	// --- End of new segmented approach ---

	err = action.MoveToCoords(inifussTreePos)
	if err != nil {
		return err
	}

	obj, found := a.ctx.Data.Objects.FindOne(object.InifussTree)
	if !found {
		a.ctx.Logger.Error("InifussTree not found, aborting quest.")
		return errors.New("InifussTree not found")
	}

	err = action.InteractObject(obj, func() bool {
		updatedObj, found := a.ctx.Data.Objects.FindOne(object.InifussTree)
		return found && !updatedObj.Selectable
	})
	if err != nil {
		return fmt.Errorf("error interacting with Inifuss Tree: %w", err)
	}

	scrollInifussUnitID := 524
	scrollInifussAfterAkara := 525
	scrollInifussName := "Scroll of Inifuss"

PickupLoop:
	for i := 0; i < 5; i++ {
		a.ctx.RefreshGameData()

		foundInInv := false
		for _, itm := range a.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
			if itm.ID == scrollInifussUnitID || itm.ID == scrollInifussAfterAkara {
				foundInInv = true
				break
			}
		}

		if foundInInv {
			a.ctx.Logger.Info(fmt.Sprintf("%s found in inventory. Proceeding with quest.", scrollInifussName))
			break PickupLoop
		}

		// Find the scroll on the ground.
		var scrollObj data.Item
		foundOnGround := false
		for _, itm := range a.ctx.Data.Inventory.ByLocation(item.LocationGround) {
			if itm.ID == scrollInifussUnitID {
				scrollObj = itm
				foundOnGround = true
				break
			}
		}

		if foundOnGround {
			a.ctx.Logger.Info(fmt.Sprintf("%s found on the ground at position %v. Attempting pickup (Attempt %d)...", scrollInifussName, scrollObj.Position, i+1))

			playerPos := a.ctx.Data.PlayerUnit.Position
			safeAwayPos := atDistance(scrollObj.Position, playerPos, -5)

			pickupAttempts := 0
			for pickupAttempts < 8 {
				a.ctx.Logger.Debug("Moving away from scroll for a brief moment...")
				moveAwayErr := action.MoveToCoords(safeAwayPos)
				if moveAwayErr != nil {
					a.ctx.Logger.Warn(fmt.Sprintf("Failed to move away from scroll: %v", moveAwayErr))
				}
				utils.Sleep(200)

				moveErr := action.MoveToCoords(scrollObj.Position)
				if moveErr != nil {
					a.ctx.Logger.Error(fmt.Sprintf("Failed to move to scroll position: %v", moveErr))
					utils.Sleep(500)
					pickupAttempts++
					continue
				}

				// --- Refresh game data just before pickup attempt ---
				a.ctx.RefreshGameData()

				pickupErr := action.ItemPickup(10)
				if pickupErr != nil {
					a.ctx.Logger.Warn(fmt.Sprintf("Pickup attempt %d failed: %v", pickupAttempts+1, pickupErr))
					utils.Sleep(500)
					pickupAttempts++
					continue
				}

				a.ctx.RefreshGameData()
				foundInInvAfterPickup := false
				for _, itm := range a.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
					if itm.ID == scrollInifussUnitID {
						foundInInvAfterPickup = true
						break
					}
				}
				if foundInInvAfterPickup {
					a.ctx.Logger.Info(fmt.Sprintf("Pickup confirmed for %s after %d attempts. Proceeding.", scrollInifussName, pickupAttempts+1))
					break PickupLoop
				}
				pickupAttempts++
			}
		} else {
			a.ctx.Logger.Debug(fmt.Sprintf("%s not found on the ground on attempt %d. Retrying.", scrollInifussName, i+1))
			utils.Sleep(1000)
		}
	}

	foundInInv := false
	for _, itm := range a.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if itm.ID == scrollInifussUnitID {
			foundInInv = true
			break
		}
	}
	if !foundInInv {
		a.ctx.Logger.Error(fmt.Sprintf("Failed to pick up %s after all attempts. Aborting current run.", scrollInifussName))
		return errors.New("failed to pick up Scroll of Inifuss")
	}

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	err = action.InteractNPC(npc.Akara)
	if err != nil {
		return err
	}

	step.CloseAllMenus()

	err = NewTristram().Run(nil)
	if err != nil {
		return err
	}

	action.ReturnTown()

	return nil
}

func (a Quests) retrieveHammerQuest() error {
	a.ctx.Logger.Info("Starting Retrieve Hammer Quest...")

	err := action.WayPoint(area.RogueEncampment)
	if err != nil {
		return err
	}

	err = action.WayPoint(area.OuterCloister)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.Barracks)
	if err != nil {
		return err
	}

	err = action.MoveTo(func() (data.Position, bool) {
		for _, o := range a.ctx.Data.Objects {
			if o.Name == object.Malus {
				return o.Position, true
			}
		}
		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(20, data.MonsterAnyFilter())

	malus, found := a.ctx.Data.Objects.FindOne(object.Malus)
	if !found {
		a.ctx.Logger.Debug("Malus not found")
	}

	err = action.InteractObject(malus, nil)
	if err != nil {
		return err
	}

	action.ItemPickup(0)

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	err = action.InteractNPC(npc.Charsi)
	if err != nil {
		return err
	}

	step.CloseAllMenus()

	return nil
}

func (a Quests) killRadamentQuest() error {

	if itm, found := a.ctx.Data.Inventory.Find("BookofSkill"); found {
		a.ctx.Logger.Info("BookofSkill found in inventory. Using it...")

		// Use the book of skill
		step.CloseAllMenus()
		a.ctx.HID.PressKeyBinding(a.ctx.Data.KeyBindings.Inventory)
		screenPos := ui.GetScreenCoordsForItem(itm)
		utils.Sleep(200)
		a.ctx.HID.Click(game.RightButton, screenPos.X, screenPos.Y)
		step.CloseAllMenus()

		a.ctx.Logger.Info("Book of Skill used successfully.")

	}

	var startingPositionAtma = data.Position{
		X: 5138,
		Y: 5057,
	}

	a.ctx.Logger.Info("Starting Kill Radament Quest...")

	err := action.WayPoint(area.LutGholein)
	if err != nil {
		return err
	}

	err = action.WayPoint(area.SewersLevel2Act2)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.SewersLevel3Act2)
	if err != nil {
		return err
	}
	action.Buff()

	// cant find npc.Radament for some reason, using the sparkly chest with ID 355 next him to find him
	err = action.MoveTo(func() (data.Position, bool) {
		for _, o := range a.ctx.Data.Objects {
			if o.Name == object.Name(355) {
				return o.Position, true
			}
		}

		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(30, data.MonsterAnyFilter())

	if !a.ctx.Data.Quests[quest.Act2RadamentsLair].Completed() {

		// Sometimes it moves too far away from the book to pick it up, making sure it moves back to the chest
		err = action.MoveTo(func() (data.Position, bool) {
			for _, o := range a.ctx.Data.Objects {
				if o.Name == object.Name(355) {
					return o.Position, true
				}
			}

			return data.Position{}, false
		})
		if err != nil {
			return err
		}

		// If its still too far away, we're making sure it detects it
		action.ItemPickup(50)

		err = action.ReturnTown()
		if err != nil {
			return err
		}

		err = action.MoveToCoords(startingPositionAtma)
		if err != nil {
			return err
		}

		err = action.InteractNPC(npc.Atma)
		if err != nil {
			return err
		}

		step.CloseAllMenus()
		a.ctx.HID.PressKeyBinding(a.ctx.Data.KeyBindings.Inventory)
		itm, _ := a.ctx.Data.Inventory.Find("BookofSkill")
		screenPos := ui.GetScreenCoordsForItem(itm)
		utils.Sleep(200)
		a.ctx.HID.Click(game.RightButton, screenPos.X, screenPos.Y)
		step.CloseAllMenus()

	}

	return nil
}

func (a Quests) getHoradricCube() error {
	a.ctx.Logger.Info("Starting Retrieve the Cube Quest...")

	err := action.WayPoint(area.LutGholein)
	if err != nil {
		return err
	}

	err = action.WayPoint(area.HallsOfTheDeadLevel2)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.HallsOfTheDeadLevel3)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveTo(func() (data.Position, bool) {
		chest, found := a.ctx.Data.Objects.FindOne(object.HoradricCubeChest)
		if found {
			a.ctx.Logger.Info("Horadric Cube chest found, moving to that room")
			return chest.Position, true
		}
		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())

	obj, found := a.ctx.Data.Objects.FindOne(object.HoradricCubeChest)
	if !found {
		return err
	}

	err = action.InteractObject(obj, func() bool {
		updatedObj, found := a.ctx.Data.Objects.FindOne(object.HoradricCubeChest)
		if found {
			if !updatedObj.Selectable {
				a.ctx.Logger.Debug("Interacted with Horadric Cube Chest")
			}
			return !updatedObj.Selectable
		}
		return false
	})
	if err != nil {
		return err
	}

	// Making sure we pick up the cube
	action.ItemPickup(10)

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	return nil
}

func (a Quests) retrieveBookQuest() error {
	a.ctx.Logger.Info("Starting Retrieve Book Quest...")

	err := action.WayPoint(area.KurastDocks)
	if err != nil {
		return err
	}

	err = action.WayPoint(area.KurastBazaar)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.RuinedTemple)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveTo(func() (data.Position, bool) {
		for _, o := range a.ctx.Data.Objects {
			if o.Name == object.LamEsensTome {
				return o.Position, true
			}
		}

		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(30, data.MonsterAnyFilter())

	tome, found := a.ctx.Data.Objects.FindOne(object.LamEsensTome)
	if !found {
		return err
	}

	err = action.InteractObject(tome, nil)
	if err != nil {
		return err
	}

	// Making sure we pick up the tome
	action.ItemPickup(10)

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	err = action.InteractNPC(npc.Alkor)
	if err != nil {
		return err
	}

	step.CloseAllMenus()

	return nil
}

func (a Quests) killIzualQuest() error {
	a.ctx.Logger.Info("Starting Kill Izual Quest...")

	err := action.MoveToArea(area.OuterSteppes)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.PlainsOfDespair)
	if err != nil {
		return err
	}
	action.Buff()

	// special barb leveling handling
	if preparer, ok := a.ctx.Char.(bossEquipmentPreparer); ok {
		preparer.PrepareBossEquipment(npc.Izual)
	}

	// Start a timer to ensure we find Izual within 5 minutes
	startTime := time.Now()
	timeout := time.Minute * 10
	a.ctx.Logger.Info("Searching for Izual...")

	// Check if the timeout has been exceeded on each loop
	if time.Since(startTime) > timeout {
		return fmt.Errorf("timeout: failed to find Izual within %v", timeout)
	}

	// Once Izual is found, move to him
	err = action.MoveTo(func() (data.Position, bool) {
		areaData := a.ctx.Data.Areas[area.PlainsOfDespair]
		izualNPC, found := areaData.NPCs.FindOne(npc.Izual)
		if !found {
			return data.Position{}, false
		}

		return izualNPC.Positions[0], true
	})
	if err != nil {
		return err
	}

	// Engage and kill Izual
	err = a.ctx.Char.KillIzual()
	if err != nil {
		return err
	}

	action.ItemPickup(30)

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	err = action.InteractNPC(npc.Tyrael2)
	if err != nil {
		return err
	}

	time.Sleep(500 * time.Millisecond)

	return nil
}

func (a Quests) killShenkQuest() error {
	var shenkPosition = data.Position{
		X: 3895,
		Y: 5120,
	}

	a.ctx.Logger.Info("Starting Kill Shenk Quest...")

	err := action.WayPoint(area.Harrogath)
	if err != nil {
		return err
	}

	err = action.WayPoint(area.FrigidHighlands)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.BloodyFoothills)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToCoords(shenkPosition)
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(25, data.MonsterAnyFilter())
	action.ItemPickup(30)

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	err = action.InteractNPC(npc.Larzuk)
	if err != nil {
		return err
	}

	step.CloseAllMenus()

	return nil
}

func (a Quests) rescueAnyaQuest() error {
	a.ctx.Logger.Info("Starting Rescuing Anya Quest...")

	err := action.WayPoint(area.CrystallinePassage)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.FrozenRiver)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveTo(func() (data.Position, bool) {
		anya, found := a.ctx.Data.NPCs.FindOne(793)
		return anya.Positions[0], found
	})
	if err != nil {
		return err
	}

	err = action.MoveTo(func() (data.Position, bool) {
		anya, found := a.ctx.Data.Objects.FindOne(object.FrozenAnya)
		return anya.Position, found
	})
	if err != nil {
		return err
	}

	//action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())

	anya, found := a.ctx.Data.Objects.FindOne(object.FrozenAnya)
	if !found {
		a.ctx.Logger.Debug("Frozen Anya not found")
	}

	err = action.InteractObject(anya, nil)
	if err != nil {
		return err
	}

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	action.IdentifyAll(false)
	action.Stash(false)
	action.ReviveMerc()
	action.Repair()
	action.VendorRefill(action.VendorRefillOpts{SellJunk: true, BuyConsumables: true})

	err = action.InteractNPC(npc.Malah)
	if err != nil {
		return err
	}

	err = action.UsePortalInTown()
	if err != nil {
		return err
	}

	err = action.InteractObject(anya, nil)
	if err != nil {
		return err
	}

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	time.Sleep(8000 * time.Millisecond)

	err = action.InteractNPC(npc.Malah)
	if err != nil {
		return err
	}

	step.CloseAllMenus()
	a.ctx.HID.PressKeyBinding(a.ctx.Data.KeyBindings.Inventory)
	itm, _ := a.ctx.Data.Inventory.Find("ScrollOfResistance")
	screenPos := ui.GetScreenCoordsForItem(itm)
	utils.Sleep(200)
	a.ctx.HID.Click(game.RightButton, screenPos.X, screenPos.Y)
	step.CloseAllMenus()

	return nil
}

func (a Quests) killAncientsQuest() error {
	// Store the original configuration
	originalBackToTownCfg := a.ctx.CharacterCfg.BackToTown
	originalTownChicken := a.ctx.CharacterCfg.Health.TownChickenAt

	// Defer the restoration of the configuration.
	// This will run when the function exits, regardless of how.
	defer func() {
		a.ctx.CharacterCfg.BackToTown = originalBackToTownCfg
		a.ctx.CharacterCfg.Health.TownChickenAt = originalTownChicken
		a.ctx.Logger.Info("Restored original back-to-town checks after Ancients fight.")
	}()

	err := action.WayPoint(area.Harrogath)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.WayPoint(area.TheAncientsWay)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.ArreatSummit)
	if err != nil {
		return err
	}
	action.Buff()

	action.ReturnTown()
	action.InRunReturnTownRoutine()
	action.UsePortalInTown()
	action.Buff()

	// Find and interact with the altar object
	altar, found := a.ctx.Data.Objects.FindOne(object.AncientsAltar)
	if !found {
		return fmt.Errorf("AncientsAltar not found")
	}

	err = action.InteractObject(altar, func() bool {
		// After clicking, press Enter to confirm the dialog
		a.ctx.HID.PressKey(win.VK_RETURN)
		utils.Sleep(2000)

		// Check if Ancients spawned (elite monsters appeared)
		ancients := a.ctx.Data.Monsters.Enemies(data.MonsterEliteFilter())
		return len(ancients) > 0
	})
	if err != nil {
		return err
	}

	// Modify the configuration for the Ancients fight
	a.ctx.CharacterCfg.BackToTown.NoHpPotions = false
	a.ctx.CharacterCfg.BackToTown.NoMpPotions = false
	a.ctx.CharacterCfg.BackToTown.EquipmentBroken = false
	a.ctx.CharacterCfg.BackToTown.MercDied = false
	a.ctx.CharacterCfg.Health.TownChickenAt = 0

	for {
		ancients := a.ctx.Data.Monsters.Enemies(data.MonsterEliteFilter())
		if len(ancients) == 0 {
			break
		}

		err = a.ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			for _, m := range d.Monsters.Enemies(data.MonsterEliteFilter()) {
				return m.UnitID, true
			}
			return 0, false
		}, nil)
	}

	action.ItemPickup(30)

	// The defer statement above will handle the restoration
	// a.ctx.CharacterCfg.BackToTown = originalBackToTownCfg // This line is now removed
	// a.ctx.Logger.Info("Restored original back-to-town checks after Ancients fight.") // This line is now part of the defer

	utils.Sleep(500)
	step.CloseAllMenus()
	action.InRunReturnTownRoutine()

	// Continue to Worldstone Keep and get the waypoint
	err = action.MoveToArea(area.TheWorldStoneKeepLevel1)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.TheWorldStoneKeepLevel2)
	if err != nil {
		return err
	}

	err = action.DiscoverWaypoint()
	if err != nil {
		return err
	}

	return nil
}
