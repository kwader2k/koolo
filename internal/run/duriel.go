package run

import (
	"errors"
	"fmt"
	"slices"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/mode"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

const (
	maxOrificeAttempts    = 10
	orificeCheckDelay     = 200
	thawingPotionsToDrink = 10 // Number of thawing potions to consume (counts separately for the character and the mercenary)
	thawingMinBuffSeconds = 100
)

var talTombs = []area.ID{area.TalRashasTomb1, area.TalRashasTomb2, area.TalRashasTomb3, area.TalRashasTomb4, area.TalRashasTomb5, area.TalRashasTomb6, area.TalRashasTomb7}

type Duriel struct {
	ctx *context.Status
}

func NewDuriel() *Duriel {
	return &Duriel{
		ctx: context.Get(),
	}
}

func (d Duriel) Name() string {
	return string(config.DurielRun)
}

func (d Duriel) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		if d.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
			return SequencerOk
		}
		return SequencerSkip
	} else if d.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		if slices.Contains(d.ctx.Data.PlayerUnit.AvailableWaypoints, area.KurastDocks) ||
			slices.Contains(d.ctx.Data.PlayerUnit.AvailableWaypoints, area.ThePandemoniumFortress) ||
			slices.Contains(d.ctx.Data.PlayerUnit.AvailableWaypoints, area.Harrogath) {
			return SequencerSkip
		}

		//Workaround AvailableWaypoints only filled when wp menu has been opened on act page
		//Check if any act 3 quests has started or is completed
		if action.HasAnyQuestStartedOrCompleted(quest.Act3TheGoldenBird, quest.Act3TheGuardian) || d.ctx.Data.PlayerUnit.Area.Act() >= 3 {
			return SequencerSkip
		}
		return SequencerOk
	}

	horadricStaffQuestCompleted := d.ctx.Data.Quests[quest.Act2TheHoradricStaff].Completed()
	summonerQuestCompleted := d.ctx.Data.Quests[quest.Act2TheSummoner].Completed()
	if horadricStaffQuestCompleted && summonerQuestCompleted {
		return SequencerOk
	}

	if _, foundHoradric := d.ctx.Data.Inventory.Find("HoradricStaff", item.LocationInventory, item.LocationStash, item.LocationEquipped, item.LocationCube); foundHoradric {
		return SequencerOk
	}

	_, foundStaff := d.ctx.Data.Inventory.Find("StaffOfKings", item.LocationInventory, item.LocationStash, item.LocationEquipped, item.LocationCube)
	_, foundAmulet := d.ctx.Data.Inventory.Find("AmuletOfTheViper", item.LocationInventory, item.LocationStash, item.LocationEquipped, item.LocationCube)
	if foundStaff && foundAmulet {
		return SequencerOk
	}

	return SequencerStop
}

func (d Duriel) Run(parameters *RunParameters) error {

	if IsQuestRun(parameters) {
		err := action.WayPoint(area.LutGholein)
		if err != nil {
			return err
		}
		//Try completing quest and early exit if possible
		d.tryTalkToJerhyn()
		if d.tryTalkToMeshif() {
			return nil
		}

		//prepare for quest
		if err := d.prepareStaff(); err != nil {
			return err
		}
	}

	_, isLevelingChar := d.ctx.Char.(context.LevelingCharacter)
	useThawing := d.ctx.CharacterCfg.Game.Duriel.UseThawing || isLevelingChar
	// Track thawing buff duration based on player consumption (30 seconds per potion).
	var thawingBuffDeadline time.Time
	thawingBuffRemaining := func() time.Duration {
		if thawingBuffDeadline.IsZero() {
			return 0
		}
		return time.Until(thawingBuffDeadline)
	}
	updateThawingBuff := func(selfCount int) {
		if selfCount <= 0 {
			return
		}
		now := time.Now()
		if thawingBuffDeadline.IsZero() || thawingBuffDeadline.Before(now) {
			thawingBuffDeadline = now
		}
		thawingBuffDeadline = thawingBuffDeadline.Add(time.Duration(selfCount*30) * time.Second)
	}
	// Refresh thawing potions in town if the remaining buff is too low (or missing).
	ensureThawingBuff := func(minSeconds int) error {
		if !useThawing {
			return nil
		}
		if minSeconds == 0 {
			if !thawingBuffDeadline.IsZero() && thawingBuffRemaining() > 0 {
				return nil
			}
		} else if thawingBuffRemaining() >= time.Duration(minSeconds)*time.Second {
			return nil
		}
		if err := action.ReturnTown(); err != nil {
			return err
		}
		if err := action.WayPoint(area.LutGholein); err != nil {
			return err
		}
		mercAlive := d.ctx.Data.MercHPPercent() > 0
		if err := d.buyAndDrinkThawingPotions(mercAlive, updateThawingBuff); err != nil {
			return err
		}
		return action.UsePortalInTown()
	}

	// Get thawing potions at the start of the run (Lysander in Lut Gholein).
	if useThawing {
		if err := action.WayPoint(area.LutGholein); err != nil {
			return err
		}

		mercAlive := d.ctx.Data.MercHPPercent() > 0
		if err := d.buyAndDrinkThawingPotions(mercAlive, updateThawingBuff); err != nil {
			return err
		}
	}

	err := action.WayPoint(area.CanyonOfTheMagi)
	if err != nil {
		return err
	}

	// Find and move to the real Tal Rasha tomb.
	realTalRashaTomb, err := d.findRealTomb()
	if err != nil {
		return err
	}

	// Leveling characters skip fighting in the canyon to speed up the run.
	if isLevelingChar && d.ctx.Data.PlayerUnit.Area == area.CanyonOfTheMagi {
		originalClearPathDist := d.ctx.CharacterCfg.Character.ClearPathDist
		d.ctx.CharacterCfg.Character.ClearPathDist = 0
		err = action.MoveToArea(realTalRashaTomb)
		d.ctx.CharacterCfg.Character.ClearPathDist = originalClearPathDist
	} else {
		err = action.MoveToArea(realTalRashaTomb)
	}
	if err != nil {
		return err
	}

	// Wait for area to fully load and get synchronized.
	utils.Sleep(500)
	d.ctx.RefreshGameData()

	// Find orifice with retry logic.
	var orifice data.Object
	var found bool

	for attempts := 0; attempts < maxOrificeAttempts; attempts++ {
		orifice, found = d.ctx.Data.Objects.FindOne(object.HoradricOrifice)
		if found && orifice.Mode == mode.ObjectModeOpened {
			break
		}
		utils.Sleep(orificeCheckDelay)
		d.ctx.RefreshGameData()
	}

	if !found {
		return errors.New("failed to find Duriel's Lair entrance after multiple attempts")
	}

	// Move to orifice and clear the area.
	moveOptions := []step.MoveOption{}
	if d.ctx.Data.CanTeleport() {
		moveOptions = append(moveOptions, step.WithIgnoreMonsters())
	}
	err = action.MoveToCoords(orifice.Position, moveOptions...)
	if err != nil {
		return err
	}

	staff, ok := d.ctx.Data.Inventory.Find("HoradricStaff", item.LocationInventory)
	if !d.ctx.Data.Quests[quest.Act2TheHoradricStaff].Completed() && ok {

		err = action.ClearAreaAroundPlayer(10, data.MonsterAnyFilter())
		if err != nil {
			return err
		}

		action.InteractObject(orifice, func() bool {
			return d.ctx.Data.OpenMenus.Anvil
		})

		screenPos := ui.GetScreenCoordsForItem(staff)

		d.ctx.HID.Click(game.LeftButton, screenPos.X, screenPos.Y)
		utils.Sleep(300)
		if d.ctx.Data.LegacyGraphics {
			d.ctx.HID.Click(game.LeftButton, ui.AnvilCenterXClassic, ui.AnvilCenterYClassic)
			utils.Sleep(500)
			d.ctx.HID.Click(game.LeftButton, ui.AnvilBtnXClassic, ui.AnvilBtnYClassic)
		} else {
			d.ctx.HID.Click(game.LeftButton, ui.AnvilCenterX, ui.AnvilCenterY)
			utils.Sleep(500)
			d.ctx.HID.Click(game.LeftButton, ui.AnvilBtnX, ui.AnvilBtnY)
		}
		// Leveling characters wait actively for the portal animation while staying safe.
		if isLevelingChar {
			d.ctx.Logger.Info("Waiting for Duriel's lair to open, securing area.")
			nextClear := time.Now()
			deadline := time.Now().Add(20 * time.Second)
			portalOpened := false
			for time.Now().Before(deadline) {
				d.ctx.RefreshGameData()
				if _, found := d.ctx.Data.Objects.FindOne(object.DurielsLairPortal); found {
					portalOpened = true
					break
				}
				if time.Now().After(nextClear) {
					if err := action.ClearAreaAroundPlayer(12, data.MonsterAnyFilter()); err != nil {
						return err
					}
					nextClear = time.Now().Add(2 * time.Second)
				}
				utils.Sleep(300)
			}
			if !portalOpened {
				d.ctx.Logger.Warn("Duriel's lair did not open after waiting", "timeoutSeconds", 20)
			}
		} else {
			utils.Sleep(20000)
		}
	}

	// In Normal/Nightmare check the remaining time of thawing potions buff after the town routine.
	if isLevelingChar && d.ctx.CharacterCfg.Game.Difficulty != difficulty.Hell {
		action.ClearAreaAroundPlayer(20, data.MonsterAnyFilter())
		if err := action.InRunReturnTownRoutine(); err != nil {
			return err
		}
		if useThawing && thawingBuffRemaining() < time.Duration(thawingMinBuffSeconds)*time.Second {
			d.ctx.Logger.Info("Refreshing thawing potions after town routine", "remainingSeconds", int(thawingBuffRemaining().Seconds()))
			if err := ensureThawingBuff(thawingMinBuffSeconds); err != nil {
				return err
			}
		}
	}

	err = action.MoveToCoords(orifice.Position)
	if err != nil {
		return err
	}

	d.ctx.RefreshGameData()
	utils.Sleep(200)

	// Ensure we enter the lair only with an active thawing buff.
	if err := ensureThawingBuff(0); err != nil {
		return err
	}

	duriellair, found := d.ctx.Data.Objects.FindOne(object.DurielsLairPortal)
	if found {
		// Now enter Duriel's lair with thawing potions active
		action.InteractObject(duriellair, func() bool {
			return d.ctx.Data.PlayerUnit.Area == area.DurielsLair && d.ctx.Data.AreaData.IsInside(d.ctx.Data.PlayerUnit.Position)
		})
	}
	d.ctx.Logger.Debug(fmt.Sprintf("Quest Status %v", d.ctx.Data.Quests[quest.Act2TheSevenTombs]))

	d.ctx.Logger.Info("Killing Duriel")
	// Final refresh before fight
	d.ctx.RefreshGameData()

	utils.Sleep(700)

	if err := d.ctx.Char.KillDuriel(); err != nil {
		return err
	}

	action.ItemPickup(30)

	if IsQuestRun(parameters) {
		action.ClearAreaAroundPlayer(30, d.durielFilter())

		duriel, found := d.ctx.Data.Monsters.FindOne(npc.Duriel, data.MonsterTypeUnique)
		if !found || duriel.Stats[stat.Life] <= 0 || d.ctx.Data.Quests[quest.Act2TheSevenTombs].HasStatus(quest.StatusInProgress3) {
			action.MoveToCoords(data.Position{
				X: 22577,
				Y: 15600,
			})
			action.InteractNPC(npc.Tyrael)
		}

		action.ReturnTown()

		if !d.tryTalkToJerhyn() {
			return errors.New("failed to talk to jerhyn")
		}

		if !d.tryTalkToMeshif() {
			return errors.New("failed to talk to meshif")
		}
	}

	return nil
}

// Consume thawing potions from the inventory only, optionally feeding the mercenary.
func (d Duriel) drinkThawingPotions(selfTarget, mercTarget int) (int, int) {
	mercAlive := d.ctx.Data.MercHPPercent() > 0
	if selfTarget < 0 {
		selfTarget = 0
	}
	if mercTarget < 0 {
		mercTarget = 0
	}
	if !mercAlive {
		mercTarget = 0
	}
	if selfTarget == 0 && mercTarget == 0 {
		return 0, 0
	}
	shouldGiveMerc := mercAlive && mercTarget > 0
	reHidePortraits := false
	if shouldGiveMerc && d.ctx.CharacterCfg.HidePortraits && !d.ctx.Data.OpenMenus.PortraitsShown {
		d.ctx.CharacterCfg.HidePortraits = false
		reHidePortraits = true
		d.ctx.HID.PressKey(d.ctx.Data.KeyBindings.ShowPortraits.Key1[0])
		utils.Sleep(200)
	}

	d.ctx.HID.PressKeyBinding(d.ctx.Data.KeyBindings.Inventory)
	utils.Sleep(300)

	selfCount := 0
	mercCount := 0

	for _, itm := range d.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if itm.Name != "ThawingPotion" {
			continue
		}
		pos := ui.GetScreenCoordsForItem(itm)
		utils.Sleep(500)

		if selfCount < selfTarget {
			d.ctx.HID.Click(game.RightButton, pos.X, pos.Y)
			selfCount++
			continue
		}

		if mercCount < mercTarget {
			d.ctx.HID.Click(game.LeftButton, pos.X, pos.Y)
			utils.Sleep(300)
			if d.ctx.Data.LegacyGraphics {
				d.ctx.HID.Click(game.LeftButton, ui.MercAvatarPositionXClassic, ui.MercAvatarPositionYClassic)
			} else {
				d.ctx.HID.Click(game.LeftButton, ui.MercAvatarPositionX, ui.MercAvatarPositionY)
			}
			mercCount++
			continue
		}

		d.ctx.HID.Click(game.RightButton, pos.X, pos.Y)
		selfCount++
	}
	step.CloseAllMenus()

	if reHidePortraits {
		d.ctx.CharacterCfg.HidePortraits = true
		_ = action.HidePortraits()
	}

	return selfCount, mercCount
}

func (d Duriel) findRealTomb() (area.ID, error) {
	var realTomb area.ID

	for _, tomb := range talTombs {
		for _, obj := range d.ctx.Data.Areas[tomb].Objects {
			if obj.Name == object.HoradricOrifice {
				realTomb = tomb
				break
			}
		}
	}

	if realTomb == 0 {
		return 0, errors.New("failed to find the real Tal Rasha tomb")
	}

	return realTomb, nil
}

// Count only free, unlocked inventory cells.
func (d Duriel) countFreeInventorySlots() int {
	occupied := [4][10]bool{}
	for _, i := range d.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		w := i.Desc().InventoryWidth
		h := i.Desc().InventoryHeight
		for y := 0; y < h; y++ {
			for x := 0; x < w; x++ {
				if i.Position.Y+y < 4 && i.Position.X+x < 10 {
					occupied[i.Position.Y+y][i.Position.X+x] = true
				}
			}
		}
	}

	for y, row := range d.ctx.CharacterCfg.Inventory.InventoryLock {
		if y < 4 {
			for x, cell := range row {
				if x < 10 && cell == 0 {
					occupied[y][x] = true
				}
			}
		}
	}

	free := 0
	for y := 0; y < 4; y++ {
		for x := 0; x < 10; x++ {
			if !occupied[y][x] {
				free++
			}
		}
	}
	return free
}

// Buy thawing potions in batches based on free inventory space, then consume them.
func (d Duriel) buyAndDrinkThawingPotions(mercAlive bool, updateTimer func(int)) error {
	selfTarget := thawingPotionsToDrink
	mercTarget := 0
	if mercAlive {
		mercTarget = thawingPotionsToDrink
	}

	drinkAndLog := func(selfTarget, mercTarget int) (int, int) {
		selfCount, mercCount := d.drinkThawingPotions(selfTarget, mercTarget)
		if updateTimer != nil {
			updateTimer(selfCount)
		}
		d.ctx.Logger.Info("Thawing potions consumed", "self", selfCount, "merc", mercCount)
		return selfCount, mercCount
	}

	selfCount, mercCount := drinkAndLog(selfTarget, mercTarget)
	selfTarget -= selfCount
	mercTarget -= mercCount
	if selfTarget < 0 {
		selfTarget = 0
	}
	if mercTarget < 0 {
		mercTarget = 0
	}

	for selfTarget > 0 || mercTarget > 0 {
		freeSlots := d.countFreeInventorySlots()
		if freeSlots == 0 {
			d.ctx.Logger.Warn("Not enough inventory space to buy thawing potions", "free", freeSlots, "required", selfTarget+mercTarget)
			return nil
		}

		batch := min(freeSlots, selfTarget+mercTarget)
		if err := d.buyThawingPotions(batch); err != nil {
			return err
		}
		d.ctx.RefreshGameData()

		selfCount, mercCount = drinkAndLog(selfTarget, mercTarget)
		if selfCount == 0 && mercCount == 0 {
			d.ctx.Logger.Warn("Failed to consume thawing potions after purchase", "remaining", selfTarget+mercTarget)
			return nil
		}

		selfTarget -= selfCount
		mercTarget -= mercCount
		if selfTarget < 0 {
			selfTarget = 0
		}
		if mercTarget < 0 {
			mercTarget = 0
		}
	}

	return nil
}

func (d Duriel) buyThawingPotions(quantity int) error {
	if quantity <= 0 {
		return nil
	}
	d.ctx.Logger.Info("Buying thawing potions from Lysander", "quantity", quantity)
	if err := action.InteractNPC(npc.Lysander); err != nil {
		return err
	}

	if err := action.BuyAtVendor(npc.Lysander, action.VendorItemRequest{
		Item:     "ThawingPotion",
		Quantity: quantity,
		Tab:      4,
	}); err != nil {
		return err
	}
	return step.CloseAllMenus()
}

func (d Duriel) prepareStaff() error {
	horadricStaff, found := d.ctx.Data.Inventory.Find("HoradricStaff", item.LocationInventory, item.LocationStash, item.LocationEquipped, item.LocationCube)
	if found {
		if horadricStaff.Location.LocationType == item.LocationCube {
			if err := action.EmptyCube(); err != nil {
				return err
			}
			d.ctx.RefreshGameData()
			utils.Sleep(200)
			horadricStaff, found = d.ctx.Data.Inventory.Find("HoradricStaff", item.LocationInventory, item.LocationStash, item.LocationEquipped)
			if !found {
				return errors.New("failed to move horadric staff out of cube")
			}
		}

		if horadricStaff.Location.LocationType == item.LocationStash {

			bank, found := d.ctx.Data.Objects.FindOne(object.Bank)
			if !found {
				d.ctx.Logger.Info("bank object not found")
			}

			err := action.InteractObject(bank, func() bool {
				return d.ctx.Data.OpenMenus.Stash
			})
			if err != nil {
				return err
			}

			screenPos := ui.GetScreenCoordsForItem(horadricStaff)
			d.ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
			utils.Sleep(300)
			step.CloseAllMenus()
		}
		return nil
	}

	staff, found := d.ctx.Data.Inventory.Find("StaffOfKings", item.LocationInventory, item.LocationStash, item.LocationEquipped, item.LocationCube)
	if !found {
		d.ctx.Logger.Info("Staff of Kings not found, skipping")
		return nil
	}

	amulet, found := d.ctx.Data.Inventory.Find("AmuletOfTheViper", item.LocationInventory, item.LocationStash, item.LocationEquipped, item.LocationCube)
	if !found {
		d.ctx.Logger.Info("Amulet of the Viper not found, skipping")
		return nil
	}

	staff, err := action.EnsureItemNotEquipped(staff)
	if err != nil {
		return err
	}

	amulet, err = action.EnsureItemNotEquipped(amulet)
	if err != nil {
		return err
	}

	err = action.CubeAddItems(staff, amulet)
	if err != nil {
		return err
	}

	err = action.CubeTransmute()
	if err != nil {
		return err
	}

	return nil
}

func (d Duriel) durielFilter() data.MonsterFilter {
	return func(a data.Monsters) []data.Monster {
		var filteredMonsters []data.Monster
		for _, mo := range a {
			if mo.Name == npc.Duriel {
				filteredMonsters = append(filteredMonsters, mo)
			}
		}

		return filteredMonsters
	}
}

func (d Duriel) tryTalkToJerhyn() bool {
	if d.ctx.Data.Quests[quest.Act2TheSevenTombs].HasStatus(quest.StatusLeaveTown + quest.StatusInProgress1) {
		d.ctx.Logger.Info("The Seven Tombs quest in progress 5. Speaking to Jerhyn.")
		action.MoveToCoords(data.Position{
			X: 5092,
			Y: 5144,
		})
		action.InteractNPC(npc.Jerhyn)
		utils.Sleep(500)
		return true
	}
	return false
}

func (d Duriel) tryTalkToMeshif() bool {
	if d.ctx.Data.Quests[quest.Act2TheSevenTombs].HasStatus(quest.StatusStarted+quest.StatusEnterArea+quest.StatusInProgress1) ||
		d.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		d.ctx.Logger.Info("Act 2, The Seven Tombs quest completed. Moving to Act 3.")
		action.MoveToCoords(data.Position{
			X: 5195,
			Y: 5060,
		})
		action.InteractNPC(npc.Meshif)
		utils.Sleep(500)
		d.ctx.HID.KeySequence(win.VK_SPACE)
		utils.Sleep(500)
		d.ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
		utils.Sleep(1000)
		action.HoldKey(win.VK_SPACE, 2000) // Hold the Escape key (VK_ESCAPE or 0x1B) for 2000 milliseconds (2 seconds)
		utils.Sleep(1000)
		return true
	}
	return false
}
