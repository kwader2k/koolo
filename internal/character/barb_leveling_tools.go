package character

import (
	"fmt"
	"slices"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/ui"
)

const (
	frenzyCooldown = 5 * time.Second
)

var _ = BarbLeveling.restoreEquipment

// ============================================
// BOSS EQUIPMENT MANAGEMENT
// ============================================
func (s BarbLeveling) isChangeNeeded(bossNPC npc.ID) bool {
	ctx := context.Get()
	playerLevel := 0
	if lvl, found := s.Data.PlayerUnit.FindStat(stat.Level, 0); found {
		playerLevel = lvl.Value
	}

	needsCannotBeFrozen := bossNPC == npc.Duriel || bossNPC == npc.Izual || bossNPC == npc.Mephisto || bossNPC == npc.BaalCrab
	currentWeapon := action.GetEquippedItem(ctx.Data.Inventory, item.LocLeftArm)
	currentGloves := action.GetEquippedItem(ctx.Data.Inventory, item.LocGloves)
	currentBoots := action.GetEquippedItem(ctx.Data.Inventory, item.LocFeet)
	leftRing := action.GetEquippedItem(ctx.Data.Inventory, item.LocLeftRing)
	rightRing := action.GetEquippedItem(ctx.Data.Inventory, item.LocRightRing)
	currentShield := action.GetEquippedItem(ctx.Data.Inventory, item.LocRightArm)

	hasCBWeapon := false
	if currentWeapon.UnitID != 0 {
		if cbStat, found := currentWeapon.FindStat(stat.CrushingBlow, 0); found && cbStat.Value > 0 {
			hasCBWeapon = true
		}
	}

	hasCBArmor := false
	if currentGloves.UnitID != 0 {
		if cbStat, found := currentGloves.FindStat(stat.CrushingBlow, 0); found && cbStat.Value > 0 {
			hasCBArmor = true
		}
	}
	if !hasCBArmor && currentBoots.UnitID != 0 {
		if cbStat, found := currentBoots.FindStat(stat.CrushingBlow, 0); found && cbStat.Value > 0 {
			hasCBArmor = true
		}
	}

	hasCannotBeFrozen := false
	if needsCannotBeFrozen && playerLevel >= 31 {
		hasCannotBeFrozen = (leftRing.UnitID != 0 && s.hasCannotBeFrozen(leftRing)) ||
			(rightRing.UnitID != 0 && s.hasCannotBeFrozen(rightRing)) ||
			(currentShield.UnitID != 0 && s.hasCannotBeFrozen(currentShield))
	} else if !needsCannotBeFrozen {
		hasCannotBeFrozen = true
	}

	if hasCBWeapon && hasCBArmor && hasCannotBeFrozen {
		return false
	}

	locations := []item.LocationType{
		item.LocationInventory,
		item.LocationEquipped,
	}
	if ctx.CharacterCfg.Game.Leveling.AutoEquipFromSharedStash {
		locations = append(locations, item.LocationSharedStash)
	}
	locations = append(locations, item.LocationStash)
	allItems := s.Data.Inventory.ByLocation(locations...)

	hasCBWeaponAvailable := false
	hasCBArmorAvailable := false
	hasCannotBeFrozenAvailable := false

	if !hasCBWeapon {
		for _, itm := range allItems {
			if _, isTwoHanded := itm.FindStat(stat.TwoHandedMinDamage, 0); isTwoHanded {
				continue
			}
			itemType := itm.Desc().Type
			if slices.Contains([]string{"shie", "ashd", "head"}, string(itemType)) {
				continue
			}
			bodyLocs := itm.Desc().GetType().BodyLocs
			if !slices.Contains(bodyLocs, item.LocLeftArm) {
				continue
			}
			if cbStat, found := itm.FindStat(stat.CrushingBlow, 0); found && cbStat.Value > 0 {
				hasCBWeaponAvailable = true
				break
			}
		}
	} else {
		hasCBWeaponAvailable = true
	}

	if !hasCBArmor {
		for _, itm := range allItems {
			bodyLocs := itm.Desc().GetType().BodyLocs
			if !slices.Contains(bodyLocs, item.LocGloves) && !slices.Contains(bodyLocs, item.LocFeet) {
				continue
			}
			if cbStat, found := itm.FindStat(stat.CrushingBlow, 0); found && cbStat.Value > 0 {
				hasCBArmorAvailable = true
				break
			}
		}
	} else {
		hasCBArmorAvailable = true
	}

	if needsCannotBeFrozen && playerLevel >= 31 && !hasCannotBeFrozen {
		hasRavenFrost := false
		hasRhymeShield := false

		for _, itm := range allItems {
			if itm.Name == "RavenFrost" || itm.Name == "Raven Frost" {
				bodyLocs := itm.Desc().GetType().BodyLocs
				if slices.Contains(bodyLocs, item.LocLeftRing) || slices.Contains(bodyLocs, item.LocRightRing) {
					hasRavenFrost = true
					break
				}
			}
		}

		if !hasRavenFrost {
			for _, itm := range allItems {
				itemType := itm.Desc().Type
				if !slices.Contains([]string{"shie", "ashd", "head"}, string(itemType)) {
					continue
				}
				bodyLocs := itm.Desc().GetType().BodyLocs
				if !slices.Contains(bodyLocs, item.LocRightArm) {
					continue
				}
				if s.hasCannotBeFrozen(itm) {
					if sockets, found := itm.FindStat(stat.NumSockets, 0); found && sockets.Value == 2 {
						if res, found := itm.FindStat(stat.FireResist, 0); found && res.Value >= 20 {
							hasRhymeShield = true
							break
						}
					}
				}
			}
		}

		hasCannotBeFrozenAvailable = hasRavenFrost || hasRhymeShield
	} else if !needsCannotBeFrozen || hasCannotBeFrozen {
		hasCannotBeFrozenAvailable = true
	}

	needsCBWeapon := !hasCBWeapon
	needsCBArmor := !hasCBArmor
	needsCannotBeFrozenItem := needsCannotBeFrozen && playerLevel >= 31 && !hasCannotBeFrozen

	if needsCBWeapon && !hasCBWeaponAvailable && needsCBArmor && !hasCBArmorAvailable && needsCannotBeFrozenItem && !hasCannotBeFrozenAvailable {
		s.Logger.Debug("No boss equipment items available, skipping boss equipment change")
		return false
	}

	return true
}

func (s BarbLeveling) PrepareBossEquipment(bossNPC npc.ID) {
	s.equipBossEquipment(bossNPC)
}

func (s BarbLeveling) equipBossEquipment(bossNPC npc.ID) {
	ctx := context.Get()

	if !s.isChangeNeeded(bossNPC) {
		return
	}

	ctx.IsBossEquipmentActive = true
	success := false
	defer func() {
		if !success {
			ctx.IsBossEquipmentActive = false
		}
	}()

	s.Logger.Info("Equipping boss-specific equipment (Crushing Blow and Cannot Be Frozen)")

	wasInTown := s.Data.PlayerUnit.Area.IsTown()
	if !wasInTown {
		s.Logger.Info("Not in town, returning via portal...")
		if err := action.ReturnTown(); err != nil {
			s.Logger.Error(fmt.Sprintf("Failed to return to town: %v", err))
			return
		}
		ctx.RefreshGameData()
		if !s.Data.PlayerUnit.Area.IsTown() {
			s.Logger.Error("Still not in town after ReturnTown, aborting equipment change")
			return
		}
	}

	s.Logger.Info("Identifying and stashing items before equipment change...")
	if err := action.IdentifyAll(false); err != nil {
		s.Logger.Warn(fmt.Sprintf("Error identifying items: %v", err))
	}
	time.Sleep(150 * time.Millisecond)

	if err := action.Stash(false); err != nil {
		s.Logger.Warn(fmt.Sprintf("Error stashing items: %v", err))
	}

	s.moveCloserToRefillNPC()

	if err := action.VendorRefill(action.VendorRefillOpts{ForceRefill: true, SellJunk: true, BuyConsumables: true}); err != nil {
		s.Logger.Warn(fmt.Sprintf("Error selling junk items: %v", err))
	}

	playerLevel := 0
	if lvl, found := s.Data.PlayerUnit.FindStat(stat.Level, 0); found {
		playerLevel = lvl.Value
	}

	refreshItems := func() []data.Item {
		time.Sleep(150 * time.Millisecond)
		locations := []item.LocationType{
			item.LocationStash,
			item.LocationInventory,
			item.LocationEquipped,
		}
		if ctx.CharacterCfg.Game.Leveling.AutoEquipFromSharedStash {
			locations = append(locations, item.LocationSharedStash)
		}
		return s.Data.Inventory.ByLocation(locations...)
	}

	needsCannotBeFrozen := bossNPC == npc.Duriel || bossNPC == npc.Izual || bossNPC == npc.Mephisto || bossNPC == npc.BaalCrab

	allItems := refreshItems()
	if playerLevel < 31 {
		s.equipCrushingBlowWeapon(allItems, item.LocLeftArm)
		allItems = refreshItems()
		s.equipCrushingBlowArmor(allItems)
	} else {
		s.equipCrushingBlowWeapon(allItems, item.LocLeftArm)
		allItems = refreshItems()
		s.equipCrushingBlowArmor(allItems)
		if needsCannotBeFrozen {
			allItems = refreshItems()
			s.equipCannotBeFrozenRing(allItems)
			allItems = refreshItems()
			s.equipCannotBeFrozenShield(allItems)
		}
	}

	if !wasInTown {
		s.Logger.Info("Returning to boss through portal...")
		if err := action.UsePortalInTown(); err != nil {
			s.Logger.Error(fmt.Sprintf("Failed to return to boss via portal: %v", err))
			return
		}
		ctx.RefreshGameData()
	}

	success = true
}

func (s BarbLeveling) restoreEquipment() {
	ctx := context.Get()

	if !ctx.IsBossEquipmentActive {
		return
	}

	s.Logger.Info("Restoring standard equipment after boss fight")

	wasInTown := s.Data.PlayerUnit.Area.IsTown()
	if !wasInTown {
		s.Logger.Info("Returning to town to restore equipment...")
		if err := action.ReturnTown(); err != nil {
			s.Logger.Warn(fmt.Sprintf("Failed to return to town for restoration: %v", err))
			return
		}
		ctx.RefreshGameData()
	}

	s.Logger.Info("Cleaning inventory before AutoEquip...")
	if err := action.IdentifyAll(false); err != nil {
		s.Logger.Warn(fmt.Sprintf("Error identifying items: %v", err))
	}
	time.Sleep(150 * time.Millisecond)

	if err := action.Stash(false); err != nil {
		s.Logger.Warn(fmt.Sprintf("Error stashing items: %v", err))
	}

	s.moveCloserToRefillNPC()

	if err := action.VendorRefill(action.VendorRefillOpts{ForceRefill: true, SellJunk: true, BuyConsumables: true}); err != nil {
		s.Logger.Warn(fmt.Sprintf("Error selling junk items: %v", err))
	}

	s.Logger.Info("Running AutoEquip to restore best gear...")
	if err := action.AutoEquip(); err != nil {
		s.Logger.Warn(fmt.Sprintf("AutoEquip failed during restoration: %v", err))
		return
	}
	ctx.RefreshGameData()

	ctx.IsBossEquipmentActive = false

	if err := action.Stash(false); err != nil {
		s.Logger.Warn(fmt.Sprintf("Error stashing items after AutoEquip: %v", err))
	}

	s.moveCloserToRefillNPC()
	if err := action.VendorRefill(action.VendorRefillOpts{ForceRefill: true, SellJunk: true, BuyConsumables: true}); err != nil {
		s.Logger.Warn(fmt.Sprintf("Error selling junk items after AutoEquip: %v", err))
	}

	if !wasInTown {
		s.Logger.Info("Returning to boss area after restoring gear...")
		if err := action.UsePortalInTown(); err != nil {
			s.Logger.Warn(fmt.Sprintf("Failed to return to boss area after restoration: %v", err))
			return
		}
		ctx.RefreshGameData()
	}
}

func (s BarbLeveling) moveCloserToRefillNPC() {
	ctx := context.Get()
	if !ctx.Data.PlayerUnit.Area.IsTown() {
		return
	}

	refillNPC := town.GetTownByArea(ctx.Data.PlayerUnit.Area).RefillNPC()
	if refillNPC == 0 {
		return
	}

	targetPos, found := s.findVendorPosition(refillNPC)
	if !found {
		s.Logger.Debug(fmt.Sprintf("Refill NPC %d position unavailable; skipping vendor reposition", int(refillNPC)))
		return
	}

	distance := ctx.PathFinder.DistanceFromMe(targetPos)
	if distance <= 6 {
		return
	}

	if err := action.MoveToCoords(targetPos); err != nil {
		s.Logger.Warn(fmt.Sprintf("Failed to move near vendor NPC %d: %v", int(refillNPC), err))
		return
	}

	ctx.RefreshGameData()
}

func (s BarbLeveling) findVendorPosition(npcID npc.ID) (data.Position, bool) {
	ctx := context.Get()

	if townNPC, found := ctx.Data.Monsters.FindOne(npcID, data.MonsterTypeNone); found {
		return townNPC.Position, true
	}

	if npcInfo, ok := ctx.Data.NPCs.FindOne(npcID); ok && len(npcInfo.Positions) > 0 {
		playerPos := ctx.Data.PlayerUnit.Position
		bestPos := npcInfo.Positions[0]
		bestDist := distanceSquared(playerPos, bestPos)
		for _, pos := range npcInfo.Positions[1:] {
			if dist := distanceSquared(playerPos, pos); dist < bestDist {
				bestPos = pos
				bestDist = dist
			}
		}
		return bestPos, true
	}

	return data.Position{}, false
}

func distanceSquared(a, b data.Position) int {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return dx*dx + dy*dy
}

func (s BarbLeveling) equipCrushingBlowWeapon(allItems []data.Item, slot item.LocationType) {
	ctx := context.Get()

	currentWeapon := action.GetEquippedItem(ctx.Data.Inventory, slot)
	if currentWeapon.UnitID != 0 {
		if cbStat, found := currentWeapon.FindStat(stat.CrushingBlow, 0); found && cbStat.Value > 0 {
			return
		}
	}

	var cbWeapons []data.Item
	for _, itm := range allItems {
		if _, isTwoHanded := itm.FindStat(stat.TwoHandedMinDamage, 0); isTwoHanded {
			continue
		}
		itemType := itm.Desc().Type
		if slices.Contains([]string{"shie", "ashd", "head"}, string(itemType)) {
			continue
		}

		bodyLocs := itm.Desc().GetType().BodyLocs
		if !slices.Contains(bodyLocs, slot) {
			continue
		}

		if cbStat, found := itm.FindStat(stat.CrushingBlow, 0); found && cbStat.Value > 0 {
			cbWeapons = append(cbWeapons, itm)
		}
	}

	if len(cbWeapons) == 0 {
		s.Logger.Debug("No Crushing Blow weapons found in stash/inventory")
		return
	}

	sort.Slice(cbWeapons, func(i, j int) bool {
		canEquipI := action.IsItemEquippable(cbWeapons[i], slot, item.LocationEquipped)
		canEquipJ := action.IsItemEquippable(cbWeapons[j], slot, item.LocationEquipped)
		if canEquipI != canEquipJ {
			return canEquipI
		}
		if !canEquipI {
			return false
		}
		scoreI := action.CalculateItemScore(cbWeapons[i])
		scoreJ := action.CalculateItemScore(cbWeapons[j])
		return scoreI > scoreJ
	})

	for _, weapon := range cbWeapons {
		if weapon.UnitID == 0 || weapon.IdentifiedName == "" {
			s.Logger.Debug(fmt.Sprintf("Skipping invalid weapon (UnitID: %d, Name: '%s')", weapon.UnitID, weapon.IdentifiedName))
			continue
		}

		if action.IsItemEquippable(weapon, slot, item.LocationEquipped) {
			s.Logger.Info(fmt.Sprintf("Equipping Crushing Blow weapon: %s (UnitID: %d)", weapon.IdentifiedName, weapon.UnitID))
			oldWeapon := currentWeapon

			if err := action.EquipItem(weapon, slot, item.LocationEquipped); err == nil {
				ctx.RefreshGameData()
				if oldWeapon.UnitID != 0 {
					s.stashItem(oldWeapon)
				}
				return
			} else {
				s.Logger.Warn(fmt.Sprintf("Failed to equip %s (UnitID: %d): %v", weapon.IdentifiedName, weapon.UnitID, err))
			}
		}
	}
}

func (s BarbLeveling) equipCrushingBlowArmor(allItems []data.Item) {
	ctx := context.Get()
	armorSlots := []item.LocationType{item.LocGloves, item.LocFeet}

	for _, slot := range armorSlots {
		currentArmor := action.GetEquippedItem(ctx.Data.Inventory, slot)
		if currentArmor.UnitID != 0 {
			if cbStat, found := currentArmor.FindStat(stat.CrushingBlow, 0); found && cbStat.Value > 0 {
				continue
			}
		}
		var cbArmor []data.Item
		for _, itm := range allItems {
			bodyLocs := itm.Desc().GetType().BodyLocs
			if !slices.Contains(bodyLocs, slot) {
				continue
			}
			if cbStat, found := itm.FindStat(stat.CrushingBlow, 0); found && cbStat.Value > 0 {
				cbArmor = append(cbArmor, itm)
			}
		}

		if len(cbArmor) == 0 {
			continue
		}

		sort.Slice(cbArmor, func(i, j int) bool {
			canEquipI := action.IsItemEquippable(cbArmor[i], slot, item.LocationEquipped)
			canEquipJ := action.IsItemEquippable(cbArmor[j], slot, item.LocationEquipped)
			if canEquipI != canEquipJ {
				return canEquipI
			}
			if !canEquipI {
				return false
			}
			scoreI := action.CalculateItemScore(cbArmor[i])
			scoreJ := action.CalculateItemScore(cbArmor[j])
			return scoreI > scoreJ
		})

		for _, armor := range cbArmor {
			if action.IsItemEquippable(armor, slot, item.LocationEquipped) {
				s.Logger.Info(fmt.Sprintf("Equipping Crushing Blow armor: %s to %s", armor.IdentifiedName, slot))
				oldArmor := currentArmor

				if err := action.EquipItem(armor, slot, item.LocationEquipped); err == nil {
					ctx.RefreshGameData()
					if oldArmor.UnitID != 0 {
						s.stashItem(oldArmor)
					}
					break
				} else {
					s.Logger.Warn(fmt.Sprintf("Failed to equip %s: %v", armor.IdentifiedName, err))
				}
			}
		}
	}
}

func (s BarbLeveling) equipCannotBeFrozenRing(allItems []data.Item) {
	ctx := context.Get()

	leftRing := action.GetEquippedItem(ctx.Data.Inventory, item.LocLeftRing)
	rightRing := action.GetEquippedItem(ctx.Data.Inventory, item.LocRightRing)
	if (leftRing.UnitID != 0 && s.hasCannotBeFrozen(leftRing)) ||
		(rightRing.UnitID != 0 && s.hasCannotBeFrozen(rightRing)) {
		return
	}

	var ravenFrost data.Item
	for _, itm := range allItems {
		if itm.Name == "RavenFrost" || itm.Name == "Raven Frost" {
			bodyLocs := itm.Desc().GetType().BodyLocs
			if slices.Contains(bodyLocs, item.LocLeftRing) || slices.Contains(bodyLocs, item.LocRightRing) {
				ravenFrost = itm
				break
			}
		}
	}

	if ravenFrost.UnitID == 0 {
		s.Logger.Debug("No Raven Frost ring found")
		return
	}

	slot := item.LocLeftRing
	oldRing := leftRing
	if leftRing.UnitID != 0 {
		slot = item.LocRightRing
		oldRing = rightRing
	}

	if action.IsItemEquippable(ravenFrost, slot, item.LocationEquipped) {
		s.Logger.Info(fmt.Sprintf("Equipping Raven Frost ring to %s", slot))

		if err := action.EquipItem(ravenFrost, slot, item.LocationEquipped); err == nil {
			ctx.RefreshGameData()
			if oldRing.UnitID != 0 {
				s.stashItem(oldRing)
			}
		} else {
			s.Logger.Warn(fmt.Sprintf("Failed to equip Raven Frost ring: %v", err))
		}
	}
}

func (s BarbLeveling) equipCannotBeFrozenShield(allItems []data.Item) {
	ctx := context.Get()
	currentShield := action.GetEquippedItem(ctx.Data.Inventory, item.LocRightArm)
	if currentShield.UnitID != 0 && s.hasCannotBeFrozen(currentShield) {
		return
	}
	var rhymeShield data.Item
	for _, itm := range allItems {
		itemType := itm.Desc().Type
		if !slices.Contains([]string{"shie", "ashd", "head"}, string(itemType)) {
			continue
		}
		bodyLocs := itm.Desc().GetType().BodyLocs
		if !slices.Contains(bodyLocs, item.LocRightArm) {
			continue
		}
		if s.hasCannotBeFrozen(itm) {
			if sockets, found := itm.FindStat(stat.NumSockets, 0); found && sockets.Value == 2 {
				if res, found := itm.FindStat(stat.FireResist, 0); found && res.Value >= 20 {
					rhymeShield = itm
					break
				}
			}
		}
	}

	if rhymeShield.UnitID == 0 {
		s.Logger.Debug("No Rhyme shield found")
		return
	}

	if action.IsItemEquippable(rhymeShield, item.LocRightArm, item.LocationEquipped) {
		s.Logger.Info("Equipping Rhyme shield")
		oldShield := currentShield

		if err := action.EquipItem(rhymeShield, item.LocRightArm, item.LocationEquipped); err == nil {
			ctx.RefreshGameData()
			if oldShield.UnitID != 0 {
				s.stashItem(oldShield)
			}
		} else {
			s.Logger.Warn(fmt.Sprintf("Failed to equip Rhyme shield: %v", err))
		}
	}
}

func (s BarbLeveling) hasCannotBeFrozen(itm data.Item) bool {
	_, found := itm.FindStat(stat.CannotBeFrozen, 0)
	return found
}

func (s BarbLeveling) stashItem(itm data.Item) {
	ctx := context.Get()

	findInventoryItem := func() (data.Item, bool) {
		for _, invItem := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
			if invItem.UnitID == itm.UnitID {
				return invItem, true
			}
		}
		return data.Item{}, false
	}

	ctx.RefreshGameData()
	foundItem, ok := findInventoryItem()
	if !ok {
		s.Logger.Debug(fmt.Sprintf("Item %s not yet in inventory, waiting briefly", itm.IdentifiedName))
		time.Sleep(300 * time.Millisecond)
		ctx.RefreshGameData()
		foundItem, ok = findInventoryItem()
		if !ok {
			s.Logger.Debug(fmt.Sprintf("Item %s still not in inventory, skipping stash", itm.IdentifiedName))
			return
		}
	}

	if err := action.OpenStash(); err != nil {
		s.Logger.Warn(fmt.Sprintf("Failed to open stash: %v", err))
		return
	}

	if !ctx.Data.OpenMenus.Inventory {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		time.Sleep(500 * time.Millisecond)
	}

	ctx.RefreshGameData()
	foundItem, ok = findInventoryItem()
	if !ok {
		s.Logger.Debug(fmt.Sprintf("Item %s not found in inventory after refresh", itm.IdentifiedName))
		action.CloseStash()
		return
	}

	startTab := 1
	if ctx.CharacterCfg.Character.StashToShared {
		startTab = 2
	}

	stashed := false
	for tab := startTab; tab <= 4; tab++ {
		action.SwitchStashTab(tab)
		time.Sleep(300 * time.Millisecond)

		coords := ui.GetScreenCoordsForItem(foundItem)
		ctx.HID.ClickWithModifier(game.LeftButton, coords.X, coords.Y, game.CtrlKey)
		time.Sleep(500 * time.Millisecond)

		ctx.RefreshGameData()
		stashed = true
		for _, invItem := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
			if invItem.UnitID == foundItem.UnitID {
				stashed = false
				break
			}
		}

		if stashed {
			s.Logger.Info(fmt.Sprintf("Successfully stashed %s to tab %d", foundItem.IdentifiedName, tab))
			break
		}
	}

	if !stashed {
		s.Logger.Warn(fmt.Sprintf("Failed to stash %s - all stash tabs may be full", foundItem.IdentifiedName))
	}

	action.CloseStash()
	ctx.RefreshGameData()
}

// ============================================
// COMBAT HELPERS
// ============================================
func (s BarbLeveling) hasSkill(sk skill.ID) bool {
	return s.Data.PlayerUnit.Skills[sk].Level > 0
}

func (s BarbLeveling) getManaPercentage() float64 {
	currentMana, foundMana := s.Data.PlayerUnit.FindStat(stat.Mana, 0)
	maxMana, foundMaxMana := s.Data.PlayerUnit.FindStat(stat.MaxMana, 0)
	if !foundMana || !foundMaxMana || maxMana.Value == 0 {
		return 0
	}
	return float64(currentMana.Value) / float64(maxMana.Value) * 100
}

func (s BarbLeveling) barbFCR() time.Duration {
	fcr, found := s.Data.PlayerUnit.FindStat(stat.FasterCastRate, 0)
	if !found || fcr.Value < 26 {
		return 500 * time.Millisecond
	}
	return 300 * time.Millisecond
}

func (s BarbLeveling) hasDualOneHand() bool {
	left := action.GetEquippedItem(s.Data.Inventory, item.LocLeftArm)
	right := action.GetEquippedItem(s.Data.Inventory, item.LocRightArm)
	if left.UnitID == 0 || right.UnitID == 0 {
		return false
	}

	shieldTypes := []string{"shie", "ashd", "head"}
	leftIsShield := slices.Contains(shieldTypes, string(left.Desc().Type))
	rightIsShield := slices.Contains(shieldTypes, string(right.Desc().Type))

	return !leftIsShield && !rightIsShield
}

func (s BarbLeveling) hasMonstersInRange(rangeYards int) bool {
	for _, m := range s.Data.Monsters.Enemies() {
		if m.Stats[stat.Life] <= 0 {
			continue
		}
		distance := s.PathFinder.DistanceFromMe(m.Position)
		if distance <= rangeYards {
			return true
		}
	}
	return false
}

// ============================================
// SKILL PERFORMANCE
// ============================================
func (s BarbLeveling) tryLeapAttack(id data.UnitID, leapAttackExecuted *bool) bool {
	excludedAreas := []area.ID{area.MaggotLairLevel1, area.MaggotLairLevel2, area.MaggotLairLevel3, area.ChaosSanctuary}
	if slices.Contains(excludedAreas, s.Data.PlayerUnit.Area) {
		return false
	}

	if !*leapAttackExecuted && s.hasSkill(skill.LeapAttack) {
		if !s.hasMonstersInRange(6) {
			if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.LeapAttack); found {
				step.SecondaryAttack(skill.LeapAttack, id, 1, step.RangedDistance(5, 15))
				*leapAttackExecuted = true
				return true
			}
		} else {
			*leapAttackExecuted = true
		}
	}
	return false
}

func (s BarbLeveling) tryHowl(id data.UnitID, lastHowlCast *time.Time) bool {
	hasHowl := s.Data.PlayerUnit.Skills[skill.Howl].Level > 0
	if s.CharacterCfg.Character.BarbLeveling.UseHowl && hasHowl {
		return s.PerformHowl(id, lastHowlCast)
	}
	return false
}

func (s BarbLeveling) tryBattleCry(id data.UnitID, lastBattleCryCast *time.Time) bool {
	hasBattleCry := s.Data.PlayerUnit.Skills[skill.BattleCry].Level > 0
	if s.CharacterCfg.Character.BarbLeveling.UseBattleCry && hasBattleCry {
		return s.PerformBattleCry(id, lastBattleCryCast)
	}
	return false
}

func (s BarbLeveling) PerformHowl(targetID data.UnitID, lastHowlCast *time.Time) bool {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	howlCooldownSeconds := s.CharacterCfg.Character.BarbLeveling.HowlCooldown
	if howlCooldownSeconds <= 0 {
		howlCooldownSeconds = 8
	}
	howlCooldown := time.Duration(howlCooldownSeconds) * time.Second

	minMonsters := s.CharacterCfg.Character.BarbLeveling.HowlMinMonsters
	if minMonsters <= 0 {
		minMonsters = 4
	}

	if !lastHowlCast.IsZero() && time.Since(*lastHowlCast) < howlCooldown {
		return false
	}

	const howlRange = 4
	closeMonsters := 0

	for _, m := range ctx.Data.Monsters.Enemies() {
		if m.Stats[stat.Life] <= 0 {
			continue
		}

		distance := s.PathFinder.DistanceFromMe(m.Position)
		if distance <= howlRange {
			closeMonsters++
		}
	}

	if closeMonsters < minMonsters {
		return false
	}

	_, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.Howl)
	if !found {
		return false
	}

	*lastHowlCast = time.Now()

	time.Sleep(100 * time.Millisecond)

	err := step.SecondaryAttack(skill.Howl, targetID, 1, step.Distance(1, 10))
	if err != nil {
		return false
	}

	time.Sleep(s.barbFCR())

	return true
}

func (s BarbLeveling) PerformBattleCry(monsterID data.UnitID, lastBattleCryCast *time.Time) bool {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	manaPercentage := s.getManaPercentage()
	if manaPercentage < 20 {
		return false
	}

	battleCryCooldownSeconds := s.CharacterCfg.Character.BarbLeveling.BattleCryCooldown
	if battleCryCooldownSeconds <= 0 {
		battleCryCooldownSeconds = 6
	}
	battleCryCooldown := time.Duration(battleCryCooldownSeconds) * time.Second

	if !lastBattleCryCast.IsZero() && time.Since(*lastBattleCryCast) < battleCryCooldown {
		return false
	}

	minMonsters := s.CharacterCfg.Character.BarbLeveling.BattleCryMinMonsters
	if minMonsters <= 0 {
		minMonsters = 1
	}

	const battleCryRange = 4
	closeMonsters := 0

	for _, m := range ctx.Data.Monsters.Enemies() {
		if m.Stats[stat.Life] <= 0 {
			continue
		}

		distance := s.PathFinder.DistanceFromMe(m.Position)
		if distance <= battleCryRange {
			closeMonsters++
		}
	}

	if closeMonsters < minMonsters {
		return false
	}

	if _, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.BattleCry); found {
		*lastBattleCryCast = time.Now()

		time.Sleep(100 * time.Millisecond)

		err := step.SecondaryAttack(skill.BattleCry, monsterID, 1, step.Distance(1, 5))
		if err != nil {
			return false
		}

		time.Sleep(s.barbFCR())

		return true
	}

	return false
}

func (s BarbLeveling) PerformBattleCryBoss(monsterID data.UnitID, lastBattleCryCast *time.Time) bool {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	manaPercentage := s.getManaPercentage()
	if manaPercentage < 20 {
		return false
	}

	battleCryCooldownSeconds := s.CharacterCfg.Character.BarbLeveling.BattleCryCooldown
	if battleCryCooldownSeconds <= 0 {
		battleCryCooldownSeconds = 6
	}
	battleCryCooldown := time.Duration(battleCryCooldownSeconds) * time.Second

	if !lastBattleCryCast.IsZero() && time.Since(*lastBattleCryCast) < battleCryCooldown {
		return false
	}

	if _, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.BattleCry); found {
		*lastBattleCryCast = time.Now()

		time.Sleep(100 * time.Millisecond)

		err := step.SecondaryAttack(skill.BattleCry, monsterID, 1, step.Distance(1, 5))
		if err != nil {
			return false
		}

		time.Sleep(s.barbFCR())

		return true
	}

	return false
}

// ============================================
// ATTACK ROUTINES
// ============================================
func (s BarbLeveling) executeAttackUnderLevel6(id data.UnitID, lastHowlCast *time.Time) bool {
	if s.tryHowl(id, lastHowlCast) {
		return true
	}

	if s.hasSkill(skill.Bash) {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Bash); found {
			step.SecondaryAttack(skill.Bash, id, 1, step.Distance(1, 3))
			return true
		}
	}

	step.PrimaryAttack(id, 1, false, step.Distance(1, 3))
	return true
}

func (s BarbLeveling) executeAttackPhysicalImmune(id data.UnitID, lastHowlCast *time.Time, leapAttackExecuted *bool) bool {
	if s.tryLeapAttack(id, leapAttackExecuted) {
		return true
	}

	if s.tryHowl(id, lastHowlCast) {
		return true
	}

	if s.hasSkill(skill.Berserk) {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Berserk); found {
			step.SecondaryAttack(skill.Berserk, id, 1, step.Distance(1, 3))
			step.SecondaryAttack(skill.Berserk, id, 1, step.Distance(1, 3))
			return true
		}
	}

	if s.hasSkill(skill.WarCry) {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.WarCry); found {
			step.SecondaryAttack(skill.WarCry, id, 1, step.Distance(1, 3))
			return true
		}
	}

	step.PrimaryAttack(id, 1, false, step.Distance(1, 3))
	return true
}

func (s BarbLeveling) executeAttackPreWarcry(
	id data.UnitID,
	hasDualOneHand bool,
	lastHowlCast *time.Time,
	lastBattleCryCast *time.Time,
	lastFrenzyCast *time.Time,
	lastDoubleSwingCast *time.Time,
	leapAttackExecuted *bool,
) bool {
	if s.tryLeapAttack(id, leapAttackExecuted) {
		return true
	}

	if s.tryHowl(id, lastHowlCast) {
		return true
	}

	if s.tryBattleCry(id, lastBattleCryCast) {
		return true
	}

	if s.hasSkill(skill.Frenzy) {
		if lastFrenzyCast.IsZero() || time.Since(*lastFrenzyCast) >= frenzyCooldown {
			if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Frenzy); found {
				step.SecondaryAttack(skill.Frenzy, id, 1, step.Distance(1, 3))
				*lastFrenzyCast = time.Now()
				return true
			}
		}
	}

	const doubleSwingCooldown = 400 * time.Millisecond
	if s.hasSkill(skill.DoubleSwing) && hasDualOneHand {
		if lastDoubleSwingCast.IsZero() || time.Since(*lastDoubleSwingCast) >= doubleSwingCooldown {
			if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.DoubleSwing); found {
				step.SecondaryAttack(skill.DoubleSwing, id, 1, step.Distance(1, 3))
				*lastDoubleSwingCast = time.Now()
				time.Sleep(400 * time.Millisecond)
				return true
			}
		}
	}

	step.PrimaryAttack(id, 1, false, step.Distance(1, 3))
	return true
}

func (s BarbLeveling) executeAttackWarcry(
	id data.UnitID,
	lastHowlCast *time.Time,
	lastBattleCryCast *time.Time,
	lastWarCryCast *time.Time,
	leapAttackExecuted *bool,
) bool {
	if s.tryLeapAttack(id, leapAttackExecuted) {
		return true
	}

	if s.tryHowl(id, lastHowlCast) {
		return true
	}

	if s.tryBattleCry(id, lastBattleCryCast) {
		return true
	}

	if s.hasSkill(skill.WarCry) {
		manaPercentage := s.getManaPercentage()
		if manaPercentage > 5 {
			hasHowl := s.Data.PlayerUnit.Skills[skill.Howl].Level > 0
			howlEnabled := s.CharacterCfg.Character.BarbLeveling.UseHowl && hasHowl
			if howlEnabled && !lastHowlCast.IsZero() {
				howlWaitTime := s.barbFCR()
				if time.Since(*lastHowlCast) < howlWaitTime {
				} else {
					hasBattleCry := s.Data.PlayerUnit.Skills[skill.BattleCry].Level > 0
					battleCryEnabled := s.CharacterCfg.Character.BarbLeveling.UseBattleCry && hasBattleCry
					if battleCryEnabled && !lastBattleCryCast.IsZero() {
						battleCryWaitTime := s.barbFCR()
						if time.Since(*lastBattleCryCast) < battleCryWaitTime {
						} else {
							if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.WarCry); found {
								step.SecondaryAttack(skill.WarCry, id, 1, step.Distance(1, 3))
								*lastWarCryCast = time.Now()
								return true
							}
						}
					} else {
						if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.WarCry); found {
							step.SecondaryAttack(skill.WarCry, id, 1, step.Distance(1, 3))
							*lastWarCryCast = time.Now()
							return true
						}
					}
				}
			} else {
				hasBattleCry := s.Data.PlayerUnit.Skills[skill.BattleCry].Level > 0
				battleCryEnabled := s.CharacterCfg.Character.BarbLeveling.UseBattleCry && hasBattleCry
				if battleCryEnabled && !lastBattleCryCast.IsZero() {
					battleCryWaitTime := s.barbFCR()
					if time.Since(*lastBattleCryCast) < battleCryWaitTime {
					} else {
						if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.WarCry); found {
							step.SecondaryAttack(skill.WarCry, id, 1, step.Distance(1, 3))
							*lastWarCryCast = time.Now()
							return true
						}
					}
				} else {
					if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.WarCry); found {
						step.SecondaryAttack(skill.WarCry, id, 1, step.Distance(1, 3))
						*lastWarCryCast = time.Now()
						return true
					}
				}
			}
		}
	}

	step.PrimaryAttack(id, 1, false, step.Distance(1, 3))
	return true
}

func (s BarbLeveling) executeAttackBoss(
	id data.UnitID,
	bossNPC npc.ID,
	hasDualOneHand bool,
	lastHowlCast *time.Time,
	lastBattleCryCast *time.Time,
	lastWarCryCast *time.Time,
	lastBerserkCast *time.Time,
	leapAttackExecuted *bool,
) bool {
	if s.tryLeapAttack(id, leapAttackExecuted) {
		return true
	}

	if s.tryHowl(id, lastHowlCast) {
		return true
	}

	hasBattleCry := s.Data.PlayerUnit.Skills[skill.BattleCry].Level > 0
	if s.CharacterCfg.Character.BarbLeveling.UseBattleCry && hasBattleCry {
		if s.PerformBattleCryBoss(id, lastBattleCryCast) {
			return true
		}
	}

	hasFreeze := bossNPC == npc.Duriel || bossNPC == npc.Izual || bossNPC == npc.BaalCrab

	if s.hasSkill(skill.Berserk) {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Berserk); found {
			if hasFreeze {
				const berserkFreezeCooldown = 1 * time.Second
				if !lastBerserkCast.IsZero() && time.Since(*lastBerserkCast) < berserkFreezeCooldown {
				} else {
					step.SecondaryAttack(skill.Berserk, id, 1, step.Distance(1, 3))
					step.SecondaryAttack(skill.Berserk, id, 1, step.Distance(1, 3))
					step.SecondaryAttack(skill.Berserk, id, 1, step.Distance(1, 3))
					*lastBerserkCast = time.Now()
					time.Sleep(200 * time.Millisecond)
					return true
				}
			} else {
				step.SecondaryAttack(skill.Berserk, id, 1, step.Distance(1, 3))
				step.SecondaryAttack(skill.Berserk, id, 1, step.Distance(1, 3))
				step.SecondaryAttack(skill.Berserk, id, 1, step.Distance(1, 3))
				return true
			}
		}
	}

	if s.hasSkill(skill.DoubleSwing) && hasDualOneHand {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.DoubleSwing); found {
			step.SecondaryAttack(skill.DoubleSwing, id, 1, step.Distance(1, 3))
			step.SecondaryAttack(skill.DoubleSwing, id, 1, step.Distance(1, 3))
			step.SecondaryAttack(skill.DoubleSwing, id, 1, step.Distance(1, 3))
			if hasFreeze {
				time.Sleep(600 * time.Millisecond)
			}
			return true
		}
	}

	if s.hasSkill(skill.WarCry) {
		manaPercentage := s.getManaPercentage()
		if manaPercentage > 5 {
			if lastWarCryCast.IsZero() || time.Since(*lastWarCryCast) >= 2*time.Second {
				if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.WarCry); found {
					step.SecondaryAttack(skill.WarCry, id, 1, step.Distance(1, 3))
					*lastWarCryCast = time.Now()
					return true
				}
			}
		}
	}

	step.PrimaryAttack(id, 1, false, step.Distance(1, 3))
	return true
}
