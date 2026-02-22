package run

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Torch struct {
	ctx                    *context.Status
	savedTorchInventoryPos data.Position
	savedTorchStashTab     int
	savedTorchUnitID       data.UnitID
	newTorchUnitID         data.UnitID
}

func NewTorch() *Torch {
	return &Torch{
		ctx: context.Get(),
	}
}

func (t Torch) Name() string {
	return string(config.PandemoniumRun)
}

func (t Torch) CheckConditions(parameters *RunParameters) SequencerResult {
	if !hasOrgans(t.ctx) {
		t.ctx.Logger.Warn("Not enough organs in stash. Need 1x Mephisto's Brain, 1x Diablo's Horn, 1x Baal's Eye")
		return SequencerSkip
	}
	return SequencerOk
}

func (t Torch) Run(parameters *RunParameters) error {
	if err := getCube(t.ctx); err != nil {
		return err
	}

	if err := checkForRejuv(t.ctx); err != nil {
		t.ctx.Logger.Warn(fmt.Sprintf("Failed to check/fill rejuvs: %v", err))
	}

	utils.Sleep(500)
	t.ctx.RefreshGameData()

	hasBattleOrders := t.ctx.Data.PlayerUnit.States.HasState(state.Battleorders)
	hasBattleCommand := t.ctx.Data.PlayerUnit.States.HasState(state.Battlecommand)

	if !hasBattleOrders || !hasBattleCommand {
		if err := action.WayPoint(area.FrigidHighlands); err != nil {
			t.ctx.Logger.Warn(fmt.Sprintf("Failed to visit Frigid Highlands waypoint: %v", err))
		} else {
			utils.Sleep(500)
			t.ctx.RefreshGameData()
			if !t.ctx.Data.PlayerUnit.Area.IsTown() || t.ctx.Data.PlayerUnit.Area != area.Harrogath {
				if err := action.WayPoint(area.Harrogath); err != nil {
					t.ctx.Logger.Warn(fmt.Sprintf("Failed to return to Harrogath: %v", err))
				} else {
					utils.Sleep(500)
					t.ctx.RefreshGameData()
				}
			}
		}
	}

	if err := openStash(t.ctx); err != nil {
		return fmt.Errorf("failed to open stash: %w", err)
	}

	organs, err := getOrganSet(t.ctx)
	if err != nil {
		return fmt.Errorf("failed to get organs: %w", err)
	}

	portal, err := openUT(t.ctx, organs)
	if err != nil {
		return fmt.Errorf("failed to open Uber Tristram portal: %w", err)
	}

	if err := enterUberTristramPortal(t.ctx, portal); err != nil {
		return fmt.Errorf("failed to enter portal: %w", err)
	}

	action.Buff()

	t.ctx.Logger.Info("Starting Uber Tristram run")
	if err := t.runUberTristram(parameters); err != nil {
		return fmt.Errorf("failed to complete Uber Tristram: %w", err)
	}
	t.ctx.Logger.Info("Successfully completed Uber Tristram run")

	if err := action.ReturnTown(); err != nil {
		t.ctx.Logger.Warn(fmt.Sprintf("Failed to return to town: %v", err))
	}

	utils.Sleep(200)

	goToMalahIfInHarrogath(t.ctx)
	if err := action.VendorRefill(action.VendorRefillOpts{ForceRefill: true, SellJunk: true, BuyConsumables: true}); err != nil {
		t.ctx.Logger.Warn(fmt.Sprintf("Failed to visit vendor: %v", err))
	}

	action.ReviveMerc()

	if err := action.Stash(true); err != nil {
		t.ctx.Logger.Warn(fmt.Sprintf("Failed to stash items: %v", err))
	}

	return nil
}

func (t Torch) enterPortalAndStop(portal data.Object) error {
	if err := enterUberTristramPortal(t.ctx, portal); err != nil {
		return err
	}

	portalPos := data.Position{X: 25100, Y: 5093}
	if err := action.MoveToCoords(portalPos); err != nil {
		t.ctx.Logger.Warn(fmt.Sprintf("MoveToCoords to portal entrance failed: %v", err))
	}

	return nil
}

func (t Torch) findPortal() (data.Object, error) {
	return findUberTristramPortal(t.ctx)
}

func (t Torch) runUberTristram(parameters *RunParameters) error {
	if err := t.mephisto(); err != nil {
		return fmt.Errorf("failed to kill Mephisto: %w", err)
	}

	if err := t.baal(); err != nil {
		return fmt.Errorf("failed to kill Baal: %w", err)
	}

	if err := t.diablo(); err != nil {
		return fmt.Errorf("failed to complete Diablo/Torch search: %w", err)
	}

	return nil
}

func (t Torch) mephisto() error {
	t.ctx.Logger.Info("Starting Uber Mephisto fight")
	for _, p := range mephPath() {
		if err := action.MoveToCoords(p); err != nil {
			return fmt.Errorf("failed to reach coordinate: %w", err)
		}
		t.ctx.RefreshGameData()
	}

	if err := action.MoveToCoords(mephLurePos()); err != nil {
		return fmt.Errorf("failed to reach lure position: %w", err)
	}
	t.ctx.RefreshGameData()
	utils.Sleep(1300)

	if err := action.MoveToCoords(mephSafePos()); err != nil {
		return fmt.Errorf("failed to reach safe spot: %w", err)
	}
	t.ctx.RefreshGameData()
	utils.Sleep(1200)
	t.ctx.RefreshGameData()

	const maxDistance = 20
	const maxRetries = 3
	retryCount := 0

	for retryCount < maxRetries {
		found, _, _ := isUberMephistoNearby(t.ctx, maxDistance)
		if found {
			t.ctx.Logger.Info("Found Uber Mephisto, starting fight")
			if err := t.ctx.Char.KillUberMephisto(); err != nil {
				return fmt.Errorf("failed to kill Uber Mephisto: %w", err)
			}
			t.ctx.Logger.Info("Successfully killed Uber Mephisto")
			break
		}

		retryCount++
		if retryCount >= maxRetries {
			t.ctx.Logger.Info("Uber Mephisto not found nearby, attempting kill anyway")
			if err := t.ctx.Char.KillUberMephisto(); err != nil {
				return fmt.Errorf("failed to kill Uber Mephisto: %w", err)
			}
			t.ctx.Logger.Info("Successfully killed Uber Mephisto")
			break
		}

		if err := action.MoveToCoords(mephLurePos()); err != nil {
			return fmt.Errorf("failed to reach lure position on retry: %w", err)
		}
		t.ctx.RefreshGameData()
		utils.Sleep(500)

		if err := action.MoveToCoords(mephSafePos()); err != nil {
			return fmt.Errorf("failed to reach safe spot on retry: %w", err)
		}
		t.ctx.RefreshGameData()
		utils.Sleep(1200)
		t.ctx.RefreshGameData()
	}

	if err := t.town(); err != nil {
		return fmt.Errorf("failed to return to town after Mephisto: %w", err)
	}

	utils.Sleep(200)
	if err := vendorRefillOrHeal(t.ctx); err != nil {
		t.ctx.Logger.Warn(fmt.Sprintf("Failed to visit vendor/heal: %v", err))
	}

	action.ReviveMerc()

	if err := checkForRejuv(t.ctx); err != nil {
		t.ctx.Logger.Warn(fmt.Sprintf("Failed to check/fill rejuvs: %v", err))
	}

	if err := goToStashPosition(t.ctx); err != nil {
		return fmt.Errorf("failed to go to stash: %w", err)
	}

	if err := t.returnToUberTristram(); err != nil {
		return fmt.Errorf("failed to return to Uber Tristram: %w", err)
	}

	return nil
}

func (t Torch) baal() error {
	t.ctx.Logger.Info("Starting Uber Diablo/Baal fight")
	t.ctx.DisableItemPickup()

	for _, p := range diaPath() {
		if err := action.MoveToCoords(p); err != nil {
			return fmt.Errorf("failed to reach coordinate: %w", err)
		}
		t.ctx.RefreshGameData()
	}

	if err := action.MoveToCoords(diaLurePos()); err != nil {
		return fmt.Errorf("failed to reach lure position: %w", err)
	}
	t.ctx.RefreshGameData()
	utils.Sleep(1000)

	if err := action.MoveToCoords(diaSafeFirst()); err != nil {
		return fmt.Errorf("failed to reach safe spot: %w", err)
	}
	t.ctx.RefreshGameData()
	utils.Sleep(1000)

	for _, p := range diaSafeSeq() {
		if err := action.MoveToCoords(p); err != nil {
			return fmt.Errorf("failed to reach safe spot: %w", err)
		}
		t.ctx.RefreshGameData()
		utils.Sleep(1000)
	}

	t.ctx.RefreshGameData()
	const maxDistance = 24
	const maxRetries = 3
	retryCount := 0

	for retryCount < maxRetries {
		diabloFound, _, diabloDistanceUnits := isUberDiabloNearby(t.ctx, maxDistance)
		baalFound, _, baalDistanceUnits := isUberBaalNearby(t.ctx, maxDistance)

		if diabloFound || baalFound {
			if diabloFound && baalFound {
				if diabloDistanceUnits <= baalDistanceUnits {
					t.ctx.Logger.Info("Found both bosses, killing Uber Diablo first (closer)")
					if err := t.ctx.Char.KillUberDiablo(); err != nil {
						return fmt.Errorf("failed to kill Uber Diablo: %w", err)
					}
					t.ctx.Logger.Info("Successfully killed Uber Diablo")
				} else {
					t.ctx.Logger.Info("Found both bosses, killing Uber Baal first (closer)")
					if err := t.ctx.Char.KillUberBaal(); err != nil {
						return fmt.Errorf("failed to kill Uber Baal: %w", err)
					}
					t.ctx.Logger.Info("Successfully killed Uber Baal")
				}
			} else if diabloFound {
				t.ctx.Logger.Info("Found Uber Diablo, starting fight")
				if err := t.ctx.Char.KillUberDiablo(); err != nil {
					return fmt.Errorf("failed to kill Uber Diablo: %w", err)
				}
				t.ctx.Logger.Info("Successfully killed Uber Diablo")
			} else if baalFound {
				t.ctx.Logger.Info("Found Uber Baal, starting fight")
				if err := t.ctx.Char.KillUberBaal(); err != nil {
					return fmt.Errorf("failed to kill Uber Baal: %w", err)
				}
				t.ctx.Logger.Info("Successfully killed Uber Baal")
			}
			break
		}

		retryCount++
		if retryCount >= maxRetries {
			diabloFoundAny, _, _ := isUberDiabloNearby(t.ctx, 999)
			baalFoundAny, _, _ := isUberBaalNearby(t.ctx, 999)

			if diabloFoundAny && baalFoundAny {
				t.ctx.Logger.Info("Found both bosses after retries, killing Uber Diablo")
				if err := t.ctx.Char.KillUberDiablo(); err != nil {
					return fmt.Errorf("failed to kill Uber Diablo: %w", err)
				}
				t.ctx.Logger.Info("Successfully killed Uber Diablo")
			} else if diabloFoundAny {
				t.ctx.Logger.Info("Found Uber Diablo after retries, starting fight")
				if err := t.ctx.Char.KillUberDiablo(); err != nil {
					return fmt.Errorf("failed to kill Uber Diablo: %w", err)
				}
				t.ctx.Logger.Info("Successfully killed Uber Diablo")
			} else if baalFoundAny {
				t.ctx.Logger.Info("Found Uber Baal after retries, starting fight")
				if err := t.ctx.Char.KillUberBaal(); err != nil {
					return fmt.Errorf("failed to kill Uber Baal: %w", err)
				}
				t.ctx.Logger.Info("Successfully killed Uber Baal")
			} else {
				t.ctx.Logger.Warn(fmt.Sprintf("Neither Uber Diablo nor Uber Baal found after %d retries", maxRetries))
				return fmt.Errorf("neither Uber Diablo nor Uber Baal found after %d retries", maxRetries)
			}
			break
		}

		if err := action.MoveToCoords(diaLurePos()); err != nil {
			return fmt.Errorf("failed to reach lure position on retry: %w", err)
		}
		t.ctx.RefreshGameData()
		utils.Sleep(500)

		if err := action.MoveToCoords(diaSafeFirst()); err != nil {
			return fmt.Errorf("failed to reach safe spot on retry: %w", err)
		}
		t.ctx.RefreshGameData()
		utils.Sleep(1200)
		t.ctx.RefreshGameData()
	}

	if err := t.townBaal(); err != nil {
		return fmt.Errorf("failed to return to town after Baal/Diablo: %w", err)
	}

	utils.Sleep(200)
	if err := vendorRefillOrHeal(t.ctx); err != nil {
		t.ctx.Logger.Warn(fmt.Sprintf("Failed to visit vendor/heal: %v", err))
	}

	action.ReviveMerc()

	if err := checkForRejuv(t.ctx); err != nil {
		t.ctx.Logger.Warn(fmt.Sprintf("Failed to check/fill rejuvs: %v", err))
	}

	if err := goToStashPosition(t.ctx); err != nil {
		return fmt.Errorf("failed to go to stash: %w", err)
	}

	if err := t.returnToUberTristram(); err != nil {
		return fmt.Errorf("failed to return to Uber Tristram: %w", err)
	}

	return nil
}

func (t *Torch) diablo() error {
	t.ctx.Logger.Info("Checking for remaining bosses in Uber Tristram")
	t.ctx.DisableItemPickup()

	t.ctx.RefreshGameData()

	if !isInUberTristram(t.ctx) {
		goToMalahIfInHarrogath(t.ctx)
		if err := action.VendorRefill(action.VendorRefillOpts{ForceRefill: true, SellJunk: true, BuyConsumables: true}); err != nil {
			t.ctx.Logger.Warn(fmt.Sprintf("Failed to visit vendor: %v", err))
		}
		action.ReviveMerc()

		if err := goToStashPosition(t.ctx); err != nil {
			return fmt.Errorf("failed to go to stash: %w", err)
		}

		if err := t.returnToUberTristram(); err != nil {
			return fmt.Errorf("failed to return to Uber Tristram: %w", err)
		}
		utils.Sleep(500)
		t.ctx.RefreshGameData()
		utils.Sleep(500)

		t.ctx.EnableItemPickup()
	}

	utils.Sleep(200)
	t.ctx.RefreshGameData()
	utils.Sleep(200)

	diabloAlive := false
	baalAlive := false
	var uberDiablo data.Monster
	var uberBaal data.Monster

	for _, m := range t.ctx.Data.Monsters.Enemies() {
		if m.Name == npc.UberDiablo && m.Stats[stat.Life] > 0 {
			diabloAlive = true
			uberDiablo = m
		}
		if m.Name == npc.UberBaal && m.Stats[stat.Life] > 0 {
			baalAlive = true
			uberBaal = m
		}
	}

	if baalAlive && !diabloAlive {
		t.ctx.Logger.Info("Only Uber Baal remaining, starting fight")
		if err := t.searchAndKillBoss(npc.UberBaal, "Baal"); err != nil {
			return err
		}
		if err := vendorRefillOrHeal(t.ctx); err != nil {
			t.ctx.Logger.Warn(fmt.Sprintf("Failed to visit vendor/heal: %v", err))
		}
		return t.townTorch()
	}

	if diabloAlive && !baalAlive {
		t.ctx.Logger.Info("Only Uber Diablo remaining, starting fight")
		if err := t.searchAndKillBoss(npc.UberDiablo, "Diablo"); err != nil {
			return err
		}
		if err := vendorRefillOrHeal(t.ctx); err != nil {
			t.ctx.Logger.Warn(fmt.Sprintf("Failed to visit vendor/heal: %v", err))
		}
		return t.townTorch()
	}

	if !diabloAlive && !baalAlive {
		t.ctx.Logger.Info("All bosses defeated, proceeding to torch search")
		if err := vendorRefillOrHeal(t.ctx); err != nil {
			t.ctx.Logger.Warn(fmt.Sprintf("Failed to visit vendor/heal: %v", err))
		}
		return t.townTorch()
	}

	if diabloAlive && baalAlive {
		t.ctx.Logger.Warn("Both bosses are still alive, this shouldn't happen. Killing the closer one...")
		diabloDistance := t.ctx.PathFinder.DistanceFromMe(uberDiablo.Position)
		baalDistance := t.ctx.PathFinder.DistanceFromMe(uberBaal.Position)

		if diabloDistance <= baalDistance {
			t.ctx.Logger.Info("Both bosses alive, killing Uber Diablo (closer)")
			if err := t.searchAndKillBoss(npc.UberDiablo, "Diablo"); err != nil {
				return err
			}
			if err := vendorRefillOrHeal(t.ctx); err != nil {
				t.ctx.Logger.Warn(fmt.Sprintf("Failed to visit vendor/heal: %v", err))
			}
			return t.townTorch()
		}
		t.ctx.Logger.Info("Both bosses alive, killing Uber Baal (closer)")
		if err := t.searchAndKillBoss(npc.UberBaal, "Baal"); err != nil {
			return err
		}
		if err := vendorRefillOrHeal(t.ctx); err != nil {
			t.ctx.Logger.Warn(fmt.Sprintf("Failed to visit vendor/heal: %v", err))
		}
		return t.townTorch()
	}

	return nil
}

func (t *Torch) searchAndKillBoss(bossNPC npc.ID, bossName string) error {
	exploreFilter := func(m data.Monsters) []data.Monster {
		return []data.Monster{}
	}

	err := action.ClearCurrentLevelEx(false, exploreFilter, func() bool {
		t.ctx.RefreshGameData()
		if boss, found := t.ctx.Data.Monsters.FindOne(bossNPC, data.MonsterTypeUnique); found {
			if boss.Stats[stat.Life] > 0 {
				if err := action.MoveToCoords(boss.Position, step.WithIgnoreMonsters()); err != nil {
					t.ctx.Logger.Warn(fmt.Sprintf("Failed to teleport to boss: %v", err))
				}
				return true
			}
		}
		return false
	})
	if err != nil {
		return fmt.Errorf("failed to search for Uber %s: %w", bossName, err)
	}

	t.ctx.RefreshGameData()
	if boss, found := t.ctx.Data.Monsters.FindOne(bossNPC, data.MonsterTypeUnique); found && boss.Stats[stat.Life] > 0 {
		t.ctx.Logger.Info(fmt.Sprintf("Found Uber %s, starting fight", bossName))
		if bossNPC == npc.UberDiablo {
			if err := t.ctx.Char.KillUberDiablo(); err != nil {
				return fmt.Errorf("failed to kill Uber Diablo: %w", err)
			}
			t.ctx.Logger.Info("Successfully killed Uber Diablo")
		} else if bossNPC == npc.UberBaal {
			if err := t.ctx.Char.KillUberBaal(); err != nil {
				return fmt.Errorf("failed to kill Uber Baal: %w", err)
			}
			t.ctx.Logger.Info("Successfully killed Uber Baal")
		}
	} else {
		t.ctx.Logger.Warn(fmt.Sprintf("Uber %s not found after search", bossName))
		return fmt.Errorf("uber %s not found after search", bossName)
	}

	return nil
}

func (t *Torch) townTorch() error {
	path := []data.Position{portalPos()}
	if err := enterTownFromPortal(t.ctx, path); err != nil {
		return err
	}

	goToMalahIfInHarrogath(t.ctx)
	if err := action.VendorRefill(action.VendorRefillOpts{ForceRefill: true, SellJunk: true, BuyConsumables: true}); err != nil {
		t.ctx.Logger.Warn(fmt.Sprintf("Failed to visit vendor: %v", err))
	}
	action.ReviveMerc()

	if err := goToStashPosition(t.ctx); err != nil {
		return fmt.Errorf("failed to go to stash: %w", err)
	}

	if err := t.safeTorch(); err != nil {
		return fmt.Errorf("failed to safe torch: %w", err)
	}

	if err := t.returnToUberTristram(); err != nil {
		return fmt.Errorf("failed to return to Uber Tristram: %w", err)
	}

	if err := t.searchForTorch(); err != nil {
		return fmt.Errorf("failed to search for torch: %w", err)
	}

	return t.restoreTorch()
}

func (t *Torch) safeTorch() error {
	if err := openStash(t.ctx); err != nil {
		return fmt.Errorf("failed to open stash: %w", err)
	}
	utils.Sleep(300)
	t.ctx.RefreshGameData()

	torchItem, found := findTorchInInventory(t.ctx, 0)
	if !found {
		t.ctx.Logger.Warn("No unique large charm (torch) found in inventory - may already be stashed or not present")
		return nil
	}

	t.savedTorchInventoryPos = torchItem.Position
	t.savedTorchUnitID = torchItem.UnitID

	tab, err := stashToSharedStash(t.ctx, torchItem)
	if err != nil {
		return err
	}

	t.savedTorchStashTab = tab

	step.CloseAllMenus()
	utils.Sleep(200)
	t.ctx.RefreshGameData()

	return nil
}

func (t *Torch) searchForTorch() error {
	t.ctx.RefreshGameData()
	for _, invItem := range t.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if (invItem.Name == "LargeCharm" && invItem.Quality == item.QualityUnique) ||
			invItem.IdentifiedName == "Hellfire Torch" {
			t.newTorchUnitID = invItem.UnitID
			return nil
		}
	}

	if err := action.MoveToCoords(torchMiddlePos(), step.WithIgnoreMonsters()); err != nil {
		t.ctx.Logger.Warn(fmt.Sprintf("Failed to teleport to middle coordinate: %v", err))
	} else {
		utils.Sleep(500)
		t.ctx.RefreshGameData()
	}

	exploreFilter := func(m data.Monsters) []data.Monster {
		return []data.Monster{}
	}

	t.ctx.EnableItemPickup()

	torchFound := false
	err := action.ClearCurrentLevelEx(false, exploreFilter, func() bool {
		t.ctx.RefreshGameData()

		for _, invItem := range t.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
			if (invItem.Name == "LargeCharm" && invItem.Quality == item.QualityUnique) ||
				invItem.IdentifiedName == "Hellfire Torch" {
				t.newTorchUnitID = invItem.UnitID
				torchFound = true
				return true
			}
		}

		for _, itm := range t.ctx.Data.Inventory.ByLocation(item.LocationGround) {
			if (itm.Name == "largecharm" || itm.Name == "LargeCharm") && itm.Quality == item.QualityUnique {
				t.newTorchUnitID = itm.UnitID

				if err := action.ItemPickup(0); err != nil {
					t.ctx.Logger.Warn(fmt.Sprintf("ItemPickup returned error (may be normal): %v", err))
				}

				utils.Sleep(2000)
				t.ctx.RefreshGameData()

				for _, invItem := range t.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
					if invItem.UnitID == itm.UnitID {
						torchFound = true
						return true
					}
				}
			}
		}

		return false
	})

	if err != nil {
		return fmt.Errorf("failed to search for torch: %w", err)
	}

	if !torchFound {
		t.ctx.Logger.Warn("torch not found after exploration - it may have already been picked up or not dropped")
	}

	return nil
}

func (t *Torch) restoreTorch() error {
	path := []data.Position{portalPos()}
	if err := enterTownFromPortal(t.ctx, path); err != nil {
		return err
	}

	if err := goToStashPosition(t.ctx); err != nil {
		return fmt.Errorf("failed to go to stash: %w", err)
	}

	if err := openStash(t.ctx); err != nil {
		return fmt.Errorf("failed to open stash: %w", err)
	}
	utils.Sleep(300)
	t.ctx.RefreshGameData()

	var newTorch data.Item
	newTorchFound := false

	if t.newTorchUnitID != 0 {
		for _, invItem := range t.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
			if invItem.UnitID == t.newTorchUnitID {
				newTorch = invItem
				newTorchFound = true
				break
			}
		}
	}

	if !newTorchFound {
		newTorch, newTorchFound = findTorchInInventory(t.ctx, t.savedTorchUnitID)
		if newTorchFound {
			t.newTorchUnitID = newTorch.UnitID
		}
	}

	if newTorchFound {
		if err := standardofHeros(t.ctx); err != nil {
			t.ctx.Logger.Warn(fmt.Sprintf("Failed to stash Standard of Heroes: %v", err))
		}

		if _, err := stashToSharedStash(t.ctx, newTorch); err != nil {
			t.ctx.Logger.Warn(fmt.Sprintf("Failed to stash new torch: %v", err))
		}
	} else {
		t.ctx.Logger.Warn("New torch not found in inventory - may have already been stashed or not picked up")
	}

	if t.savedTorchUnitID != 0 && t.savedTorchStashTab > 0 {
		oldTorch, restored := restoreFromSharedStash(t.ctx, t.savedTorchUnitID, t.savedTorchStashTab)
		if restored {
			if err := moveItemToPosition(t.ctx, oldTorch, t.savedTorchInventoryPos); err != nil {
				t.ctx.Logger.Warn(fmt.Sprintf("Failed to move torch to original position: %v", err))
			}
		} else {
			t.ctx.Logger.Warn("Old torch not found in shared stash - may have been moved or removed")
		}
	}

	step.CloseAllMenus()
	utils.Sleep(200)
	t.ctx.RefreshGameData()

	return nil
}

func (t Torch) town() error {
	path := []data.Position{mephC2, mephC1, portalPos()}
	if err := enterTownFromPortal(t.ctx, path); err != nil {
		return err
	}

	return nil
}

func (t Torch) townBaal() error {
	path := []data.Position{diaC1, portalPos()}
	if err := enterTownFromPortal(t.ctx, path); err != nil {
		return err
	}

	return nil
}

func (t Torch) returnToUberTristram() error {
	return returnToUberTristram(t.ctx, t.findPortal, t.enterPortalAndStop)
}
