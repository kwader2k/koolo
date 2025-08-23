package action

import (
	"log/slog"
	"strings"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/health"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	PotionMoveDelayMS = 300
)

func ManageBelt() error {
	ctx := context.Get()
	ctx.SetLastAction("ManageBelt")

	// New: Fill belt from inventory before anything else
	if err := fillBeltFromInventory(); err != nil {
		slog.Error("Could not fill belt from inventory", "error", err)
	}

	// Check for misplaced potions
	misplacedPotions := checkMisplacedPotions()

	// Consume misplaced potions
	for len(misplacedPotions) > 0 {
		for _, potion := range misplacedPotions {
			slog.Info("Consuming misplaced potion", "potion", potion.Name, "position", potion.Position)
			ctx.HID.PressKey(ctx.Data.KeyBindings.UseBelt[potion.Position.X].Key1[0])
			time.Sleep(500 * time.Millisecond)
		}

		misplacedPotions = checkMisplacedPotions()
		if len(misplacedPotions) == 0 {
			break
		}
	}

	return nil
}

func fillBeltFromInventory() error {
	ctx := context.Get()
	bm := health.NewBeltManager(ctx.Data, ctx.HID, ctx.Logger, "supervisor")

	if !bm.ShouldBuyPotions() {
		return nil // Belt is full enough, no need to do anything.
	}

	potionsInInventory := findPotionsInInventory()
	if len(potionsInInventory) == 0 {
		return nil // No potions in inventory, vendor will be triggered later if needed.
	}

	slog.Info("Found potions in inventory, moving to belt.")
	if !ctx.Data.OpenMenus.Inventory {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		utils.Sleep(PotionMoveDelayMS)
	}
	defer step.CloseAllMenus()

	for _, potion := range potionsInInventory {
		if !bm.ShouldBuyPotions() {
			break // Stop if belt is full enough
		}

		movePotionToBelt(potion)
		*ctx.Data = ctx.GameReader.GetData() // Refresh data after moving a potion
	}

	return nil
}

func findPotionsInInventory() []data.Item {
	ctx := context.Get()
	var potions []data.Item
	for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		itemName := strings.ToLower(string(itm.Name))
		if strings.Contains(itemName, "healing") || strings.Contains(itemName, "mana") || strings.Contains(itemName, "rejuvenation") {
			potions = append(potions, itm)
		}
	}
	return potions
}

func movePotionToBelt(potion data.Item) {
	ctx := context.Get()
	itemCoords := ui.GetScreenCoordsForItem(potion)
	ctx.HID.ClickWithModifier(game.LeftButton, itemCoords.X, itemCoords.Y, game.ShiftKey)
	utils.Sleep(PotionMoveDelayMS)
}

func checkMisplacedPotions() []data.Item {
	ctx := context.Get()
	ctx.SetLastAction("CheckMisplacedPotions")

	// Get list of potions in the first row
	potions := []data.Item{}

	for _, pot := range ctx.Data.Inventory.Belt.Items {
		if pot.Position.Y == 0 && (pot.Position.X == 0 || pot.Position.X == 1 || pot.Position.X == 2 || pot.Position.X == 3) {
			potions = append(potions, pot)
		}
	}

	expectedPotions := ctx.Data.CharacterCfg.Inventory.BeltColumns

	// Count occurrences of each potion type
	expectedCounts := make(map[string]int)
	for _, potion := range expectedPotions {
		expectedCounts[potion]++
	}

	// Helper function to match potion types
	matchPotionType := func(actualPotion, expectedType string) bool {
		return strings.Contains(strings.ToLower(actualPotion), expectedType)
	}

	// Check for misplaced potions
	misplacedPotions := []data.Item{}

	for _, potion := range potions {
		matched := false
		for expectedType, count := range expectedCounts {
			if matchPotionType(string(potion.Name), expectedType) {
				if count > 0 {
					expectedCounts[expectedType]--
				} else {
					misplacedPotions = append(misplacedPotions, potion)
				}
				matched = true
				break
			}
		}
		if !matched {
			misplacedPotions = append(misplacedPotions, potion)
		}
	}

	return misplacedPotions
}