package action

import (
	"sort"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

var (
	beltSlotsResurrected = buildBeltSlotCoords(ui.BeltOriginX, ui.BeltOriginY, ui.BeltOffsetPerColX, ui.BeltOffsetPerRowY)
	beltSlotsClassic     = buildBeltSlotCoords(ui.BeltOriginClassicX, ui.BeltOriginClassicY, ui.BeltClassicOffsetPerColX, ui.BeltClassicOffsetPerRowY)
)

func isLowGold() bool {
	// this is redefined, a bigger restructure is needed but out of scope for this change
	ctx := context.Get()

	var playerLevel int
	if lvl, found := ctx.Data.PlayerUnit.FindStat(stat.Level, 0); found {
		playerLevel = lvl.Value
	} else {
		playerLevel = 1
	}

	return ctx.Data.PlayerUnit.TotalPlayerGold() < playerLevel*1000
}

func ManageBelt() error {
	ctx := context.Get()
	ctx.SetLastAction("ManageBelt")

	misplacedPotions := checkMisplacedPotions()
	if len(misplacedPotions) == 0 {
		ctx.Logger.Info("No misplaced potions found in belt")
		return nil
	}

	if err := openInventoryIfNeeded(); err != nil {
		ctx.Logger.Error("Failed to open inventory")
		return err
	}

	wasBeltVisible := ctx.Data.OpenMenus.BeltRows
	if !wasBeltVisible {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.ShowBelt)
		utils.Sleep(150)
	}

	for _, potion := range misplacedPotions {
		utils.Sleep(100)
		screenPos, ok := beltPositionToScreenCoords(ctx, potion.Position)
		if !ok {
			ctx.Logger.Warn("Unable to translate belt position to screen coords", "position", potion.Position)
			continue
		}

		// Just consume the potion if there is no space in inventory for the potion
		if !hasInventorySpaceForPotion(potion) {
			ctx.HID.Click(game.RightButton, screenPos.X, screenPos.Y)
			continue
		}

		ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.ShiftKey)
	}

	ctx.RefreshGameData()
	step.CloseAllMenus()
	return nil
}

func checkMisplacedPotions() []data.Item {
	ctx := context.Get()
	ctx.SetLastAction("CheckMisplacedPotions")

	configuredBeltColumns := ctx.Data.CharacterCfg.Inventory.BeltColumns

	if len(configuredBeltColumns) != 4 {
		return nil
	}

	rows := ctx.Data.Inventory.Belt.Rows()

	matchPotionType := func(actualPotion, expectedType string) bool {
		expectedType = strings.ToLower(expectedType)
		if expectedType == "" {
			return false
		}
		return strings.Contains(strings.ToLower(actualPotion), expectedType)
	}

	misplacedPotions := []data.Item{}
	for _, pot := range ctx.Data.Inventory.Belt.Items {
		if pot.Position.X < 0 || pot.Position.X >= (rows*4) {
			ctx.Logger.Warn("Potion in invalid belt position", "potion", pot.Name, "position", pot.Position)
			continue
		}

		expectedPotionType := configuredBeltColumns[pot.Position.X%4]
		if !matchPotionType(string(pot.Name), expectedPotionType) {
			misplacedPotions = append(misplacedPotions, pot)
		}
	}
	// need import "sort"
	sort.Slice(misplacedPotions, func(i, j int) bool {
		return misplacedPotions[i].Position.X > misplacedPotions[j].Position.X
	})
	// or A way to avoid the import "sort"
	// n := len(misplacedPotions)
	// for i := 0; i < n; i++ {
	// 	for j := 0; j < n-i-1; j++ {
	// 		if misplacedPotions[j].Position.X < misplacedPotions[j+1].Position.X {
	// 			misplacedPotions[j], misplacedPotions[j+1] = misplacedPotions[j+1], misplacedPotions[j]
	// 		}
	// 	}
	// }

	//debug
	// for i, pot := range misplacedPotions {
	// 	ctx.Logger.Warn(fmt.Sprintf("Misplaced Potion #%d: %s at %v", i, pot.Name, pot.Position))
	// }
	return misplacedPotions
}

func openInventoryIfNeeded() error {
	ctx := context.Get()
	if ctx.Data.OpenMenus.Inventory {
		return nil
	}
	return step.OpenInventory()
}

func beltPositionToScreenCoords(ctx *context.Status, position data.Position) (data.Position, bool) {
	slotIndex := position.X
	if slotIndex < 0 || slotIndex >= 16 {
		return data.Position{}, false
	}

	if ctx.Data.LegacyGraphics {
		return beltSlotsClassic[slotIndex], true
	}
	return beltSlotsResurrected[slotIndex], true
}

func hasInventorySpaceForPotion(potion data.Item) bool {
	_, ok := findInventorySpace(potion)
	return ok
}

func buildBeltSlotCoords(originX, originY, offsetX, offsetY int) [16]data.Position {
	var slots [16]data.Position
	idx := 0
	for row := 0; row < 4; row++ {
		for col := 0; col < 4; col++ {
			slots[idx] = data.Position{
				X: originX + col*offsetX,
				Y: originY - row*offsetY,
			}
			idx++
		}
	}
	return slots
}
