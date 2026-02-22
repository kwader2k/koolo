package action

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const malusScoreIndividualSlots = -0.1
const bonusScorePerSameSurface = 1.0
const narrowLinesMultiplier = 0.2
const narrowColumnMultiplier = 0.5
const HeightRatioBonusMultiplier = 1.0

type InventoryMask struct {
	Width, Height int
	Grid          [][]bool
}

func NewInventoryMask(width, height int) *InventoryMask {
	grid := make([][]bool, height)
	for y := 0; y < height; y++ {
		grid[y] = make([]bool, width)
	}
	return &InventoryMask{Width: width, Height: height, Grid: grid}
}

func (inv *InventoryMask) Clear() {
	for y := range inv.Grid {
		for x := range inv.Grid[y] {
			inv.Grid[y][x] = false
		}
	}
}

func (inv *InventoryMask) CanPlace(x, y, w, h int) bool {
	if x+w > inv.Width || y+h > inv.Height {
		return false
	}
	for j := 0; j < h; j++ {
		for i := 0; i < w; i++ {
			if inv.Grid[y+j][x+i] {
				return false
			}
		}
	}
	return true
}

func (inv *InventoryMask) Place(x, y, w, h int) {
	for j := 0; j < h; j++ {
		for i := 0; i < w; i++ {
			inv.Grid[y+j][x+i] = true
		}
	}
}

func (inv *InventoryMask) Remove(x, y, w, h int) {
	for j := 0; j < h; j++ {
		for i := 0; i < w; i++ {
			inv.Grid[y+j][x+i] = false
		}
	}
}

func (inv *InventoryMask) isInBounds(x, y int) bool {
	return x >= 0 && x < inv.Width && y >= 0 && y < inv.Height
}

func (inv *InventoryMask) isMalus(x, y int) bool {
	return inv.isInBounds(x, y) && inv.Grid[y][x]
}

func (inv *InventoryMask) computeMalus(x, y int) float64 {
	malus := 0
	if inv.isMalus(x, y-1) {
		malus++
	}
	if inv.isMalus(x, y+1) {
		malus++
	}
	if inv.isMalus(x-1, y) {
		malus++
	}
	if inv.isMalus(x+1, y) {
		malus++
	}
	if malus > 2 {
		malus = malus * 4
	} else if malus == 2 &&
		(((x == 0 || x == inv.Width-1) && y != 0 && y != inv.Height-1) || ((y == 0 || y == inv.Height-1) && x != 0 && x != inv.Width-1)) {
		malus = malus * 2
	} else {
		malus = 0
	}
	return float64(malus) * malusScoreIndividualSlots
}

func (inv *InventoryMask) LargestFreeRectangleScore() (area, w, h int, score float64) {
	hist := make([]int, inv.Width)
	maxScore := 0.0
	bestArea, bestW, bestH := 0, 0, 0
	sameScoreCount := 0
	scoreMalus := 0.0
	for y := 0; y < inv.Height; y++ {
		for x := 0; x < inv.Width; x++ {
			if !inv.Grid[y][x] {
				hist[x]++
				scoreMalus += inv.computeMalus(x, y)
			} else {
				hist[x] = 0
			}
		}
		a, ww, hh, s := largestRectangleInHistogramWeighted(hist, inv.Height)
		if s > maxScore {
			maxScore = s
			bestArea, bestW, bestH = a, ww, hh
			sameScoreCount = 1
		} else if s == maxScore {
			sameScoreCount++
		}
	}

	//If moving item allows several spaces of same score, bump score a bit
	if sameScoreCount > 1 {
		maxScore += bonusScorePerSameSurface * (float64(sameScoreCount) - 1.0)
	}
	maxScore += scoreMalus
	return bestArea, bestW, bestH, maxScore
}

// Modified histogram solver: prioritize height over width
func largestRectangleInHistogramWeighted(heights []int, maxHeight int) (area, w, h int, score float64) {
	stack := []int{}
	maxScore := 0.0
	bestArea, bestW, bestH := 0, 0, 0

	for i := 0; i <= len(heights); i++ {
		hh := 0
		if i < len(heights) {
			hh = heights[i]
		}
		for len(stack) > 0 && (i == len(heights) || hh < heights[stack[len(stack)-1]]) {
			height := heights[stack[len(stack)-1]]
			stack = stack[:len(stack)-1]
			width := i
			if len(stack) > 0 {
				width = i - stack[len(stack)-1] - 1
			}
			a := height * width
			var s float64

			// Apply prioritization rules
			// Priority: full-height rectangle
			switch {
			case width <= 1:
				s = float64(a) * narrowColumnMultiplier
			case height == maxHeight:
				s = float64(a) * 3.0
			case height >= maxHeight-1:
				s = float64(a) * 1.5
			case width > height && height < maxHeight-1:
				s = float64(a) * narrowLinesMultiplier
			default:
				// Normal case: bias toward taller rectangles
				heightBias := float64(height) / float64(width)
				s = float64(a) * (1.0 + HeightRatioBonusMultiplier*heightBias)
			}

			if s > maxScore {
				maxScore = s
				bestArea, bestW, bestH = a, width, height
			}
		}
		stack = append(stack, i)
	}
	return bestArea, bestW, bestH, maxScore
}

func (inv *InventoryMask) findBestItemPlacement(items []data.Item) (bool, data.Item, data.Position) {
	needMove := false
	bestItem := data.Item{}
	itemPosition := data.Position{}
	bestItemScore := 0.0
	itemBetterPositionned := false

	for idx := range items {
		item := &items[idx]
		w, h := item.Desc().InventoryWidth, item.Desc().InventoryHeight
		x, y := item.Position.X, item.Position.Y

		// Skip invalid & locked positions
		if x < 0 || y < 0 || IsInLockedInventorySlot(*item) {
			continue
		}

		bestPosition := item.Position
		_, _, _, currentScore := inv.LargestFreeRectangleScore()
		bestScore := currentScore
		betterPositioning := false

		//All items should favor right placement except tomes
		placeRight := item.Name != "TomeOfTownPortal" && item.Name != "TomeOfIdentify"

		// Remove item from mask temporarily
		inv.Remove(x, y, w, h)

		// Try all possible new positions
		for nx := 0; nx < inv.Width; nx++ {
			for ny := 0; ny < inv.Height; ny++ {
				tempPos := data.Position{X: nx, Y: ny}
				if !utils.IsSamePosition(item.Position, tempPos) && inv.CanPlace(nx, ny, w, h) {
					inv.Place(nx, ny, w, h)
					_, _, _, s := inv.LargestFreeRectangleScore()
					inv.Remove(nx, ny, w, h)

					betterPosition := (placeRight && nx > item.Position.X) || (!placeRight && nx < item.Position.X)

					// use this item only if it improves inventory score, or if it organises the inventory better
					if s > bestScore ||
						(s >= bestScore && betterPosition) {
						bestScore = s
						bestPosition.X, bestPosition.Y = nx, ny
						betterPositioning = betterPosition
					}
				}
			}
		}

		// replace item in mask
		inv.Place(x, y, w, h)

		if !utils.IsSamePosition(bestPosition, item.Position) && (bestScore > bestItemScore || (bestScore >= bestItemScore && betterPositioning && !itemBetterPositionned)) {
			bestItemScore = bestScore
			itemBetterPositionned = betterPositioning
			bestItem = *item
			itemPosition = bestPosition
			needMove = true
		}
	}

	return needMove, bestItem, itemPosition
}

func OptimizeInventory(location item.LocationType) error {
	ctx := context.Get()
	width := 10
	height := 4
	switch location {
	case item.LocationStash, item.LocationSharedStash:
		height = 10
	}

	inv := NewInventoryMask(width, height)
	items := ctx.Data.Inventory.ByLocation(location)

	// mark all current item positions as occupied in mask
	for _, item := range items {
		w, h := item.Desc().InventoryWidth, item.Desc().InventoryHeight
		x, y := item.Position.X, item.Position.Y
		if x >= 0 && y >= 0 && inv.CanPlace(x, y, w, h) {
			inv.Place(x, y, w, h)
		}
	}

	needContinue := true
	maxIterations := 30
	iterations := 0
	for needContinue && iterations < maxIterations {
		iterations++
		ctx.PauseIfNotPriority()
		ctx.RefreshGameData()
		utils.Sleep(200)
		items = ctx.Data.Inventory.ByLocation(location)

		//Find best item to reorganise
		betterFound, itm, position := inv.findBestItemPlacement(items)
		if betterFound {

			//Open inventory location if needed
			if !ctx.Data.OpenMenus.Inventory {
				switch location {
				case item.LocationStash, item.LocationSharedStash:
					err := OpenStash()
					if err != nil {
						return err
					}
				case item.LocationInventory:
					ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
				}
			}

			//Move object...
			w := itm.Desc().InventoryWidth
			h := itm.Desc().InventoryHeight

			//...From mask...
			inv.Remove(itm.Position.X, itm.Position.Y, w, h)
			inv.Place(position.X, position.Y, w, h)

			//..Then inventory location
			screenPos := ui.GetScreenCoordsForItem(itm)
			ctx.HID.Click(game.LeftButton, screenPos.X, screenPos.Y)
			utils.PingSleep(utils.Medium, 500)
			newScreenPos := ui.GetScreenCoordsForInventoryPosition(position, location)
			ctx.HID.Click(game.LeftButton, newScreenPos.X, newScreenPos.Y)
			utils.PingSleep(utils.Medium, 500)
		} else {
			//No more items to reorganise, we're done
			needContinue = false
		}
	}

	//_, w, h, s := inv.LargestFreeRectangleScore()
	//fmt.Printf("Optimization complete. Best free rectangle: %dx%d (score %.2f)\n", w, h, s)

	step.CloseAllMenus()

	// if something is left on cursor, drop it and pick it up again
	DropAndRecoverCursorItem()

	return nil
}
