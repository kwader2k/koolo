package astar

import (
	"encoding/gob"
	"os"
	"testing"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/koolo/internal/game"
)

func BenchmarkAstar(b *testing.B) {
	grid := loadGrid()

	start := data.Position{X: 336, Y: 701}
	goal := data.Position{X: 11, Y: 330}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		CalculatePath(grid, start, goal, false, nil)
	}
}

func BenchmarkAstarWithBuffers(b *testing.B) {
	grid := loadGrid()

	start := data.Position{X: 336, Y: 701}
	goal := data.Position{X: 11, Y: 330}
	buffers := &AStarBuffers{}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		CalculatePath(grid, start, goal, false, buffers)
	}
}

func TestAstar(t *testing.T) {
	grid := loadGrid()

	start := data.Position{X: 336, Y: 701}
	goal := data.Position{X: 11, Y: 330}

	p, dist, found := CalculatePath(grid, start, goal, false, nil)

	if !found {
		t.Fatalf("Expected path to be found")
	}

	// Verify path starts at start and ends at goal
	if len(p) == 0 {
		t.Fatalf("Path is empty")
	}
	if p[0] != start {
		t.Errorf("Path should start at %v, got %v", start, p[0])
	}
	if p[len(p)-1] != goal {
		t.Errorf("Path should end at %v, got %v", goal, p[len(p)-1])
	}

	// Path length should be reasonable (original was 546, allow some variance)
	if dist < 400 || dist > 700 {
		t.Errorf("Path distance %d seems unreasonable (expected ~546)", dist)
	}

	t.Logf("Path found: length=%d, dist=%d", len(p), dist)
}

// legacyGrid represents the old Grid format with 2D CollisionGrid
type legacyGrid struct {
	OffsetX       int
	OffsetY       int
	Width         int
	Height        int
	CollisionGrid [][]game.CollisionType
}

func loadGrid() *game.Grid {
	var legacy legacyGrid
	file, err := os.Open("durance_of_hate_grid.bin")
	if err != nil {
		panic(err)
	}
	defer file.Close()

	decoder := gob.NewDecoder(file)
	if err := decoder.Decode(&legacy); err != nil {
		panic(err)
	}

	// Convert 2D to flat 1D
	flat := make([]game.CollisionType, legacy.Width*legacy.Height)
	for y := 0; y < legacy.Height; y++ {
		copy(flat[y*legacy.Width:(y+1)*legacy.Width], legacy.CollisionGrid[y])
	}

	return &game.Grid{
		OffsetX:       legacy.OffsetX,
		OffsetY:       legacy.OffsetY,
		Width:         legacy.Width,
		Height:        legacy.Height,
		CollisionGrid: flat,
	}
}
