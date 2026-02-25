package pather

import (
	"testing"

	"github.com/hectorgimenez/d2go/pkg/data"
)

func TestEnsurePathReachability(t *testing.T) {
	tests := []struct {
		name        string
		inputPath   Path
		expectedLen int
		checkFunc   func(Path) bool
	}{
		{
			name:        "Empty path",
			inputPath:   Path{},
			expectedLen: 0,
			checkFunc: func(p Path) bool {
				return len(p) == 0
			},
		},
		{
			name:        "Single point",
			inputPath:   Path{data.Position{X: 0, Y: 0}},
			expectedLen: 1,
			checkFunc: func(p Path) bool {
				return len(p) == 1
			},
		},
		{
			name:        "Two points within range",
			inputPath:   Path{data.Position{X: 0, Y: 0}, data.Position{X: 5, Y: 5}},
			expectedLen: 2,
			checkFunc: func(p Path) bool {
				return len(p) == 2
			},
		},
		{
			name:        "Two points exactly at max distance",
			inputPath:   Path{data.Position{X: 0, Y: 0}, data.Position{X: 7, Y: 0}},
			expectedLen: 2,
			checkFunc: func(p Path) bool {
				return len(p) == 2
			},
		},
		{
			name:        "Two points exceeding max distance",
			inputPath:   Path{data.Position{X: 0, Y: 0}, data.Position{X: 20, Y: 0}},
			expectedLen: 4, // 0,0 -> 7,0 -> 14,0 -> 20,0
			checkFunc: func(p Path) bool {
				// Should have intermediate waypoints
				return len(p) > 2
			},
		},
		{
			name:        "Two points diagonal exceeding max distance",
			inputPath:   Path{data.Position{X: 0, Y: 0}, data.Position{X: 15, Y: 15}},
			expectedLen: 4, // 0,0 -> 7,7 -> 14,14 -> 15,15 (being conservative with intermediate points)
			checkFunc: func(p Path) bool {
				return len(p) > 2
			},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := ensurePathReachability(tt.inputPath)
			if len(result) != tt.expectedLen {
				t.Errorf("Expected path length %d, got %d", tt.expectedLen, len(result))
			}
			if !tt.checkFunc(result) {
				t.Errorf("Check function failed for path: %v", result)
			}

			// Verify all consecutive points are within MaxWaypointDistance units (Chebyshev distance)
			for i := 1; i < len(result); i++ {
				prev := result[i-1]
				curr := result[i]

				dx := curr.X - prev.X
				if dx < 0 {
					dx = -dx
				}
				dy := curr.Y - prev.Y
				if dy < 0 {
					dy = -dy
				}

				dist := dx
				if dy > dist {
					dist = dy
				}

				if dist > MaxWaypointDistance {
					t.Errorf("Distance between consecutive points exceeds MaxWaypointDistance (%d): %v to %v (distance=%d)", MaxWaypointDistance, prev, curr, dist)
				}
			}
		})
	}
}

func TestEnsurePathReachabilityComplexPath(t *testing.T) {
	// Create a path with multiple segments, some exceeding the limit
	path := Path{
		data.Position{X: 0, Y: 0},
		data.Position{X: 25, Y: 25}, // Far away, needs interpolation
		data.Position{X: 30, Y: 25}, // Close to previous
		data.Position{X: 50, Y: 50}, // Far away, needs interpolation
	}

	result := ensurePathReachability(path)

	// Verify all consecutive points are within MaxWaypointDistance units
	for i := 1; i < len(result); i++ {
		prev := result[i-1]
		curr := result[i]

		dx := curr.X - prev.X
		if dx < 0 {
			dx = -dx
		}
		dy := curr.Y - prev.Y
		if dy < 0 {
			dy = -dy
		}

		dist := dx
		if dy > dist {
			dist = dy
		}

		if dist > MaxWaypointDistance {
			t.Errorf("Distance between consecutive points exceeds MaxWaypointDistance (%d): %v to %v (distance=%d)", MaxWaypointDistance, prev, curr, dist)
		}
	}

	// Start and end points should be preserved
	if result[0] != path[0] {
		t.Errorf("Start point should be preserved: got %v, expected %v", result[0], path[0])
	}
	if result[len(result)-1] != path[len(path)-1] {
		t.Errorf("End point should be preserved: got %v, expected %v", result[len(result)-1], path[len(path)-1])
	}

	t.Logf("Complex path validation passed. Original path length: %d, Result path length: %d", len(path), len(result))
}
