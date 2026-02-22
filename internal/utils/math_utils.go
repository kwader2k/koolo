package utils

import (
	"math"

	"github.com/hectorgimenez/d2go/pkg/data"
)

type Vector struct {
	X float64
	Y float64
}

// calculateDistance returns the Euclidean distance between two positions.
func CalculateDistance(p1, p2 data.Position) float64 {
	dx := float64(p1.X - p2.X)
	dy := float64(p1.Y - p2.Y)
	return math.Sqrt(dx*dx + dy*dy)
}

func IsZeroPosition(v data.Position) bool {
	return v.X == 0 && v.Y == 0
}

func IsSamePosition(v1, v2 data.Position) bool {
	return v1.X == v2.X && v1.Y == v2.Y
}

func PositionAddCoords(v data.Position, x int, y int) data.Position {
	res := v
	res.X += x
	res.Y += y
	return res
}

func PositionAdd(v1, v2 data.Position) data.Position {
	var res = v1
	res.X += v2.X
	res.Y += v2.Y
	return res
}

func PositionSubCoords(v data.Position, x int, y int) data.Position {
	res := v
	res.X -= x
	res.Y -= y
	return res
}

func PositionSub(v1, v2 data.Position) data.Position {
	var res = v1
	res.X -= v2.X
	res.Y -= v2.Y
	return res
}

func PositionMultiply(v data.Position, m int) data.Position {
	res := v
	res.X *= m
	res.Y *= m
	return res
}

func PositionDivide(v data.Position, d int) data.Position {
	res := v
	res.X /= d
	res.Y /= d
	return res
}

func PositionToVector(p data.Position) Vector {
	var v Vector
	v.X = float64(p.X)
	v.Y = float64(p.Y)
	return v
}

func VectorAddCoords(v Vector, x float64, y float64) Vector {
	res := v
	res.X += x
	res.Y += y
	return res
}

func VectorAdd(v1, v2 Vector) Vector {
	var res = v1
	res.X += v2.X
	res.Y += v2.Y
	return res
}

func VectorSubCoords(v Vector, x float64, y float64) Vector {
	res := v
	res.X -= x
	res.Y -= y
	return res
}

func VectorSub(v1, v2 Vector) Vector {
	var res = v1
	res.X -= v2.X
	res.Y -= v2.Y
	return res
}

func VectorMultiply(v Vector, m float64) Vector {
	res := v
	res.X *= m
	res.Y *= m
	return res
}

func VectorDivide(v Vector, d float64) Vector {
	res := v
	res.X /= d
	res.Y /= d
	return res
}

func VectorToPosition(v Vector) data.Position {
	var p data.Position
	p.X = int(math.Round(v.X))
	p.Y = int(math.Round(v.Y))
	return p
}
