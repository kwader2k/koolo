package utils

import (
	"math"
	"math/rand"
	"sync"
	"time"
)

// session fatigue state — reset at the start of each play session.
var (
	sessionMu    sync.RWMutex
	sessionStart time.Time
)

// ResetSessionClock records the start of a new play session. Call once each
// time the bot enters a game. Sleep will then apply a progressive fatigue
// multiplier that rises from 1.0 to 1.25 over the first 3 hours, modelling
// the mild reaction-time slowdown observed in extended human play sessions.
func ResetSessionClock() {
	sessionMu.Lock()
	sessionStart = time.Now()
	sessionMu.Unlock()
}

// DriftFatigue returns a multiplier in [1.0, 1.25] that grows linearly over
// the first 3 hours of a session and then plateaus. Returns 1.0 when no
// session has been started (e.g. outside a play sequence).
// Exported so that other packages (e.g. SigmaDrift) can apply the same
// fatigue curve to their own timing parameters.
func DriftFatigue() float64 {
	sessionMu.RLock()
	start := sessionStart
	sessionMu.RUnlock()
	if start.IsZero() {
		return 1.0
	}
	f := time.Since(start).Hours() / 3.0
	if f > 1.0 {
		f = 1.0
	}
	return 1.0 + 0.25*f
}

// drawGamma returns a sample from the Gamma(shape, scale) distribution using
// the Marsaglia-Tsang squeeze method. shape must be >= 1.
func drawGamma(shape, scale float64) float64 {
	d := shape - 1.0/3.0
	c := 1.0 / math.Sqrt(9.0*d)
	for {
		x := rand.NormFloat64()
		v := 1.0 + c*x
		if v <= 0 {
			continue
		}
		v = v * v * v
		x2 := x * x
		u := rand.Float64()
		// Fast accept path
		if u < 1.0-0.0331*(x2*x2) {
			return d * v * scale
		}
		// Slow accept path
		if math.Log(u) < 0.5*x2+d*(1.0-v+math.Log(v)) {
			return d * v * scale
		}
	}
}

// GammaDurationMs returns a time.Duration sampled from a
// Gamma(shape, mean/shape) distribution with the requested mean in milliseconds.
// Higher shape → narrower spread; shape=3 gives a moderate right-skewed
// distribution that matches empirical human walk-click intervals better than
// the narrow uniform windows previously used.
func GammaDurationMs(meanMs float64, shape float64) time.Duration {
	scale := meanMs / shape
	sample := drawGamma(shape, scale)
	if sample < 1 {
		sample = 1
	}
	return time.Duration(sample) * time.Millisecond
}
