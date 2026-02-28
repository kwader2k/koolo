package utils

import (
	"math/rand"
	"time"
)

// PingMultiplierType represents the sensitivity level for ping-adaptive delays
type PingMultiplierType float64

const (
	// Light - For lightweight operations (polling, small checks)
	// Adds 1x ping to base delay - minimal impact from network latency
	Light PingMultiplierType = 1.0

	// Medium - For medium operations (UI interactions, clicks, movements)
	// Adds 2x ping to base delay - moderate sensitivity to network conditions
	Medium PingMultiplierType = 2.0

	// Critical - For critical operations (state changes, transitions)
	// Adds 4x ping to base delay - high sensitivity to ensure operation completes
	Critical PingMultiplierType = 4.0
)

// Legacy constants for backward compatibility (deprecated)
const (
	PingMultiplierLight    = Light
	PingMultiplierMedium   = Medium
	PingMultiplierCritical = Critical
)

// pingGetter is a function that returns current ping
// Set this at initialization to avoid import cycles
var pingGetter func() int

// SetPingGetter sets the function to retrieve current ping
// Called during initialization by context package
func SetPingGetter(getter func() int) {
	pingGetter = getter
}

// addRandomJitter adds a random duration between 0-1000ms
// Returns random milliseconds to make bot behavior more human-like
func addRandomJitter(randomize int) int {
	if randomize <= 0 {
		randomize = 3000 // Default jitter up to 1000ms
	}
	return rand.Intn(randomize) // 0-randomize ms
}

// Sleep provides a Sleep function that randomize the sleep time up/down to a maximum of 30%
func Sleep(milliseconds int, randomize int) {
	jitter := addRandomJitter(randomize)

	time.Sleep(time.Duration(milliseconds+jitter) * time.Millisecond)
}

func SleepDuration(duration time.Duration, randomize int) {
	jitter := addRandomJitter(randomize)

	time.Sleep(duration + time.Duration(jitter)*time.Millisecond)
}

// GetCurrentPing retrieves the current ping value
// Returns 50ms if pingGetter not initialized (safe default)
func GetCurrentPing() int {
	if pingGetter == nil {
		return 50 // Safe default
	}
	ping := pingGetter()
	if ping < 10 {
		return 50 // Sanity check
	}
	return ping
}

// PingMultiplier calculates delay based on current ping
// multiplier: sensitivity level (use PingMultiplierLight/Medium/Critical constants)
// minimum: base delay to which ping adjustment is added
// Returns the adjusted delay in milliseconds: minimum + (multiplier * ping)
func PingMultiplier(multiplier PingMultiplierType, minimum int) int {
	ping := GetCurrentPing()

	// Calculate adjusted delay: base minimum + ping adjustment
	adjusted := minimum + int(float64(ping)*float64(multiplier))

	// Cap at 5 seconds to prevent infinite waits
	if adjusted > 5000 {
		return 5000
	}

	return adjusted
}

// PingSleep waits for minimum + (multiplier * ping) + random jitter (0-1000ms)
// multiplier: sensitivity level (use PingMultiplierLight/Medium/Critical constants)
// minimum: base delay in milliseconds
// This is the primary function to use for ping-based delays
func PingSleep(multiplier PingMultiplierType, minimum int, randomize int) {
	ms := PingMultiplier(multiplier, minimum)
	jitter := addRandomJitter(randomize)

	time.Sleep(time.Duration(ms+jitter) * time.Millisecond)
}

// RetryDelay calculates escalating delay for retry attempts
// Pattern: base + (ping * attempt)
// attemptNumber: which attempt this is (1-indexed)
// basePing: base ping multiplier (usually 1.0 - represents ping units)
// minimumMs: minimum delay in milliseconds
func RetryDelay(attemptNumber int, basePing float64, minimumMs int) int {
	ping := GetCurrentPing()

	// Pattern: base + (ping * attempt)
	// basePing here represents "ping units" for the retry multiplier
	delay := minimumMs + int(basePing*float64(ping)*float64(attemptNumber))

	// Cap at 5 seconds
	if delay > 5000 {
		return 5000
	}

	return delay
}

// RetrySleep waits with escalating delay based on attempt number + random jitter (0-1000ms)
func RetrySleep(attemptNumber int, basePing float64, minimumMs int) {
	ms := RetryDelay(attemptNumber, basePing, minimumMs)
	jitter := addRandomJitter(500)
	time.Sleep(time.Duration(ms+jitter) * time.Millisecond)
}

// PingAwareTimeout calculates timeout based on ping with cap
// Pattern: min(maxMs, baseMs + ping * multiplier)
func PingAwareTimeout(multiplier float64, baseMs int, maxMs int) int {
	ping := GetCurrentPing()

	timeout := baseMs + int(float64(ping)*multiplier)

	if timeout > maxMs {
		return maxMs
	}

	if timeout < baseMs {
		return baseMs
	}

	return timeout
}
