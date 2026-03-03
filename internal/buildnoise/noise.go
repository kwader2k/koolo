// Package buildnoise injects per-build entropy into the binary.
// The build script regenerates noise_gen.go before each compilation,
// changing constants and init-time side effects so that every produced
// binary has a unique layout, hash, and string table — even when the
// source code is otherwise identical.
package buildnoise

import (
	"encoding/binary"
	"hash/fnv"
)

// entropy0–entropy3 are declared in noise_gen.go (generated per build).
// If noise_gen.go doesn't exist yet, noise_defaults.go provides fallbacks.

// salt is an unexported package-level var whose address is taken,
// preventing the linker from dead-stripping the entropy vars above.
var salt uintptr

func init() {
	h := fnv.New64a()
	// Hash each entropy var individually — never assume contiguous layout.
	var buf [8]byte
	binary.LittleEndian.PutUint64(buf[:], entropy0)
	_, _ = h.Write(buf[:])
	binary.LittleEndian.PutUint64(buf[:], entropy1)
	_, _ = h.Write(buf[:])
	binary.LittleEndian.PutUint64(buf[:], entropy2)
	_, _ = h.Write(buf[:])
	binary.LittleEndian.PutUint64(buf[:], entropy3)
	_, _ = h.Write(buf[:])
	salt = uintptr(h.Sum64())
}

// Nonce returns a value derived from compile-time noise.
// Calling it from main (even discarding the result) ensures the
// linker cannot strip this package.
//
//go:noinline
func Nonce() uintptr { return salt }
