//go:build !noisegen

// Fallback entropy constants used when noise_gen.go has not been generated.
package buildnoise

var (
	entropy0 uint64 = 0x0000000000000000
	entropy1 uint64 = 0x0000000000000000
	entropy2 uint64 = 0x0000000000000000
	entropy3 uint64 = 0x0000000000000000
)
