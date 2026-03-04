//go:build !noisegen

// noise_defaults.go provides fallback entropy constants so the package
// compiles even when noise_gen.go has not been generated yet (e.g.
// during development or CI without the generate_noise.ps1 step).
package buildnoise

var (
	entropy0 uint64 = 0x0000000000000000
	entropy1 uint64 = 0x0000000000000000
	entropy2 uint64 = 0x0000000000000000
	entropy3 uint64 = 0x0000000000000000
)
