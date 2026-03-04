package game

import (
	"math"
	"math/rand"

	"github.com/hectorgimenez/koolo/internal/utils"
)

// sdPoint is a trajectory waypoint; t is elapsed milliseconds from movement start.
type sdPoint struct {
	x, y float64
	t    float64
}

// sdConfig holds all tunable parameters for the SigmaDrift trajectory generator.
type sdConfig struct {
	// Fitts' Law parameters — control predicted movement time
	fittsA      float64 // intercept (ms)
	fittsB      float64 // slope (ms/bit)
	targetWidth float64 // effective target radius (px)

	// Primary stroke — ballistic phase covering ~93-97% of distance
	undershootMin   float64 // minimum reach fraction
	undershootMax   float64 // maximum reach fraction
	peakTimeRatio   float64 // peak velocity as a fraction of movement time
	primarySigmaMin float64 // lognormal spread min
	primarySigmaMax float64 // lognormal spread max

	// Overshoot / corrective sub-movements
	overshootProb        float64 // probability of overshoot
	overshootMin         float64 // overshoot reach fraction min
	overshootMax         float64 // overshoot reach fraction max
	correctionSigmaMin   float64 // first correction lognormal min
	correctionSigmaMax   float64 // first correction lognormal max
	secondCorrectionProb float64 // probability of a second corrective sub-movement

	curvatureScale float64 // lateral arc amplitude relative to distance

	// Ornstein-Uhlenbeck drift parameters
	ouTheta float64 // mean-reversion rate
	ouSigma float64 // diffusion coefficient

	// Physiological tremor (8-12 Hz)
	tremorFreqMin float64
	tremorFreqMax float64
	tremorAmpMin  float64
	tremorAmpMax  float64

	// Signal-dependent noise coefficient (Harris-Wolpert)
	sdnK float64

	// Inter-sample timing (gamma distribution)
	sampleDtMean float64 // mean inter-sample interval (ms)
	gammaShape   float64 // gamma distribution shape parameter

	maxTotalMs float64
}

var defaultSDConfig = sdConfig{
	fittsA:      20.0,
	fittsB:      35.0,
	targetWidth: 20.0,

	undershootMin:   0.92,
	undershootMax:   0.97,
	peakTimeRatio:   0.35,
	primarySigmaMin: 0.18,
	primarySigmaMax: 0.28,

	overshootProb:        0.15,
	overshootMin:         1.02,
	overshootMax:         1.08,
	correctionSigmaMin:   0.12,
	correctionSigmaMax:   0.20,
	secondCorrectionProb: 0.25,

	curvatureScale: 0.025,

	ouTheta: 3.5,
	ouSigma: 1.2,

	tremorFreqMin: 8.0,
	tremorFreqMax: 12.0,
	tremorAmpMin:  0.15,
	tremorAmpMax:  0.55,

	sdnK: 0.04,

	sampleDtMean: 7.8,
	gammaShape:   3.5,

	maxTotalMs: 0.0,
}

// sdNormalCDF computes the standard normal CDF using the error function.
func sdNormalCDF(x float64) float64 {
	return 0.5 * (1.0 + math.Erf(x/math.Sqrt2))
}

// sdLognormalCDF computes the lognormal CDF with onset t0, log-mean mu, log-sigma sigma.
func sdLognormalCDF(t, t0, mu, sigma float64) float64 {
	if t <= t0 {
		return 0.0
	}
	return sdNormalCDF((math.Log(t-t0) - mu) / sigma)
}

// sdLognormalPDF computes the lognormal PDF (velocity primitive).
func sdLognormalPDF(t, t0, mu, sigma float64) float64 {
	if t <= t0 {
		return 0.0
	}
	dt := t - t0
	z := (math.Log(dt) - mu) / sigma
	return 1.0 / (sigma * math.Sqrt(2.0*math.Pi) * dt) * math.Exp(-0.5*z*z)
}

func sdCurvatureProfile(s float64) float64 {
	if s <= 0.0 || s >= 1.0 {
		return 0.0
	}
	v := s * s * (1.0 - s) * (1.0 - s) * (1.0 - s)
	const norm = 0.4 * 0.4 * 0.6 * 0.6 * 0.6
	return v / norm
}

func sdDirectionFactor(angle float64) float64 {
	sa := math.Abs(math.Sin(angle))
	ca := math.Abs(math.Cos(angle))
	return 0.5 + 0.8*sa - 0.15*ca
}

func sdGamma(shape, scale float64) float64 {
	if shape < 1.0 {
		// Gamma(shape) = Gamma(shape+1) * U^(1/shape)
		return sdGamma(shape+1, scale) * math.Pow(rand.Float64(), 1.0/shape)
	}
	d := shape - 1.0/3.0
	c := 1.0 / math.Sqrt(9.0*d)
	for {
		x := rand.NormFloat64()
		v := 1.0 + c*x
		if v <= 0 {
			continue
		}
		v = v * v * v
		u := rand.Float64()
		if u < 1.0-0.0331*(x*x)*(x*x) {
			return d * v * scale
		}
		if math.Log(u) < 0.5*x*x+d*(1.0-v+math.Log(v)) {
			return d * v * scale
		}
	}
}

func sdUniform(lo, hi float64) float64 {
	return rand.Float64()*(hi-lo) + lo
}

func sdClamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

type sdCorrection struct {
	D, t0, mu, sigma float64
	dirX, dirY       float64
}

func bioMotionPath(x0, y0, x1, y1 float64, cfg sdConfig) []sdPoint {
	dx := x1 - x0
	dy := y1 - y0
	distance := math.Hypot(dx, dy)
	direction := math.Atan2(dy, dx)

	if distance < 1.0 {
		return []sdPoint{{x0, y0, 0.0}, {x1, y1, 50.0}}
	}

	// Unit vectors: tangent and normal to the movement axis.
	tx := dx / distance
	ty := dy / distance
	nx := -ty
	ny := tx

	fatigue := utils.DriftFatigue()
	id := math.Log2(distance/cfg.targetWidth + 1.0)
	mt := (cfg.fittsA*fatigue + cfg.fittsB*id) * math.Exp(rand.NormFloat64()*0.08)
	if mt < 20.0 {
		mt = 20.0
	}

	// Primary stroke: either undershoot or overshoot.
	var reach float64
	if rand.Float64() < cfg.overshootProb {
		reach = sdUniform(cfg.overshootMin, cfg.overshootMax)
	} else {
		reach = sdUniform(cfg.undershootMin, cfg.undershootMax)
	}
	primaryD := distance * reach
	primarySigma := sdUniform(cfg.primarySigmaMin, cfg.primarySigmaMax)
	peakT := mt * sdUniform(cfg.peakTimeRatio-0.03, cfg.peakTimeRatio+0.03)
	// Derive mu so that the lognormal mode (exp(mu-sigma²)) equals peakT.
	primaryMu := math.Log(peakT) + primarySigma*primarySigma

	// Build corrective sub-movements to cover the residual distance.
	var corrections []sdCorrection
	remaining := distance - primaryD
	if math.Abs(remaining) > 0.5 {
		dir := 1.0
		if remaining < 0 {
			dir = -1.0
		}
		cD := math.Abs(remaining) * sdUniform(0.88, 1.02)
		cS := sdUniform(cfg.correctionSigmaMin, cfg.correctionSigmaMax)
		cPeak := mt * sdUniform(0.12, 0.18)
		corrections = append(corrections, sdCorrection{
			D: cD, t0: mt * sdUniform(0.55, 0.68),
			mu: math.Log(cPeak) + cS*cS, sigma: cS,
			dirX: tx * dir, dirY: ty * dir,
		})

		left := remaining - cD*dir
		if math.Abs(left) > 0.3 && rand.Float64() < cfg.secondCorrectionProb {
			d2 := 1.0
			if left < 0 {
				d2 = -1.0
			}
			cD2 := math.Abs(left) * sdUniform(0.85, 1.05)
			cS2 := sdUniform(0.10, 0.16)
			cP2 := mt * sdUniform(0.08, 0.12)
			corrections = append(corrections, sdCorrection{
				D: cD2, t0: mt * sdUniform(0.78, 0.88),
				mu: math.Log(cP2) + cS2*cS2, sigma: cS2,
				dirX: tx * d2, dirY: ty * d2,
			})
		}
	}

	curvNorm := sdClamp(rand.NormFloat64(), -2.5, 2.5)
	curvAmp := distance * cfg.curvatureScale * sdDirectionFactor(direction) * curvNorm

	// Tremor parameters (per-movement constants).
	tremorFreq := sdUniform(cfg.tremorFreqMin, cfg.tremorFreqMax)
	tremorAmp := sdUniform(cfg.tremorAmpMin, cfg.tremorAmpMax)
	tphX := sdUniform(0.0, 2.0*math.Pi)
	tphY := sdUniform(0.0, 2.0*math.Pi)

	// Ornstein-Uhlenbeck state (starts at zero drift).
	ouX, ouY := 0.0, 0.0

	// Build gamma-distributed sample times.
	totalT := mt * 1.15
	gScale := cfg.sampleDtMean / cfg.gammaShape
	times := []float64{0.0}
	for t := 0.0; t < totalT; {
		dt := sdClamp(sdGamma(cfg.gammaShape, gScale), 2.0, 25.0)
		t += dt
		if t <= totalT+15.0 {
			times = append(times, t)
		}
	}

	if cfg.maxTotalMs > 0 && len(times) > 1 {
		if last := times[len(times)-1]; last > cfg.maxTotalMs {
			scale := cfg.maxTotalMs / last
			for i := range times {
				times[i] *= scale
			}
		}
	}

	result := make([]sdPoint, 0, len(times))
	for i, t := range times {
		// Inter-sample interval (seconds) for Euler-Maruyama integration.
		dtMs := cfg.sampleDtMean
		if i > 0 {
			dtMs = t - times[i-1]
		}
		dtS := dtMs / 1000.0

		// Primary stroke position via lognormal CDF (fractional progress).
		s := sdLognormalCDF(t, 0.0, primaryMu, primarySigma)
		bx := x0 + tx*primaryD*s
		by := y0 + ty*primaryD*s

		// Add lateral curvature.
		bx += nx * curvAmp * sdCurvatureProfile(s)
		by += ny * curvAmp * sdCurvatureProfile(s)

		// Add corrective sub-movements.
		for _, c := range corrections {
			cs := sdLognormalCDF(t, c.t0, c.mu, c.sigma)
			bx += c.dirX * c.D * cs
			by += c.dirY * c.D * cs
		}

		// Current overall speed used to gate noise and tremor.
		speed := primaryD * sdLognormalPDF(t, 0.0, primaryMu, primarySigma)
		for _, c := range corrections {
			speed += c.D * sdLognormalPDF(t, c.t0, c.mu, c.sigma)
		}

		// Ornstein-Uhlenbeck drift — Euler-Maruyama step.
		ouX += -cfg.ouTheta*ouX*dtS + cfg.ouSigma*math.Sqrt(dtS)*rand.NormFloat64()
		ouY += -cfg.ouTheta*ouY*dtS + cfg.ouSigma*math.Sqrt(dtS)*rand.NormFloat64()

		// Physiological tremor (8-12 Hz, proprioceptively suppressed at high speed).
		tS := t / 1000.0
		tremMod := 1.0 / (1.0 + speed*0.3)
		trX := tremorAmp * tremMod * math.Sin(2.0*math.Pi*tremorFreq*tS+tphX)
		trY := tremorAmp * tremMod * math.Sin(2.0*math.Pi*tremorFreq*tS+tphY)

		// Signal-dependent noise (Harris-Wolpert): noise ∝ motor command magnitude.
		sdnX := cfg.sdnK * speed * rand.NormFloat64()
		sdnY := cfg.sdnK * speed * rand.NormFloat64()

		result = append(result, sdPoint{
			x: bx + ouX + trX + sdnX,
			y: by + ouY + trY + sdnY,
			t: t,
		})
	}

	return result
}
