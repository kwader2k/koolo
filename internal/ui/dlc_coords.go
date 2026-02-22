package ui

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
)

// DLCTabCoords maps item Name to fixed screen coordinates for each DLC stash tab.
// DLC tab items have opaque 1D memory positions (Position.X, Y=0) that do not
// follow any regular grid formula. Each item type always appears at the same
// fixed screen position, so we use a hardcoded lookup table.
//
// Coordinates are for 1280x720 HD resolution.
var DLCTabCoords = map[item.Name]data.Position{
	// ── Gems tab ──────────────────────────────────────────────
	// Column 1: Diamond
	"ChippedDiamond":  {X: 147, Y: 163},
	"FlawedDiamond":   {X: 151, Y: 189},
	"Diamond":         {X: 149, Y: 237},
	"FlawlessDiamond": {X: 151, Y: 258},
	"PerfectDiamond":  {X: 148, Y: 294},
	// Column 2: Emerald
	"ChippedEmerald":  {X: 191, Y: 163},
	"FlawedEmerald":   {X: 191, Y: 197},
	"Emerald":         {X: 191, Y: 231},
	"FlawlessEmerald": {X: 192, Y: 262},
	"PerfectEmerald":  {X: 187, Y: 289},
	// Column 3: Ruby
	"ChippedRuby":  {X: 231, Y: 162},
	"FlawedRuby":   {X: 230, Y: 196},
	"Ruby":         {X: 230, Y: 228},
	"FlawlessRuby": {X: 232, Y: 262},
	"PerfectRuby":  {X: 232, Y: 298},
	// Column 4: Topaz
	"ChippedTopaz":  {X: 272, Y: 165},
	"FlawedTopaz":   {X: 271, Y: 197},
	"Topaz":         {X: 271, Y: 227},
	"FlawlessTopaz": {X: 273, Y: 265},
	"PerfectTopaz":  {X: 274, Y: 295},
	// Column 5: Amethyst
	"ChippedAmethyst":  {X: 313, Y: 164},
	"FlawedAmethyst":   {X: 314, Y: 196},
	"Amethyst":         {X: 314, Y: 230},
	"FlawlessAmethyst": {X: 314, Y: 264},
	"PerfectAmethyst":  {X: 313, Y: 293},
	// Column 6: Sapphire
	"ChippedSapphire":  {X: 355, Y: 164},
	"FlawedSapphire":   {X: 356, Y: 195},
	"Sapphire":         {X: 354, Y: 229},
	"FlawlessSapphire": {X: 357, Y: 264},
	"PerfectSapphire":  {X: 356, Y: 294},
	// Column 7: Skull
	"ChippedSkull":  {X: 394, Y: 166},
	"FlawedSkull":   {X: 393, Y: 196},
	"Skull":         {X: 393, Y: 228},
	"FlawlessSkull": {X: 395, Y: 265},
	"PerfectSkull":  {X: 394, Y: 295},

	// ── Materials tab ────────────────────────────────────────
	// Worldstone Shards
	"WesternWorldstoneShard":  {X: 275, Y: 239},
	"EasternWorldstoneShard":  {X: 309, Y: 238},
	"SouthernWorldstoneShard": {X: 347, Y: 239},
	"DeepWorldstoneShard":     {X: 383, Y: 239},
	"NorthernWorldstoneShard": {X: 416, Y: 239},
	// Essences & Token
	"TokenOfAbsolution":             {X: 276, Y: 286},
	"TwistedEssenceOfSuffering":     {X: 312, Y: 288},
	"ChargedEssenceOfHatred":        {X: 345, Y: 287},
	"BurningEssenceOfTerror":        {X: 380, Y: 285},
	"FesteringEssenceOfDestruction": {X: 416, Y: 282},
	// Keys
	"KeyOfTerror":      {X: 132, Y: 181},
	"KeyOfHate":        {X: 166, Y: 176},
	"KeyOfDestruction": {X: 200, Y: 178},
	// Potions
	"RejuvenationPotion":     {X: 130, Y: 286},
	"FullRejuvenationPotion": {X: 164, Y: 286},
	// Organs
	"DiablosHorn":    {X: 131, Y: 238},
	"BaalsEye":       {X: 163, Y: 238},
	"MephistosBrain": {X: 199, Y: 237},
	// Uber Ancient Summon Materials
	"UberAncientSummonMaterialAct1": {X: 276, Y: 181},
	"UberAncientSummonMaterialAct2": {X: 311, Y: 180},
	"UberAncientSummonMaterialAct3": {X: 347, Y: 180},
	"UberAncientSummonMaterialAct4": {X: 380, Y: 181},
	"UberAncientSummonMaterialAct5": {X: 414, Y: 180},

	// ── Runes tab ────────────────────────────────────────────
	// Row 1
	"ElRune":  {X: 132, Y: 177},
	"EldRune": {X: 169, Y: 183},
	"TirRune": {X: 205, Y: 179},
	"NefRune": {X: 238, Y: 178},
	"EthRune": {X: 272, Y: 176},
	"IthRune": {X: 307, Y: 179},
	"TalRune": {X: 341, Y: 179},
	"RalRune": {X: 376, Y: 178},
	"OrtRune": {X: 412, Y: 179},
	// Row 2
	"ThulRune":  {X: 133, Y: 212},
	"AmnRune":   {X: 171, Y: 212},
	"SolRune":   {X: 204, Y: 212},
	"ShaelRune": {X: 239, Y: 212},
	"DolRune":   {X: 273, Y: 214},
	"HelRune":   {X: 308, Y: 212},
	"IoRune":    {X: 342, Y: 211},
	"LumRune":   {X: 377, Y: 213},
	"KoRune":    {X: 412, Y: 214},
	// Row 3
	"FalRune": {X: 134, Y: 247},
	"LemRune": {X: 169, Y: 246},
	"PulRune": {X: 203, Y: 246},
	"UmRune":  {X: 240, Y: 247},
	"MalRune": {X: 271, Y: 247},
	"IstRune": {X: 306, Y: 247},
	"GulRune": {X: 343, Y: 247},
	"VexRune": {X: 377, Y: 247},
	"OhmRune": {X: 412, Y: 246},
	// Row 4
	"LoRune":  {X: 135, Y: 281},
	"SurRune": {X: 169, Y: 282},
	"BerRune": {X: 377, Y: 281},
	"JahRune": {X: 410, Y: 281},
	// Row 5
	"ChamRune": {X: 135, Y: 314},
	"ZodRune":  {X: 412, Y: 316},
}

// GetDLCTabScreenCoords returns the fixed screen position for a DLC tab item,
// looked up by item Name. Returns the position and true if found, or zero
// position and false if the item is not in the lookup table.
func GetDLCTabScreenCoords(name item.Name) (data.Position, bool) {
	pos, ok := DLCTabCoords[name]
	return pos, ok
}
