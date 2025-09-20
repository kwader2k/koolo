package action

import (
	"fmt"
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/context"
)

// Define a Requirement struct to package the check logic and name.
// This MUST be defined before its use in requiredChecks.
type statRequirement struct {
	Name string
	Check func(itm data.Item) (bool, string)
}

// All six required checks with item Type comparison correctly accessing the .Code field.
var requiredChecks = []statRequirement{
	// 1) Merc Helm: TYPE == "helmet" AND >= 1 Lifesteal
	{"MercHelm_Lifesteal_Found", func(itm data.Item) (bool, string) {
		itemTypeCode := itm.Type().Code
		if itemTypeCode != "helmet" {
			return false, fmt.Sprintf("Type: FAIL (Found: %s, Req: helmet)", itemTypeCode)
		}
		s, f := itm.FindStat(stat.LifeSteal, 0)
		val := 0
		if f {
			val = s.Value
		}
		success := f && val >= 1
		return success, fmt.Sprintf("Type: PASS, Lifesteal: Found=%t, Value=%d (Req: >=1)", f, val)
	}},

	// 2) Merc Weapon: TYPE == "polearm" AND 35 FCR AND 9 Minimum Damage
	{"MercWeapon_FCR_MinDmg_Found", func(itm data.Item) (bool, string) {
		itemTypeCode := itm.Type().Code
		if itemTypeCode != "polearm" {
			return false, fmt.Sprintf("Type: FAIL (Found: %s, Req: polearm)", itemTypeCode)
		}
		fcr, fFCR := itm.FindStat(stat.FasterCastRate, 0)
		minDmg, fMinDmg := itm.FindStat(stat.MinDamage, 0)
		success := fFCR && fMinDmg && fcr.Value == 35 && minDmg.Value == 9
		logMsg := fmt.Sprintf("Type: PASS, FCR: %d (Req: 35), MinDmg: %d (Req: 9)", fcr.Value, minDmg.Value)
		return success, logMsg
	}},

	// 3) Merc Armor: TYPE == "armor" AND 50 Fire Resistance AND 10 Energy
	{"MercArmor_FireRes_Energy_Found", func(itm data.Item) (bool, string) {
		itemTypeCode := itm.Type().Code
		if itemTypeCode != "armor" {
			return false, fmt.Sprintf("Type: FAIL (Found: %s, Req: armor)", itemTypeCode)
		}
		fireRes, fFireRes := itm.FindStat(stat.FireResist, 0)
		energy, fEnergy := itm.FindStat(stat.Energy, 0)
		success := fFireRes && fEnergy && fireRes.Value == 50 && energy.Value == 10
		logMsg := fmt.Sprintf("Type: PASS, FireRes: %d (Req: 50), Energy: %d (Req: 10)", fireRes.Value, energy.Value)
		return success, logMsg
	}},

	// 4) Player Armor: TYPE == "armor" AND 50 Fire Resistance AND 10 Energy
	{"PlayerArmor_FireRes_Energy_Found", func(itm data.Item) (bool, string) {
		itemTypeCode := itm.Type().Code
		if itemTypeCode != "armor" {
			return false, fmt.Sprintf("Type: FAIL (Found: %s, Req: armor)", itemTypeCode)
		}
		fireRes, fFireRes := itm.FindStat(stat.FireResist, 0)
		energy, fEnergy := itm.FindStat(stat.Energy, 0)
		success := fFireRes && fEnergy && fireRes.Value == 50 && energy.Value == 10
		logMsg := fmt.Sprintf("Type: PASS, FireRes: %d (Req: 50), Energy: %d (Req: 10)", fireRes.Value, energy.Value)
		return success, logMsg
	}},

	// 5) Player Weapon: TYPE == "sword" AND 2 All Skills (Exact) AND >= 25 Faster Cast Rate (Minimum check)
	{"PlayerWeapon_Skills_FCR_Found", func(itm data.Item) (bool, string) {
		itemTypeCode := itm.Type().Code
		if itemTypeCode != "sword" {
			return false, fmt.Sprintf("Type: FAIL (Found: %s, Req: sword)", itemTypeCode)
		}
		allSkills, fAllSkills := itm.FindStat(stat.AllSkills, 0)
		fcr, fFCR := itm.FindStat(stat.FasterCastRate, 0)
		success := fAllSkills && fFCR && allSkills.Value == 2 && fcr.Value >= 25
		logMsg := fmt.Sprintf("Type: PASS, AllSkills: %d (Req: 2), FCR: %d (Req: >=25)", allSkills.Value, fcr.Value)
		return success, logMsg
	}},

	// 6) Player Helm: TYPE == "helmet" AND 1 All Skill AND 35 Lightres
	{"PlayerHelm_Skill_LightRes_Found", func(itm data.Item) (bool, string) {
		itemTypeCode := itm.Type().Code
		if itemTypeCode != "helmet" {
			return false, fmt.Sprintf("Type: FAIL (Found: %s, Req: helmet)", itemTypeCode)
		}
		allSkills, fAllSkills := itm.FindStat(stat.AllSkills, 0)
		lightRes, fLightRes := itm.FindStat(stat.LightningResist, 0)
		success := fAllSkills && fLightRes && allSkills.Value == 1 && lightRes.Value == 35
		logMsg := fmt.Sprintf("Type: PASS, AllSkills: %d (Req: 1), LightRes: %d (Req: 35)", allSkills.Value, lightRes.Value)
		return success, logMsg
	}},
}

// CheckAllItemsForStats iterates over all items and returns a map indicating which requirements were met,
// and also a map of UnitIDs used to satisfy those requirements.
func CheckAllItemsForStats() (map[string]bool, map[string]data.UnitID) {
	ctx := context.Get()
	allItems := ctx.Data.Inventory.AllItems // Field access

	results := make(map[string]bool)
	// Tracks the UnitID (data.UnitID) used to satisfy a requirement.
	usedUnitIDs := make(map[string]data.UnitID) 
	
	for _, req := range requiredChecks {
		results[req.Name] = false
	}

	ctx.Logger.Debug(fmt.Sprintf("Starting universal item stat check for %d items...", len(allItems)))

	for _, itm := range allItems {
		ctx.Logger.Debug(fmt.Sprintf("--- Checking Item: %s (UnitID: %d, Type: %s) ---", itm.IdentifiedName, itm.UnitID, itm.Type().Code))

		if itm.UnitID == 0 {
			continue
		}

		for _, req := range requiredChecks {
			if results[req.Name] {
				continue
			}
			
			// NEW LOGIC: Enforce that Merc Armor and Player Armor must be separate items.
			if req.Name == "MercArmor_FireRes_Energy_Found" || req.Name == "PlayerArmor_FireRes_Energy_Found" {
				// Determine the other armor requirement name.
				otherReqName := ""
				if req.Name == "MercArmor_FireRes_Energy_Found" {
					otherReqName = "PlayerArmor_FireRes_Energy_Found"
				} else {
					otherReqName = "MercArmor_FireRes_Energy_Found"
				}
				
				// If the *other* requirement is already met AND was met by this exact item (same UnitID), skip.
				if results[otherReqName] && usedUnitIDs[otherReqName] == itm.UnitID {
					ctx.Logger.Debug(fmt.Sprintf("[%s]: SKIP -> Item %d already used for %s.", req.Name, itm.UnitID, otherReqName))
					continue
				}
			}

			isMet, logDetails := req.Check(itm)
			
			logStatus := "FAIL"
			if isMet {
				logStatus = "SUCCESS"
				results[req.Name] = true
				// Record the UnitID that satisfied this requirement.
				usedUnitIDs[req.Name] = itm.UnitID 
			}
			
			ctx.Logger.Debug(fmt.Sprintf("[%s]: %s -> %s", req.Name, logStatus, logDetails))
		}
	}

	ctx.Logger.Debug("--- All item checks complete. ---")
	return results, usedUnitIDs
}

// AreAllRequiredItemsFound runs the check and returns true ONLY if all six requirements were met.
func AreAllRequiredItemsFound() bool {
    results, _ := CheckAllItemsForStats()
    
    if len(results) != len(requiredChecks) {
        return false
    }

    for _, found := range results {
        if !found {
            return false
        }
    }
    
    return true

}
