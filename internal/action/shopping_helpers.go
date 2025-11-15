package action

import (
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
)

// Vendor → town area
var VendorLocationMap = map[npc.ID]area.ID{
	npc.Akara:   area.RogueEncampment,
	npc.Charsi:  area.RogueEncampment,
	npc.Gheed:   area.RogueEncampment,
	npc.Fara:    area.LutGholein,
	npc.Drognan: area.LutGholein,
	npc.Elzix:   area.LutGholein,
	npc.Ormus:   area.KurastDocks,
	npc.Halbu:   area.ThePandemoniumFortress,
	npc.Malah:   area.Harrogath,
	npc.Larzuk:  area.Harrogath,
	npc.Drehya:  area.Harrogath, // Anya in backend
}
