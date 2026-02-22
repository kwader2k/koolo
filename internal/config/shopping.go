package config

import "github.com/hectorgimenez/d2go/pkg/data/npc"

type ShoppingConfig struct {
	Enabled           bool     `yaml:"enabled"`
	MaxGoldToSpend    int      `yaml:"max_gold_to_spend"`
	MinGoldReserve    int      `yaml:"min_gold_reserve"`
	RefreshesPerRun   int      `yaml:"refreshes_per_run"`
	ShoppingRulesFile string   `yaml:"shopping_rules_file,omitempty"`
	ItemTypes         []string `yaml:"item_types,omitempty"`

	VendorAkara   bool `yaml:"vendor_akara"`
	VendorCharsi  bool `yaml:"vendor_charsi"`
	VendorGheed   bool `yaml:"vendor_gheed"`
	VendorFara    bool `yaml:"vendor_fara"`
	VendorDrognan bool `yaml:"vendor_drognan"`
	VendorElzix   bool `yaml:"vendor_elzix"`
	VendorOrmus   bool `yaml:"vendor_ormus"`
	VendorMalah   bool `yaml:"vendor_malah"`
	VendorAnya    bool `yaml:"vendor_anya"`
}

// SelectedVendors returns the list of vendor IDs to visit in shopping runs.
func (s ShoppingConfig) SelectedVendors() []npc.ID {
	out := make([]npc.ID, 0, 10)
	if s.VendorAkara   { out = append(out, npc.Akara) }
	if s.VendorCharsi  { out = append(out, npc.Charsi) }
	if s.VendorGheed   { out = append(out, npc.Gheed) }
	if s.VendorFara    { out = append(out, npc.Fara) }
	if s.VendorDrognan { out = append(out, npc.Drognan) }
	if s.VendorElzix   { out = append(out, npc.Elzix) }
	if s.VendorOrmus   { out = append(out, npc.Ormus) }
	if s.VendorMalah   { out = append(out, npc.Malah) }
	if s.VendorAnya    { out = append(out, npc.Drehya) } // Anya
	return out
}
