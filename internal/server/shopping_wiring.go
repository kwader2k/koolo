package server

import (
	"net/url"
	"strconv"
	"strings"

	"github.com/hectorgimenez/koolo/internal/config"
)

// ShoppingVM mirrors config.ShoppingConfig for templating.
type ShoppingVM struct {
	Enabled           bool
	MaxGoldToSpend    int
	MinGoldReserve    int
	RefreshesPerRun   int
	ShoppingRulesFile string
	ItemTypes         []string

	VendorAkara   bool
	VendorCharsi  bool
	VendorGheed   bool
	VendorFara    bool
	VendorDrognan bool
	VendorElzix   bool
	VendorOrmus   bool
	VendorMalah   bool
	VendorAnya    bool
}

// NewShoppingVM builds a view-model from config.
func NewShoppingVM(c config.ShoppingConfig) ShoppingVM {
	return ShoppingVM{
		Enabled:           c.Enabled,
		MaxGoldToSpend:    c.MaxGoldToSpend,
		MinGoldReserve:    c.MinGoldReserve,
		RefreshesPerRun:   c.RefreshesPerRun,
		ShoppingRulesFile: c.ShoppingRulesFile,
		ItemTypes:         append([]string{}, c.ItemTypes...),
		VendorAkara:       c.VendorAkara,
		VendorCharsi:      c.VendorCharsi,
		VendorGheed:       c.VendorGheed,
		VendorFara:        c.VendorFara,
		VendorDrognan:     c.VendorDrognan,
		VendorElzix:       c.VendorElzix,
		VendorOrmus:       c.VendorOrmus,
		VendorMalah:       c.VendorMalah,
		VendorAnya:        c.VendorAnya,
	}
}

// ApplyForm writes form values back into the given ShoppingConfig.
// Field names follow the template controls in run_settings_components.gohtml.
func (s *ShoppingVM) ApplyForm(dst *config.ShoppingConfig, form url.Values) {
	if dst == nil {
		return
	}
	dst.Enabled = postedBool(form.Get("shoppingEnabled"))

	if v, err := strconv.Atoi(form.Get("shoppingMaxGoldToSpend")); err == nil {
		dst.MaxGoldToSpend = v
	}
	if v, err := strconv.Atoi(form.Get("shoppingMinGoldReserve")); err == nil {
		dst.MinGoldReserve = v
	}
	if v, err := strconv.Atoi(form.Get("shoppingRefreshesPerRun")); err == nil {
		dst.RefreshesPerRun = v
	}
	dst.ShoppingRulesFile = form.Get("shoppingRulesFile")

	if raw := strings.TrimSpace(form.Get("shoppingItemTypes")); raw != "" {
		parts := strings.Split(raw, ",")
		items := make([]string, 0, len(parts))
		for _, p := range parts {
			if p = strings.TrimSpace(p); p != "" {
				items = append(items, p)
			}
		}
		dst.ItemTypes = items
	}

	// Vendors
	dst.VendorAkara = postedBool(form.Get("shoppingVendorAkara"))
	dst.VendorCharsi = postedBool(form.Get("shoppingVendorCharsi"))
	dst.VendorGheed = postedBool(form.Get("shoppingVendorGheed"))
	dst.VendorFara = postedBool(form.Get("shoppingVendorFara"))
	dst.VendorDrognan = postedBool(form.Get("shoppingVendorDrognan"))
	dst.VendorElzix = postedBool(form.Get("shoppingVendorElzix"))
	dst.VendorOrmus = postedBool(form.Get("shoppingVendorOrmus"))
	dst.VendorMalah = postedBool(form.Get("shoppingVendorMalah"))
	dst.VendorAnya = postedBool(form.Get("shoppingVendorAnya"))
}

func postedBool(v string) bool {
	switch strings.ToLower(v) {
	case "1", "true", "on", "yes":
		return true
	}
	return false
}
