package pickit

// GetAllStatTypes returns all available stat types for NIP rules
func GetAllStatTypes() []StatType {
	return []StatType{
		// Resistances
		{ID: "fireresist", Name: "Fire Resist", NipProperty: "[fireresist]", MinValue: 1, MaxValue: 90, IsPercent: true},
		{ID: "coldresist", Name: "Cold Resist", NipProperty: "[coldresist]", MinValue: 1, MaxValue: 90, IsPercent: true},
		{ID: "lightresist", Name: "Lightning Resist", NipProperty: "[lightresist]", MinValue: 1, MaxValue: 90, IsPercent: true},
		{ID: "poisonresist", Name: "Poison Resist", NipProperty: "[poisonresist]", MinValue: 1, MaxValue: 90, IsPercent: true},

		// Core Stats
		{ID: "strength", Name: "Strength", NipProperty: "[strength]", MinValue: 1, MaxValue: 30, IsPercent: false},
		{ID: "dexterity", Name: "Dexterity", NipProperty: "[dexterity]", MinValue: 1, MaxValue: 30, IsPercent: false},
		{ID: "vitality", Name: "Vitality", NipProperty: "[vitality]", MinValue: 1, MaxValue: 30, IsPercent: false},
		{ID: "energy", Name: "Energy", NipProperty: "[energy]", MinValue: 1, MaxValue: 30, IsPercent: false},

		// Life/Mana
		{ID: "maxhp", Name: "Max Life", NipProperty: "[maxhp]", MinValue: 1, MaxValue: 120, IsPercent: false},
		{ID: "maxmana", Name: "Max Mana", NipProperty: "[maxmana]", MinValue: 1, MaxValue: 120, IsPercent: false},
		{ID: "regen", Name: "Replenish Life", NipProperty: "[regen]", MinValue: 1, MaxValue: 20, IsPercent: false},
		{ID: "maxstamina", Name: "Max Stamina", NipProperty: "[maxstamina]", MinValue: 1, MaxValue: 100, IsPercent: true},

		// Speed Mods
		{ID: "fcr", Name: "Faster Cast Rate", NipProperty: "[fcr]", MinValue: 1, MaxValue: 20, IsPercent: true},
		{ID: "fhr", Name: "Faster Hit Recovery", NipProperty: "[fhr]", MinValue: 1, MaxValue: 60, IsPercent: true},
		{ID: "frw", Name: "Faster Run/Walk", NipProperty: "[frw]", MinValue: 1, MaxValue: 40, IsPercent: true},
		{ID: "ias", Name: "Increased Attack Speed", NipProperty: "[ias]", MinValue: 1, MaxValue: 40, IsPercent: true},
		{ID: "fblock", Name: "Faster Block Rate", NipProperty: "[fblock]", MinValue: 1, MaxValue: 30, IsPercent: true},

		// Damage/AR
		{ID: "mindamage", Name: "Min Damage", NipProperty: "[mindamage]", MinValue: 1, MaxValue: 50, IsPercent: false},
		{ID: "maxdamage", Name: "Max Damage", NipProperty: "[maxdamage]", MinValue: 1, MaxValue: 50, IsPercent: false},
		{ID: "tohit", Name: "Attack Rating", NipProperty: "[tohit]", MinValue: 1, MaxValue: 450, IsPercent: false},
		// NOTE: Internally we used to call this "eddmg".
		// The actual NIP property supported by the engine is [enhanceddamage].
		{ID: "eddmg", Name: "Enhanced Damage", NipProperty: "[enhanceddamage]", MinValue: 1, MaxValue: 450, IsPercent: true},
		{ID: "deadlystrike", Name: "Deadly Strike", NipProperty: "[deadlystrike]", MinValue: 1, MaxValue: 100, IsPercent: true},
		{ID: "crushingblow", Name: "Crushing Blow", NipProperty: "[crushingblow]", MinValue: 1, MaxValue: 50, IsPercent: true},
		{ID: "openwounds", Name: "Open Wounds", NipProperty: "[openwounds]", MinValue: 1, MaxValue: 100, IsPercent: true},

		// Leech
		{ID: "lifeleech", Name: "Life Leech", NipProperty: "[lifeleech]", MinValue: 1, MaxValue: 15, IsPercent: true},
		{ID: "manaleech", Name: "Mana Leech", NipProperty: "[manaleech]", MinValue: 1, MaxValue: 15, IsPercent: true},

		// Magic Find / Gold Find
		{ID: "itemmagicbonus", Name: "Magic Find", NipProperty: "[itemmagicbonus]", MinValue: 1, MaxValue: 50, IsPercent: true},
		{ID: "itemgoldbonus", Name: "Gold Find", NipProperty: "[itemgoldbonus]", MinValue: 1, MaxValue: 100, IsPercent: true},
		{ID: "itemfindpotion", Name: "Better Chance of Potions", NipProperty: "[itemfindpotion]", MinValue: 1, MaxValue: 50, IsPercent: true},

		// Damage Reduction
		{ID: "damageresist", Name: "Damage Reduced", NipProperty: "[damageresist]", MinValue: 1, MaxValue: 50, IsPercent: false},
		{ID: "magicdamagereduction", Name: "Magic Damage Reduced", NipProperty: "[magicdamagereduction]", MinValue: 1, MaxValue: 30, IsPercent: false},
		{ID: "percentdamageresist", Name: "Damage Reduced %", NipProperty: "[percentdamageresist]", MinValue: 1, MaxValue: 50, IsPercent: true},

		// Defense
		{ID: "defense", Name: "Defense", NipProperty: "[defense]", MinValue: 1, MaxValue: 2000, IsPercent: false},
		{ID: "enhanceddefense", Name: "Enhanced Defense", NipProperty: "[enhanceddefense]", MinValue: 1, MaxValue: 300, IsPercent: true},
		{ID: "defenseperLevel", Name: "Defense Per Level", NipProperty: "[defenseperLevel]", MinValue: 1, MaxValue: 50, IsPercent: false},

		// Sockets
		{ID: "sockets", Name: "Sockets", NipProperty: "[sockets]", MinValue: 1, MaxValue: 6, IsPercent: false},

		// Class Skills
		{ID: "amazonskills", Name: "Amazon Skills", NipProperty: "[amazonskills]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "assassinskills", Name: "Assassin Skills", NipProperty: "[assassinskills]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "necromancerskills", Name: "Necromancer Skills", NipProperty: "[necromancerskills]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "druidskills", Name: "Druid Skills", NipProperty: "[druidskills]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "paladinskills", Name: "Paladin Skills", NipProperty: "[paladinskills]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "barbarianskills", Name: "Barbarian Skills", NipProperty: "[barbarianskills]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "sorceressskills", Name: "Sorceress Skills", NipProperty: "[sorceressskills]", MinValue: 1, MaxValue: 3, IsPercent: false},

		// Skill Tabs - Sorceress
		{ID: "coldskilltab", Name: "Cold Skills (Sorc)", NipProperty: "[coldskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "fireskilltab", Name: "Fire Skills (Sorc)", NipProperty: "[fireskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "lightningskilltab", Name: "Lightning Skills (Sorc)", NipProperty: "[lightningskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},

		// Skill Tabs - Necromancer
		{ID: "poisonskilltab", Name: "Poison & Bone (Necro)", NipProperty: "[poisonskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "summonskilltab", Name: "Summoning (Necro)", NipProperty: "[summonskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "cursesskilltab", Name: "Curses (Necro)", NipProperty: "[cursesskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},

		// Skill Tabs - Paladin
		{ID: "combatskilltab", Name: "Combat Skills (Pala)", NipProperty: "[combatskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "offensiveskilltab", Name: "Offensive Auras (Pala)", NipProperty: "[offensiveskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "defensiveskilltab", Name: "Defensive Auras (Pala)", NipProperty: "[defensiveskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},

		// Skill Tabs - Amazon
		{ID: "bowskilltab", Name: "Bow & Crossbow (Ama)", NipProperty: "[bowskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "passiveskilltab", Name: "Passive & Magic (Ama)", NipProperty: "[passiveskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "javelinSkilltab", Name: "Javelin & Spear (Ama)", NipProperty: "[javelinSkilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},

		// Skill Tabs - Barbarian
		{ID: "warcriesskilltab", Name: "Warcries (Barb)", NipProperty: "[warcriesskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "combatmasterskilltab", Name: "Combat Masteries (Barb)", NipProperty: "[combatmasterskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "combatbarbskilltab", Name: "Combat Skills (Barb)", NipProperty: "[combatbarbskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},

		// Skill Tabs - Druid
		{ID: "elementalskilltab", Name: "Elemental (Druid)", NipProperty: "[elementalskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "shapeshiftskilltab", Name: "Shape Shifting (Druid)", NipProperty: "[shapeshiftskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "summondruidskilltab", Name: "Summoning (Druid)", NipProperty: "[summondruidskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},

		// Skill Tabs - Assassin
		{ID: "trapskilltab", Name: "Traps (Sin)", NipProperty: "[trapskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "shadowdisciplineskilltab", Name: "Shadow Disciplines (Sin)", NipProperty: "[shadowdisciplineskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "martialartsskilltab", Name: "Martial Arts (Sin)", NipProperty: "[martialartsskilltab]", MinValue: 1, MaxValue: 3, IsPercent: false},

		// Specific Skills - Examples (can be expanded)
		{ID: "skillteleport", Name: "+Teleport", NipProperty: "[skillteleport]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "skilllightning", Name: "+Lightning", NipProperty: "[skilllightning]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "skillfrozenorb", Name: "+Frozen Orb", NipProperty: "[skillfrozenorb]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "skillbonespear", Name: "+Bone Spear", NipProperty: "[skillbonespear]", MinValue: 1, MaxValue: 3, IsPercent: false},
		{ID: "skillraiseSkeletons", Name: "+Raise Skeleton", NipProperty: "[skillraiseSkeletons]", MinValue: 1, MaxValue: 3, IsPercent: false},

		// Other
		{ID: "lightradius", Name: "Light Radius", NipProperty: "[lightradius]", MinValue: 1, MaxValue: 5, IsPercent: false},
		{ID: "reducedreq", Name: "Requirements", NipProperty: "[reducedreq]", MinValue: -100, MaxValue: -1, IsPercent: true},
		{ID: "durability", Name: "Durability", NipProperty: "[durability]", MinValue: 1, MaxValue: 100, IsPercent: false},
		{ID: "itemreplenishdurability", Name: "Repairs Over Time", NipProperty: "[itemreplenishdurability]", MinValue: 1, MaxValue: 1, IsPercent: false},
		{ID: "indestructible", Name: "Indestructible", NipProperty: "[indestructible]", MinValue: 1, MaxValue: 1, IsPercent: false},
	}
}

// GetStatTypeByID returns a specific stat type by ID
func GetStatTypeByID(id string) *StatType {
	// Compatibility: allow both "eddmg" (legacy) and "enhanceddamage" (engine property).
	if id == "enhanceddamage" {
		id = "eddmg"
	}
	for _, st := range GetAllStatTypes() {
		if st.ID == id {
			return &st
		}
	}
	return nil
}

// GetStatTypesByCategory returns stat types grouped by category
func GetStatTypesByCategory() map[string][]StatType {
	return map[string][]StatType{
		"Resistances": {
			*GetStatTypeByID("fireresist"),
			*GetStatTypeByID("coldresist"),
			*GetStatTypeByID("lightresist"),
			*GetStatTypeByID("poisonresist"),
		},
		"Attributes": {
			*GetStatTypeByID("strength"),
			*GetStatTypeByID("dexterity"),
			*GetStatTypeByID("vitality"),
			*GetStatTypeByID("energy"),
		},
		"Life & Mana": {
			*GetStatTypeByID("maxhp"),
			*GetStatTypeByID("maxmana"),
			*GetStatTypeByID("regen"),
			*GetStatTypeByID("maxstamina"),
		},
		"Speed Mods": {
			*GetStatTypeByID("fcr"),
			*GetStatTypeByID("fhr"),
			*GetStatTypeByID("frw"),
			*GetStatTypeByID("ias"),
			*GetStatTypeByID("fblock"),
		},
		"Damage": {
			*GetStatTypeByID("mindamage"),
			*GetStatTypeByID("maxdamage"),
			*GetStatTypeByID("tohit"),
			*GetStatTypeByID("eddmg"),
			*GetStatTypeByID("deadlystrike"),
			*GetStatTypeByID("crushingblow"),
			*GetStatTypeByID("openwounds"),
		},
		"Leech": {
			*GetStatTypeByID("lifeleech"),
			*GetStatTypeByID("manaleech"),
		},
		"Magic Find": {
			*GetStatTypeByID("itemmagicbonus"),
			*GetStatTypeByID("itemgoldbonus"),
			*GetStatTypeByID("itemfindpotion"),
		},
		"Defense": {
			*GetStatTypeByID("defense"),
			*GetStatTypeByID("enhanceddefense"),
			*GetStatTypeByID("damageresist"),
			*GetStatTypeByID("magicdamagereduction"),
			*GetStatTypeByID("percentdamageresist"),
		},
	}
}
