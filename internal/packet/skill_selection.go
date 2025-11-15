package packet

import (
	"encoding/binary"

	"github.com/hectorgimenez/d2go/pkg/data/skill"
)

// SkillSelection represents packet 0x3C - Select Skill for Right Mouse Button
// Used to change the active right-click skill via packet injection (F1-F9 keybinds)
// Packet structure (9 bytes):
//
//	Byte 0: 0x3C (Select Skill packet ID)
//	Bytes 1-2: Skill ID (little-endian, 2 bytes)
//	Bytes 3-4: 0x00 0x00 (padding/flags)
//	Bytes 5-8: 0xFF 0xFF 0xFF 0xFF (constant suffix)
//
// Captured examples:
//
//	3C 28 00 00 00 FF FF FF FF - Frozen Armor (skill ID 40)
//	3C 2A 00 00 00 FF FF FF FF - Static Field (skill ID 42)
//	3C 36 00 00 00 FF FF FF FF - Teleport (skill ID 54)
//	3C 3B 00 00 00 FF FF FF FF - Blizzard (skill ID 59)
//	3C 3A 00 00 00 FF FF FF FF - Energy Shield (skill ID 58)
//	3C 2B 00 00 00 FF FF FF FF - Telekinesis (skill ID 43)
//	3C DC 00 00 00 FF FF FF FF - Unknown skill (skill ID 220)
type SkillSelection struct {
	PacketID byte
	SkillID  uint16
	Padding  uint16
	Suffix   uint32
}

// NewSkillSelection creates packet 0x3C to select right-click skill
// Used for swapping active skill without clicking UI buttons
func NewSkillSelection(skillID skill.ID) *SkillSelection {
	return &SkillSelection{
		PacketID: 0x3C,
		SkillID:  uint16(skillID),
		Padding:  0x0000,
		Suffix:   0xFFFFFFFF,
	}
}

// GetPayload returns the byte representation of the packet
func (p *SkillSelection) GetPayload() []byte {
	buf := make([]byte, 9)
	buf[0] = p.PacketID
	binary.LittleEndian.PutUint16(buf[1:3], p.SkillID)
	binary.LittleEndian.PutUint16(buf[3:5], p.Padding)
	binary.LittleEndian.PutUint32(buf[5:9], p.Suffix)
	return buf
}
