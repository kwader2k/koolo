package packet

import (
	"encoding/binary"

	"github.com/hectorgimenez/d2go/pkg/data/skill"
)

// SkillSelection represents packet 0x3C - Select Skill for Left or Right Mouse Button
// Used to change the active skill via packet injection (F1-F9 keybinds)
// Packet structure (9 bytes):
//
//	Byte 0: 0x3C (Select Skill packet ID)
//	Bytes 1-2: Skill ID (little-endian, 2 bytes)
//	Byte 3: 0x00 (padding)
//	Byte 4: Mouse button flag (0x00 = right, 0x80 = left)
//	Bytes 5-8: 0xFF 0xFF 0xFF 0xFF (constant suffix)
//
// Captured examples (Right-click):
//
//	3C 28 00 00 00 FF FF FF FF - Frozen Armor (skill ID 40)
//	3C 2A 00 00 00 FF FF FF FF - Static Field (skill ID 42)
//	3C 36 00 00 00 FF FF FF FF - Teleport (skill ID 54)
//	3C 3B 00 00 00 FF FF FF FF - Blizzard (skill ID 59)
//
// Captured examples (Left-click):
//
//	3C 27 00 00 80 FF FF FF FF - Frost Nova (skill ID 39)
//	3C 2D 00 00 80 FF FF FF FF - Glacial Spike (skill ID 45)
//	3C 37 00 00 80 FF FF FF FF - Ice Blast (skill ID 55)
type SkillSelection struct {
	PacketID    byte
	SkillID     uint16
	Padding     byte
	MouseButton byte // 0x00 = right, 0x80 = left
	Suffix      uint32
}

// NewSkillSelection creates packet 0x3C to select right-click skill
// Used for swapping active skill without clicking UI buttons
func NewSkillSelection(skillID skill.ID) *SkillSelection {
	return &SkillSelection{
		PacketID:    0x3C,
		SkillID:     uint16(skillID),
		Padding:     0x00,
		MouseButton: 0x00, // Right-click
		Suffix:      0xFFFFFFFF,
	}
}

// NewLeftSkillSelection creates packet 0x3C to select left-click skill
// Used for swapping active left-click skill without clicking UI buttons
func NewLeftSkillSelection(skillID skill.ID) *SkillSelection {
	return &SkillSelection{
		PacketID:    0x3C,
		SkillID:     uint16(skillID),
		Padding:     0x00,
		MouseButton: 0x80, // Left-click
		Suffix:      0xFFFFFFFF,
	}
}

// GetPayload returns the byte representation of the packet
func (p *SkillSelection) GetPayload() []byte {
	buf := make([]byte, 9)
	buf[0] = p.PacketID
	binary.LittleEndian.PutUint16(buf[1:3], p.SkillID)
	buf[3] = p.Padding
	buf[4] = p.MouseButton
	binary.LittleEndian.PutUint32(buf[5:9], p.Suffix)
	return buf
}
