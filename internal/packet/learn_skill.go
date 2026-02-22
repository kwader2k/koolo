package packet

import (
	"encoding/binary"

	"github.com/hectorgimenez/d2go/pkg/data/skill"
)

// LearnSkill represents packet 0x3B for learning/allocating skill points
// Format: [0x3B][SkillID:2bytes][0x0000]
// Total: 5 bytes
type LearnSkill struct {
	PacketID byte
	SkillID  uint16
	Padding  uint16
}

// NewLearnSkill creates a new skill learning packet
func NewLearnSkill(skillID skill.ID) *LearnSkill {
	return &LearnSkill{
		PacketID: 0x3B,
		SkillID:  uint16(skillID),
		Padding:  0x0000,
	}
}

func (p *LearnSkill) GetPayload() []byte {
	buf := make([]byte, 5)
	buf[0] = p.PacketID
	binary.LittleEndian.PutUint16(buf[1:], p.SkillID)
	binary.LittleEndian.PutUint16(buf[3:], p.Padding)
	return buf
}
