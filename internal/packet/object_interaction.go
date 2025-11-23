package packet

import (
	"encoding/binary"

	"github.com/hectorgimenez/d2go/pkg/data"
)

// TelekinesisInteraction represents packet 0x0D - Cast Skill on Entity
// Used for Sorceress telekinesis interaction with objects (waypoints, chests, shrines)
// Packet structure (9 bytes):
//
//	Byte 0: 0x0D (Cast Skill on Entity packet ID)
//	Bytes 1-2: Skill code 0x0002 (little-endian) - for object interaction
//	Bytes 3-4: 0x00 0x00 (padding/flags)
//	Bytes 5-8: Object GID (little-endian, 4 bytes)
//
// Captured example (waypoint):
//
//	0D 02 00 00 00 29 34 FF 75
//	- Skill code: 0x0002 (object interaction)
//	- Object GID: 0x75FF3429
type TelekinesisInteraction struct {
	PacketID  byte
	SkillCode uint16
	Padding   uint16
	ObjectGID uint32
}

// NewTelekinesisInteraction creates packet 0x0D for telekinesis object interaction
// Used by Sorceress for waypoints, chests, shrines from distance
func NewTelekinesisInteraction(objectGID data.UnitID) *TelekinesisInteraction {
	return &TelekinesisInteraction{
		PacketID:  0x0D,
		SkillCode: 0x0002, // Object interaction skill code from captured packets
		Padding:   0x0000,
		ObjectGID: uint32(objectGID),
	}
}

// GetPayload returns the byte representation of the packet
func (p *TelekinesisInteraction) GetPayload() []byte {
	buf := make([]byte, 9)
	buf[0] = p.PacketID
	binary.LittleEndian.PutUint16(buf[1:3], p.SkillCode)
	binary.LittleEndian.PutUint16(buf[3:5], p.Padding)
	binary.LittleEndian.PutUint32(buf[5:9], p.ObjectGID)
	return buf
}
