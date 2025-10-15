package game

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	packet "github.com/hectorgimenez/koolo/internal/packet"
)

type ProcessSender interface {
	SendPacket([]byte) error
}

type PacketSender struct {
	process ProcessSender
}

func NewPacketSender(process ProcessSender) *PacketSender {
	return &PacketSender{
		process: process,
	}
}

func (ps *PacketSender) SendPacket(packet []byte) error {
	return ps.process.SendPacket(packet)
}

func (ps *PacketSender) PickUpItem(item data.Item) error {
	err := ps.SendPacket(packet.NewPickUpItem(item).GetPayload())
	if err != nil {
		return fmt.Errorf("failed to send pick item packet: %w", err)
	}

	return nil
}

func (ps *PacketSender) InteractWithTp(object data.Object) error {
	if err := ps.SendPacket(packet.NewTpInteraction(object).GetPayload()); err != nil {
		return fmt.Errorf("failed to send tp interaction packet: %w", err)
	}
	return nil
}

func (ps *PacketSender) InteractWithEntrance(entrance data.Entrance) error {
	if err := ps.SendPacket(packet.NewEntranceInteraction(entrance).GetPayload()); err != nil {
		return fmt.Errorf("failed to send entrance interaction packet: %w", err)
	}
	return nil
}
