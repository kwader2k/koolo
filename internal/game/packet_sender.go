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

func (ps *PacketSender) PickupItem(item data.Item) error {
	err := ps.process.SendPacket(packet.NewPickItem(item).GetPayload())
	if err != nil {
		return fmt.Errorf("failed to send pick item packet: %w", err)
	}

	return nil
}
