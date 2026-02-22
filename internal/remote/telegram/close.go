package telegram

import "net/http"

func (b *Bot) Close() {
    if b == nil || b.bot == nil { return }
    b.bot.StopReceivingUpdates()
    if c, ok := b.bot.Client.(*http.Client); ok && c != nil {
        if tr, ok := c.Transport.(*http.Transport); ok && tr != nil {
            tr.CloseIdleConnections()
        }
    }
}
