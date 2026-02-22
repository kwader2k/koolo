package main

import (
	"context"
	"fmt"
	"log"
	"log/slog"
	_ "net/http/pprof"
	"os"
	"path/filepath"
	"runtime/debug"
	"syscall"
	"time"
	"unsafe"

	sloggger "github.com/hectorgimenez/koolo/cmd/koolo/log"
	"github.com/hectorgimenez/koolo/internal/bot"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/event"
	"github.com/hectorgimenez/koolo/internal/remote/discord"
	"github.com/hectorgimenez/koolo/internal/remote/droplog"
	ngrokremote "github.com/hectorgimenez/koolo/internal/remote/ngrok"
	"github.com/hectorgimenez/koolo/internal/remote/telegram"
	"github.com/hectorgimenez/koolo/internal/server"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/hectorgimenez/koolo/internal/utils/winproc"
	"github.com/inkeliz/gowebview"
	"golang.org/x/sync/errgroup"
)

var (
	buildID   string
	buildTime string
)

// wrapWithRecover wraps a function with panic recovery logic
func wrapWithRecover(logger *slog.Logger, f func() error) func() error {
	return func() error {
		defer func() {
			if r := recover(); r != nil {
				stackTrace := debug.Stack()
				errMsg := fmt.Sprintf("panic recovered: %v\nStacktrace: %s", r, stackTrace)
				logger.Error(errMsg)
				sloggger.FlushLog()
			}
		}()
		return f()
	}
}

func main() {

	_ = buildID
	_ = buildTime

	err := config.Load()
	if err != nil {
		utils.ShowDialog("Error loading configuration", err.Error())
		log.Fatalf("Error loading configuration: %s", err.Error())
		return
	}

	// Ensure a sensible default delay for Auto Start if not configured
	if config.Koolo.AutoStart.DelaySeconds <= 0 {
		config.Koolo.AutoStart.DelaySeconds = 60
	}

	logger, err := sloggger.NewLogger(config.Koolo.Debug.Log, config.Koolo.LogSaveDirectory, "")
	if err != nil {
		log.Fatalf("Error starting logger: %s", err.Error())
	}
	defer sloggger.FlushAndClose()

	defer func() {
		if r := recover(); r != nil {
			err = fmt.Errorf("fatal error detected, Koolo will close with the following error: %v\n Stacktrace: %s", r, debug.Stack())
			logger.Error(err.Error())
			sloggger.FlushAndClose()
			utils.ShowDialog("Koolo error :(", fmt.Sprintf("Koolo will close due to an expected error, please check the latest log file for more info!\n %s", err.Error()))
		}
	}()

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	g, ctx := errgroup.WithContext(ctx)

	winproc.SetProcessDpiAware.Call() // Set DPI awareness to be able to read the correct scale and show the window correctly

	eventListener := event.NewListener(logger)

	// Centralized droplog writer registration
	dropBase := config.Koolo.LogSaveDirectory
	if dropBase == "" {
		dropBase = "logs"
	}
	dropDir := filepath.Join(dropBase, "droplogs")
	dropWriter := droplog.NewWriter(dropDir, logger)
	eventListener.Register(dropWriter.Handle)
	manager := bot.NewSupervisorManager(logger, eventListener)
	scheduler := bot.NewScheduler(manager, logger)
	go scheduler.Start()
	srv, err := server.New(logger, manager, scheduler)
	if err != nil {
		log.Fatalf("Error starting local server: %s", err.Error())
	}
	eventListener.Register(srv.HandleRunewordHistory)
	var ngrokTunnel *ngrokremote.Tunnel
	if config.Koolo.Ngrok.Enabled {
		if config.Koolo.Ngrok.Authtoken == "" && os.Getenv("NGROK_AUTHTOKEN") == "" {
			logger.Warn("ngrok enabled but no authtoken set; skipping tunnel start")
		} else {
			opts := ngrokremote.Options{
				LocalAddr:     "http://localhost:8087",
				Authtoken:     config.Koolo.Ngrok.Authtoken,
				Region:        config.Koolo.Ngrok.Region,
				Domain:        config.Koolo.Ngrok.Domain,
				BasicAuthUser: config.Koolo.Ngrok.BasicAuthUser,
				BasicAuthPass: config.Koolo.Ngrok.BasicAuthPass,
			}
			tunnel, err := ngrokremote.Start(ctx, opts)
			if err != nil {
				logger.Error("ngrok tunnel failed to start", slog.Any("error", err))
			} else {
				logger.Info("ngrok tunnel established", slog.String("url", tunnel.URL()))
				if config.Koolo.Ngrok.SendURL {
					go event.Send(event.NgrokTunnel(tunnel.URL()))
				}
			}
			ngrokTunnel = tunnel
		}
	}

	g.Go(wrapWithRecover(logger, func() error {
		defer cancel()
		displayScale := config.GetCurrentDisplayScale()

		// 1. Load dimensions from config, or use defaults
		width := config.Koolo.WindowWidth
		if width <= 0 {
			width = 1040
		}
		height := config.Koolo.WindowHeight
		if height <= 0 {
			height = 720
		}

		w, err := gowebview.New(&gowebview.Config{URL: "http://localhost:8087", WindowConfig: &gowebview.WindowConfig{
			Title: "Koolo Resurrected",
			Size: &gowebview.Point{
				X: int64(float64(width) * displayScale),
				Y: int64(float64(height) * displayScale),
			},
		}})
		if err != nil {
			if w != nil {
				w.Destroy()
			}
			return fmt.Errorf("error creating webview: %w", err)
		}

		// 2. Set HintNone to allow mouse resizing
		w.SetSize(&gowebview.Point{
			X: int64(float64(width) * displayScale),
			Y: int64(float64(height) * displayScale),
		}, gowebview.HintNone)

		// 3. Start Auto-Save Polling
		go func() {
			handle := w.Window() // Get native Windows handle
			user32 := syscall.NewLazyDLL("user32.dll")
			getWindowRect := user32.NewProc("GetWindowRect")
			isIconic := user32.NewProc("IsIconic")
			type RECT struct{ Left, Top, Right, Bottom int32 }

			ticker := time.NewTicker(5 * time.Second)
			defer ticker.Stop()

			for {
				select {
				case <-ctx.Done():
					return
				case <-ticker.C:
					// Check if minimized (IsIconic returns non-zero if minimized)
					minimized, _, _ := isIconic.Call(handle)
					if minimized != 0 {
						continue
					}

					var rect RECT
					ret, _, _ := getWindowRect.Call(handle, uintptr(unsafe.Pointer(&rect)))
					if ret != 0 {
						curW := int(float64(rect.Right-rect.Left) / displayScale)
						curH := int(float64(rect.Bottom-rect.Top) / displayScale)

						// Check if size is valid and has changed
						if curW > 100 && curH > 100 {
							if curW != config.Koolo.WindowWidth || curH != config.Koolo.WindowHeight {
								config.Koolo.WindowWidth = curW
								config.Koolo.WindowHeight = curH
								config.ValidateAndSaveConfig(*config.Koolo)
							}
						}
					}
				}
			}
		}()

		defer w.Destroy()
		w.Run()

		return nil
	}))

	// Discord Bot initialization
	if config.Koolo.Discord.Enabled {
		discordBot, err := discord.NewBot(
			config.Koolo.Discord.Token,
			config.Koolo.Discord.ChannelID,
			config.Koolo.Discord.ItemChannelID,
			manager,
			config.Koolo.Discord.UseWebhook,
			config.Koolo.Discord.WebhookURL,
			config.Koolo.Discord.ItemWebhookURL,
		)
		if err != nil {
			logger.Error("Discord could not been initialized", slog.Any("error", err))
			return
		}

		eventListener.Register(discordBot.Handle)
		if !config.Koolo.Discord.UseWebhook {
			g.Go(wrapWithRecover(logger, func() error {
				return discordBot.Start(ctx)
			}))
		}
	}

	// Telegram Bot initialization
	if config.Koolo.Telegram.Enabled {
		telegramBot, err := telegram.NewBot(config.Koolo.Telegram.Token, config.Koolo.Telegram.ChatID, logger)
		if err != nil {
			logger.Error("Telegram could not been initialized", slog.Any("error", err))
			return
		}

		eventListener.Register(telegramBot.Handle)
		g.Go(wrapWithRecover(logger, func() error {
			return telegramBot.Start(ctx)
		}))
	}

	g.Go(wrapWithRecover(logger, func() error {
		defer cancel()
		return srv.Listen(8087)
	}))

	g.Go(wrapWithRecover(logger, func() error {
		defer cancel()
		return eventListener.Listen(ctx)
	}))

	g.Go(wrapWithRecover(logger, func() error {
		<-ctx.Done()
		logger.Info("Koolo shutting down...")
		cancel()
		manager.StopAll()
		scheduler.Stop()
		err = srv.Stop()
		if err != nil {
			logger.Error("error stopping local server", slog.Any("error", err))
		}
		if ngrokTunnel != nil {
			if closeErr := ngrokTunnel.Close(); closeErr != nil {
				logger.Error("error stopping ngrok tunnel", slog.Any("error", closeErr))
			}
		}

		return err
	}))

	err = g.Wait()
	if err != nil {
		cancel()
		logger.Error("Error running Koolo", slog.Any("error", err))
		return
	}

	sloggger.FlushAndClose()
}
