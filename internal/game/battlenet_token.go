package game

import (
	"context"
	"errors"
	"fmt"
	"net/url"
	"strings"
	"time"

	"github.com/go-rod/rod"
	"github.com/go-rod/rod/lib/launcher"
	"github.com/go-rod/rod/lib/proto"
)

// GetBattleNetToken logs in to Battle.net and returns the authentication token.
func GetBattleNetToken(username, password, realm string) (string, error) {
	return getBattleNetToken(context.Background(), username, password, realm, nil)
}

func GetBattleNetTokenWithDebug(username, password, realm string, debug func(string)) (string, error) {
	return getBattleNetToken(context.Background(), username, password, realm, debug)
}

// GetBattleNetTokenWithDebugContext logs in to Battle.net and returns the authentication token, using the provided context.
func GetBattleNetTokenWithDebugContext(ctx context.Context, username, password, realm string, debug func(string)) (string, error) {
	return getBattleNetToken(ctx, username, password, realm, debug)
}

func getBattleNetToken(ctx context.Context, username, password, realm string, debug func(string)) (string, error) {
	if ctx == nil {
		ctx = context.Background()
	}
	parentCtx := ctx
	ctx, cancel := context.WithTimeout(parentCtx, 3*time.Minute)
	defer cancel()
	logLine := func(format string, args ...any) {
		line := fmt.Sprintf(format, args...)
		fmt.Print(line)
		if debug != nil {
			debug(strings.TrimSuffix(line, "\n"))
		}
	}

	maybeLogBrowserDownload(ctx, logLine)

	launch := launcher.New().Context(ctx).Headless(true)
	controlURL, err := launch.Launch()
	if err != nil {
		return "", fmt.Errorf("failed to launch browser: %w", err)
	}
	defer launch.Cleanup()

	browser := rod.New().ControlURL(controlURL)
	if err := browser.Connect(); err != nil {
		return "", fmt.Errorf("failed to connect to browser: %w", err)
	}
	defer func() {
		_ = browser.Close()
	}()

	page, err := browser.Context(ctx).Page(proto.TargetCreateTarget{})
	if err != nil {
		return "", fmt.Errorf("failed to create page: %w", err)
	}

	loginURL := getBattleNetLoginURL(realm)
	logLine("[DEBUG] Login URL: %s\n", loginURL)

	if err := page.Navigate(loginURL); err != nil {
		return "", fmt.Errorf("failed to navigate to login page: %w", err)
	}
	if err := page.WaitLoad(); err != nil {
		return "", fmt.Errorf("failed to wait for login page: %w", err)
	}
	logLine("[DEBUG] Login page loaded\n")

	emailInput, err := page.Timeout(10 * time.Second).Element("input[type='text']")
	if err != nil {
		return "", fmt.Errorf("failed to find email input field: %w", err)
	}
	if err := emailInput.Input(username); err != nil {
		return "", fmt.Errorf("failed to input username: %w", err)
	}
	logLine("[DEBUG] Username entered\n")

	continueBtn, err := page.Timeout(5 * time.Second).Element("button[type='submit']")
	if err != nil {
		return "", fmt.Errorf("failed to find continue button: %w", err)
	}
	if err := continueBtn.Click(proto.InputMouseButtonLeft, 1); err != nil {
		return "", fmt.Errorf("failed to click continue button: %w", err)
	}
	logLine("[DEBUG] Continue button clicked\n")

	time.Sleep(2 * time.Second)

	passwordInput, err := page.Timeout(10 * time.Second).Element("input[type='password']")
	if err != nil {
		return "", fmt.Errorf("failed to find password input field: %w", err)
	}
	if err := passwordInput.Input(password); err != nil {
		return "", fmt.Errorf("failed to input password: %w", err)
	}
	logLine("[DEBUG] Password entered\n")

	loginBtn, err := page.Timeout(5 * time.Second).Element("button[type='submit']")
	if err != nil {
		return "", fmt.Errorf("failed to find login button: %w", err)
	}
	if err := loginBtn.Click(proto.InputMouseButtonLeft, 1); err != nil {
		return "", fmt.Errorf("failed to click login button: %w", err)
	}
	logLine("[DEBUG] Login button clicked\n")

	time.Sleep(3 * time.Second)
	logLine("[DEBUG] Starting to monitor URL for token...\n")

	maxAttempts := 15
	for i := 0; i < maxAttempts; i++ {
		if err := ctx.Err(); err != nil {
			return "", err
		}
		info, err := page.Info()
		if err != nil {
			return "", fmt.Errorf("failed to read current URL: %w", err)
		}
		currentURL := info.URL
		logLine("[DEBUG %d/%d] Current URL: %s\n", i+1, maxAttempts, sanitizeTokenInURL(currentURL))

		if strings.Contains(currentURL, "/challenge/") {
			logLine("[INFO] Additional authentication required! Opening browser window...\n")
			_ = browser.Close()

			return getBattleNetTokenWithUI(parentCtx, username, password, realm, debug)
		}

		if strings.Contains(currentURL, "ST=") {
			parsedURL, err := url.Parse(currentURL)
			if err == nil {
				token := parsedURL.Query().Get("ST")
				if token != "" {
					logLine("[DEBUG] Token found\n")
					return token, nil
				}
			}
		}

		time.Sleep(1 * time.Second)
	}

	return "", errors.New("authentication token not found")
}

func getBattleNetTokenWithUI(ctx context.Context, username, password, realm string, debug func(string)) (string, error) {
	if ctx == nil {
		ctx = context.Background()
	}
	ctx, cancel := context.WithTimeout(ctx, 5*time.Minute)
	defer cancel()
	logLine := func(format string, args ...any) {
		line := fmt.Sprintf(format, args...)
		fmt.Print(line)
		if debug != nil {
			debug(strings.TrimSuffix(line, "\n"))
		}
	}

	logLine("[INFO] Please complete additional authentication in the browser window...\n")

	maybeLogBrowserDownload(ctx, logLine)

	launch := launcher.New().Context(ctx).Headless(false)
	controlURL, err := launch.Launch()
	if err != nil {
		return "", fmt.Errorf("failed to launch browser UI: %w", err)
	}
	defer launch.Cleanup()

	browser := rod.New().ControlURL(controlURL)
	if err := browser.Connect(); err != nil {
		return "", fmt.Errorf("failed to connect to browser: %w", err)
	}
	defer func() {
		_ = browser.Close()
	}()

	page, err := browser.Context(ctx).Page(proto.TargetCreateTarget{})
	if err != nil {
		return "", fmt.Errorf("failed to create page: %w", err)
	}

	loginURL := getBattleNetLoginURL(realm)
	if err := page.Navigate(loginURL); err != nil {
		return "", fmt.Errorf("failed to navigate to login page: %w", err)
	}
	if err := page.WaitLoad(); err != nil {
		return "", fmt.Errorf("failed to wait for login page: %w", err)
	}

	emailInput, err := page.Timeout(10 * time.Second).Element("input[type='text']")
	if err == nil {
		if err := emailInput.Input(username); err == nil {
			continueBtn, _ := page.Timeout(5 * time.Second).Element("button[type='submit']")
			if continueBtn != nil {
				_ = continueBtn.Click(proto.InputMouseButtonLeft, 1)
			}
		}
	}

	time.Sleep(2 * time.Second)

	passwordInput, err := page.Timeout(10 * time.Second).Element("input[type='password']")
	if err == nil {
		if err := passwordInput.Input(password); err == nil {
			loginBtn, _ := page.Timeout(5 * time.Second).Element("button[type='submit']")
			if loginBtn != nil {
				_ = loginBtn.Click(proto.InputMouseButtonLeft, 1)
			}
		}
	}

	logLine("[INFO] Waiting for authentication completion (5 minutes timeout)...\n")
	logLine("[INFO] Please check your email or Battle.net app for verification code\n")

	maxAttempts := 300
	for i := 0; i < maxAttempts; i++ {
		if err := ctx.Err(); err != nil {
			return "", err
		}
		info, err := page.Info()
		if err != nil {
			return "", fmt.Errorf("failed to read current URL: %w", err)
		}
		currentURL := info.URL

		if i%10 == 0 {
			logLine("[INFO] Waiting for authentication... (%d/%d seconds)\n", i, maxAttempts)
		}

		if strings.Contains(currentURL, "ST=") {
			parsedURL, err := url.Parse(currentURL)
			if err == nil {
				token := parsedURL.Query().Get("ST")
				if token != "" {
					logLine("[DEBUG] Token found\n")
					return token, nil
				}
			}
		}

		time.Sleep(1 * time.Second)
	}

	return "", errors.New("authentication timeout (5 minutes)")
}

func getBattleNetLoginURL(realm string) string {
	switch realm {
	case "eu.actual.battle.net":
		return "https://eu.battle.net/login/en/?externalChallenge=login&app=OSI"
	case "kr.actual.battle.net":
		return "https://kr.battle.net/login/en/?externalChallenge=login&app=OSI"
	case "us.actual.battle.net":
		return "https://us.battle.net/login/en/?externalChallenge=login&app=OSI"
	default:
		// Default to US
		return "https://us.battle.net/login/en/?externalChallenge=login&app=OSI"
	}
}

func maybeLogBrowserDownload(ctx context.Context, logLine func(string, ...any)) {
	browser := launcher.NewBrowser()
	browser.Context = ctx
	if err := browser.Validate(); err != nil {
		logLine("[INFO] Downloading and installing the Chrome browser required for token generation (first time only, ~150MB)...\n")
	}
}

func sanitizeTokenInURL(rawURL string) string {
	parsed, err := url.Parse(rawURL)
	if err != nil {
		return rawURL
	}
	query := parsed.Query()
	updated := false
	for key := range query {
		if strings.EqualFold(key, "ST") {
			query.Set(key, "REDACTED")
			updated = true
		}
	}
	if !updated {
		return rawURL
	}
	parsed.RawQuery = query.Encode()
	return parsed.String()
}
