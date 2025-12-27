package discord

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"io"
	"mime/multipart"
	"net/http"
	"strings"
	"time"

	"github.com/bwmarrin/discordgo"
)

type webhookClient struct {
	url    string
	client *http.Client
}

func newWebhookClient(url string) *webhookClient {
	return &webhookClient{
		url: strings.TrimSpace(url),
		client: &http.Client{
			Timeout: 10 * time.Second,
		},
	}
}

func (w *webhookClient) Send(ctx context.Context, content, fileName string, fileData []byte) error {
	var body bytes.Buffer
	writer := multipart.NewWriter(&body)

	if err := writer.WriteField("content", content); err != nil {
		writer.Close()
		return fmt.Errorf("failed to prepare webhook payload: %w", err)
	}

	if len(fileData) > 0 && fileName != "" {
		part, err := writer.CreateFormFile("file", fileName)
		if err != nil {
			writer.Close()
			return fmt.Errorf("failed to add webhook file field: %w", err)
		}

		if _, err := part.Write(fileData); err != nil {
			writer.Close()
			return fmt.Errorf("failed to write webhook file data: %w", err)
		}
	}

	contentType := writer.FormDataContentType()
	if err := writer.Close(); err != nil {
		return fmt.Errorf("failed to finalize webhook payload: %w", err)
	}

	req, err := http.NewRequestWithContext(ctx, http.MethodPost, w.url, &body)
	if err != nil {
		return fmt.Errorf("failed to create webhook request: %w", err)
	}
	req.Header.Set("Content-Type", contentType)

	resp, err := w.client.Do(req)
	if err != nil {
		return fmt.Errorf("webhook request failed: %w", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode >= http.StatusBadRequest {
		respBody, _ := io.ReadAll(resp.Body)
		return fmt.Errorf("webhook returned %d: %s", resp.StatusCode, strings.TrimSpace(string(respBody)))
	}

	return nil
}

func (w *webhookClient) SendEmbed(ctx context.Context, embed *discordgo.MessageEmbed) error {
	var body bytes.Buffer
	writer := multipart.NewWriter(&body)

	payload := struct {
		Embeds []*discordgo.MessageEmbed `json:"embeds"`
	}{
		Embeds: []*discordgo.MessageEmbed{embed},
	}
	payloadJSON, err := json.Marshal(payload)
	if err != nil {
		writer.Close()
		return fmt.Errorf("failed to serialize webhook embed: %w", err)
	}

	if err := writer.WriteField("payload_json", string(payloadJSON)); err != nil {
		writer.Close()
		return fmt.Errorf("failed to prepare webhook embed payload: %w", err)
	}

	contentType := writer.FormDataContentType()
	if err := writer.Close(); err != nil {
		return fmt.Errorf("failed to finalize webhook embed payload: %w", err)
	}

	req, err := http.NewRequestWithContext(ctx, http.MethodPost, w.url, &body)
	if err != nil {
		return fmt.Errorf("failed to create webhook request: %w", err)
	}
	req.Header.Set("Content-Type", contentType)

	resp, err := w.client.Do(req)
	if err != nil {
		return fmt.Errorf("webhook request failed: %w", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode >= http.StatusBadRequest {
		respBody, _ := io.ReadAll(resp.Body)
		return fmt.Errorf("webhook returned %d: %s", resp.StatusCode, strings.TrimSpace(string(respBody)))
	}

	return nil
}
