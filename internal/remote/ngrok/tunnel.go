package ngrok

import (
	"context"
	"errors"
	"net/url"
	"os"
	"time"

	ngrok "golang.ngrok.com/ngrok"
	"golang.ngrok.com/ngrok/config"
)

type Options struct {
	LocalAddr     string
	Authtoken     string
	Region        string
	Domain        string
	BasicAuthUser string
	BasicAuthPass string
}

type Tunnel struct {
	forwarder ngrok.Forwarder
}

func Start(ctx context.Context, opts Options) (*Tunnel, error) {
	if opts.LocalAddr == "" {
		return nil, errors.New("ngrok local address is required")
	}

	backend, err := url.Parse(opts.LocalAddr)
	if err != nil {
		return nil, err
	}

	httpOpts := make([]config.HTTPEndpointOption, 0, 2)
	if opts.Domain != "" {
		httpOpts = append(httpOpts, config.WithDomain(opts.Domain))
	}
	if opts.BasicAuthUser != "" && opts.BasicAuthPass != "" {
		httpOpts = append(httpOpts, config.WithBasicAuth(opts.BasicAuthUser, opts.BasicAuthPass))
	}

	connectOpts := make([]ngrok.ConnectOption, 0, 2)
	if opts.Authtoken != "" {
		connectOpts = append(connectOpts, ngrok.WithAuthtoken(opts.Authtoken))
	} else if os.Getenv("NGROK_AUTHTOKEN") != "" {
		connectOpts = append(connectOpts, ngrok.WithAuthtokenFromEnv())
	}
	if opts.Region != "" {
		connectOpts = append(connectOpts, ngrok.WithRegion(opts.Region))
	}

	fwd, err := ngrok.ListenAndForward(ctx, backend, config.HTTPEndpoint(httpOpts...), connectOpts...)
	if err != nil {
		return nil, err
	}

	return &Tunnel{forwarder: fwd}, nil
}

func (t *Tunnel) URL() string {
	if t == nil || t.forwarder == nil {
		return ""
	}
	return t.forwarder.URL()
}

func (t *Tunnel) Close() error {
	if t == nil || t.forwarder == nil {
		return nil
	}
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	return t.forwarder.CloseWithContext(ctx)
}
