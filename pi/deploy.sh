#!/usr/bin/env bash
# Deploy (or update) the BESS dashboard on the Raspberry Pi gateway.
# Usage: pi/deploy.sh [--replace-config] [ssh-host]
#   ssh-host defaults to $BESS_PI_HOST (an SSH host or ~/.ssh/config alias for the Pi;
#   the user needs passwordless sudo there).
#
# Installs /opt/bess-dashboard (keeps an existing config.json unless
# --replace-config; the replaced one is saved as config.json.prev), the systemd unit
# and the nginx site, then restarts the service and reloads nginx. If nginx
# rejects the new site, the previous file is put back. Never reboots the Pi.
# Needs the SSH key in the agent: ssh-add --apple-load-keychain
set -euo pipefail
REPLACE_CONFIG=0
if [ "${1:-}" = "--replace-config" ]; then REPLACE_CONFIG=1; shift; fi
HOST="${1:-${BESS_PI_HOST:-}}"
if [ -z "$HOST" ]; then
    echo "Usage: pi/deploy.sh [--replace-config] <ssh-host>   (or set BESS_PI_HOST)"
    exit 2
fi
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$HERE")"
STAGE=/tmp/bess-dashboard-stage

ssh "$HOST" "rm -rf $STAGE && mkdir -p $STAGE/static"
scp -q "$HERE/bess_dashboard.py" "$ROOT/tools/bess_rx.py" "$HERE/config.json" \
       "$HERE/bess-dashboard.service" "$HERE/nginx-bess.conf" "$HOST:$STAGE/"
scp -q "$HERE/static/index.html" "$HOST:$STAGE/static/"


ssh -T "$HOST" "sudo REPLACE_CONFIG=$REPLACE_CONFIG bash -s" <<'EOF'
set -euo pipefail
S=/tmp/bess-dashboard-stage
D=/opt/bess-dashboard
SITE=/etc/nginx/sites-available/default

install -d -m 755 $D $D/static
install -m 644 $S/bess_dashboard.py $S/bess_rx.py $D/
# the anomaly model runs on the module now; remove the Pi-side copies
rm -f $D/bess_features.py $D/model.json $D/model.json.prev
install -m 644 $S/static/index.html $D/static/
if [ -f $D/config.json ] && [ "$REPLACE_CONFIG" != 1 ]; then
    echo "keeping the existing $D/config.json (use --replace-config to overwrite)"
else
    [ -f $D/config.json ] && cp -a $D/config.json $D/config.json.prev
    install -m 644 $S/config.json $D/
fi
python3 -m py_compile $D/bess_dashboard.py $D/bess_rx.py
install -m 644 $S/bess-dashboard.service /etc/systemd/system/

[ -f $SITE.before-bess ] || cp -a $SITE $SITE.before-bess  # first deploy: keep the original
cp -a $SITE $SITE.last
install -m 644 $S/nginx-bess.conf $SITE
if ! nginx -t 2>&1; then
    echo "nginx rejected the new site; restoring the previous one"
    cp -a $SITE.last $SITE
    exit 1
fi

systemctl daemon-reload
systemctl enable bess-dashboard >/dev/null 2>&1
systemctl restart bess-dashboard
systemctl reload nginx
for i in $(seq 1 120); do   # up to 60 s: startup rescores the last 24 h of history
    curl -fs -o /dev/null http://127.0.0.1:8080/api/status && break
    sleep 0.5
done
echo "service: $(systemctl is-active bess-dashboard)"
curl -fsS -o /dev/null http://127.0.0.1:8080/api/status && echo "dashboard API: OK"
curl -fsS -o /dev/null http://127.0.0.1/ && echo "nginx http -> dashboard: OK"
curl -fsSk -o /dev/null https://127.0.0.1/ && echo "nginx https -> dashboard: OK"
rm -rf $S
EOF
echo "Dashboard: http://$(ssh -G "$HOST" | awk '/^hostname /{print $2}')/"
