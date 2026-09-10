#!/usr/bin/env bash
# Start/stop the packaged CARLA server by PID with RPC readiness wait. Never start on a busy port.
set -o pipefail
BIN=${CARLA_BIN:-/mnt/dsk0/wk0/CARLA/T4Fork.RGL/CarlaUE5/Build/Package/Carla-0.10.0-Linux-Shipping/Linux/CarlaUnreal.sh}
PORT=${CARLA_PORT:-2000}
cmd=$1; label=$2; shift 2 || true
pidf=/tmp/carla-${label}.pid; logf=/tmp/carla-${label}.log
port_busy() { ss -ltn 2>/dev/null | grep -q ":${PORT} "; }
case "$cmd" in
  start)
    port_busy && { echo "port ${PORT} busy; stop the other server first"; exit 1; }
    "$BIN" -RenderOffScreen -nosound -carla-rpc-port=${PORT} "$@" >"$logf" 2>&1 &
    echo $! >"$pidf"; echo "started pid=$(cat $pidf) log=$logf args=$*"
    for i in $(seq 1 180); do
      python3 - <<PY 2>/dev/null && { echo "rpc ready after ${i}s"; exit 0; }
import carla; c=carla.Client("127.0.0.1", ${PORT}); c.set_timeout(2.0); c.get_server_version()
PY
      sleep 1; kill -0 "$(cat $pidf)" 2>/dev/null || { echo "server died; see $logf"; exit 1; }
    done
    echo "rpc not ready after 180s"; exit 1 ;;
  stop)
    [ -f "$pidf" ] || { echo "no pid file"; exit 0; }
    pid=$(cat "$pidf"); pkill -TERM -P "$pid" 2>/dev/null; kill -TERM "$pid" 2>/dev/null
    for i in $(seq 1 60); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
    kill -0 "$pid" 2>/dev/null && { kill -KILL "$pid"; pkill -KILL -P "$pid" 2>/dev/null; }
    for i in $(seq 1 30); do port_busy || break; sleep 1; done
    port_busy && { echo "port still busy"; exit 1; }
    rm -f "$pidf"; echo "stopped ${label}" ;;
  status) port_busy && echo "port ${PORT} busy" || echo "port ${PORT} free"; ls /tmp/carla-*.pid 2>/dev/null ;;
  *) echo "usage: $0 start|stop|status <label> [ue args]"; exit 2 ;;
esac
