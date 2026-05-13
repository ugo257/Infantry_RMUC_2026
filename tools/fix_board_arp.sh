#!/usr/bin/env bash
set -euo pipefail

DEFAULT_IPS=("192.168.1.12" "192.168.1.14")
IPS=()
IFACE=""
WATCH=0
DRY_RUN=0
PING_AFTER=1
INTERVAL=5
PING_COUNT=2
ORIG_ARGS=("$@")

usage() {
  cat <<'EOF'
Usage: tools/fix_board_arp.sh [options]

Fix stale macOS ARP/host-route entries for the robot boards.

Options:
  --ip ADDR          Board IP to check. Can be used more than once.
  --iface IFACE      Network interface to use, for example en0.
  --watch            Keep checking and fixing in a loop.
  --interval SEC     Watch interval in seconds. Default: 5.
  --no-ping          Do not ping after cleanup.
  --dry-run          Show what would be changed.
  -h, --help         Show this help.

Default board IPs:
  192.168.1.12       Chassis board
  192.168.1.14       Gimbal board
EOF
}

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

log() {
  printf '%s\n' "$*"
}

normalize_mac() {
  local raw mac
  raw="${1:-}"
  mac="$(printf '%s' "$raw" | tr '[:upper:]' '[:lower:]')"

  local parts
  IFS=':' read -r -a parts <<<"$mac"
  if [[ ${#parts[@]} -ne 6 ]]; then
    printf '%s' "$mac"
    return
  fi

  local p
  for p in "${parts[@]}"; do
    if ! [[ "$p" =~ ^[0-9a-f]{1,2}$ ]]; then
      printf '%s' "$mac"
      return
    fi
  done

  printf '%02x:%02x:%02x:%02x:%02x:%02x' \
    "0x${parts[0]}" "0x${parts[1]}" "0x${parts[2]}" \
    "0x${parts[3]}" "0x${parts[4]}" "0x${parts[5]}"
}

run_or_print() {
  if [[ "$DRY_RUN" -eq 1 ]]; then
    printf '[dry-run]'
    printf ' %q' "$@"
    printf '\n'
    return 0
  fi

  "$@"
}

run_ignore_error() {
  if run_or_print "$@"; then
    return 0
  fi
  return 0
}

detect_iface() {
  local ip iface
  ip="$1"
  if [[ -n "$IFACE" ]]; then
    printf '%s' "$IFACE"
    return
  fi

  iface="$(route -n get "$ip" 2>/dev/null | awk '/interface:/{print $2; exit}')"
  if [[ -n "$iface" ]]; then
    printf '%s' "$iface"
    return
  fi

  printf 'en0'
}

local_mac_for_iface() {
  local iface
  iface="$1"
  ifconfig "$iface" 2>/dev/null | awk '/ether /{print $2; exit}'
}

route_line_for_ip() {
  local ip
  ip="$1"
  netstat -rn -f inet 2>/dev/null | awk -v ip="$ip" '$1 == ip || $1 == ip "/32" {print; exit}'
}

arp_line_for_ip() {
  local ip
  ip="$1"
  arp -n "$ip" 2>/dev/null || true
}

is_suspicious_entry() {
  local ip iface local_mac arp_line arp_mac arp_mac_norm route_line route_gateway route_gateway_norm route_flags
  ip="$1"
  iface="$2"
  local_mac="$3"

  arp_line="$(arp_line_for_ip "$ip")"
  arp_mac="$(awk '/ at /{print $4; exit}' <<<"$arp_line")"
  arp_mac_norm="$(normalize_mac "$arp_mac")"

  route_line="$(route_line_for_ip "$ip")"
  route_gateway="$(awk '{print $2; exit}' <<<"$route_line")"
  route_gateway_norm="$(normalize_mac "$route_gateway")"
  route_flags="$(awk '{print $3; exit}' <<<"$route_line")"

  [[ -n "$arp_line" && "$arp_line" == *"permanent"* ]] && return 0
  [[ -n "$local_mac" && "$arp_mac_norm" == "$local_mac" ]] && return 0
  [[ -n "$local_mac" && "$route_gateway_norm" == "$local_mac" ]] && return 0
  [[ "$route_flags" == *S* && "$route_flags" == *H* ]] && return 0

  return 1
}

describe_entry() {
  local ip iface local_mac arp_line route_line
  ip="$1"
  iface="$2"
  local_mac="$3"

  arp_line="$(arp_line_for_ip "$ip")"
  route_line="$(route_line_for_ip "$ip")"

  log "[$ip] interface: $iface, local MAC: ${local_mac:-unknown}"
  if [[ -n "$arp_line" ]]; then
    log "[$ip] arp: $arp_line"
  else
    log "[$ip] arp: no entry"
  fi
  if [[ -n "$route_line" ]]; then
    log "[$ip] route: $route_line"
  else
    log "[$ip] route: no host route"
  fi
}

cleanup_ip() {
  local ip iface local_mac
  ip="$1"
  iface="$(detect_iface "$ip")"
  local_mac="$(normalize_mac "$(local_mac_for_iface "$iface")")"

  describe_entry "$ip" "$iface" "$local_mac"

  if is_suspicious_entry "$ip" "$iface" "$local_mac"; then
    log "[$ip] stale entry detected; deleting ARP and host route"
    run_ignore_error arp -d "$ip"
    run_ignore_error route -n delete "$ip"
    run_ignore_error arp -d "$ip"
  else
    log "[$ip] no stale permanent/self-MAC entry detected"
  fi

  if [[ "$PING_AFTER" -eq 1 ]]; then
    if ping -c "$PING_COUNT" -W 1000 "$ip" >/dev/null 2>&1; then
      log "[$ip] ping OK"
    else
      log "[$ip] ping still failed; check board power, cable/Wi-Fi bridge, IP, and firmware"
    fi
  fi
}

require_root_if_needed() {
  if [[ "$DRY_RUN" -eq 1 ]]; then
    return
  fi
  if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
    return
  fi

  local self
  self="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
  if [[ ${#ORIG_ARGS[@]} -gt 0 ]]; then
    exec sudo "$self" "${ORIG_ARGS[@]}"
  fi
  exec sudo "$self"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ip)
      [[ $# -ge 2 ]] || die "--ip needs an address"
      IPS+=("$2")
      shift 2
      ;;
    --iface)
      [[ $# -ge 2 ]] || die "--iface needs an interface name"
      IFACE="$2"
      shift 2
      ;;
    --watch)
      WATCH=1
      shift
      ;;
    --interval)
      [[ $# -ge 2 ]] || die "--interval needs seconds"
      INTERVAL="$2"
      shift 2
      ;;
    --no-ping)
      PING_AFTER=0
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
done

if [[ ${#IPS[@]} -eq 0 ]]; then
  IPS=("${DEFAULT_IPS[@]}")
fi

require_root_if_needed

if [[ "$(uname -s)" != "Darwin" ]]; then
  die "this script uses macOS arp/route syntax"
fi

while :; do
  log "== board ARP cleanup $(date '+%Y-%m-%d %H:%M:%S') =="
  for ip in "${IPS[@]}"; do
    cleanup_ip "$ip"
  done

  if [[ "$WATCH" -ne 1 ]]; then
    break
  fi
  sleep "$INTERVAL"
done
