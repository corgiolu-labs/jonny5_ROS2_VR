#!/usr/bin/env bash
# j5_net_autoswitch.sh v5
# Forza sempre il profilo corretto quando rileva la subnet, indipendentemente
# da quale profilo NM sia attivo in quel momento.
set -u

LOG_TAG="J5-NET-AUTOSWITCH"
log() { echo "[$LOG_TAG] $*" | systemd-cat -t "$LOG_TAG"; }

current_mode="unknown"
no_ip_count=0
gw_fail_count=0

log "Demone v5 avviato."

while true; do

    # ── 1. Carrier fisico ────────────────────────────────────────────────
    carrier=0
    [[ -r /sys/class/net/eth0/carrier ]] && carrier=$(cat /sys/class/net/eth0/carrier 2>/dev/null || echo 0)

    if [[ "$carrier" != "1" ]]; then
        if [[ "$current_mode" != "no-link" ]]; then
            log "eth0: cavo scollegato. Attendo."
            current_mode="no-link"
            no_ip_count=0
            gw_fail_count=0
        fi
        sleep 5
        continue
    fi

    # ── 2. IP attuale su eth0 ────────────────────────────────────────────
    ip4=$(ip -4 addr show dev eth0 | awk '/inet / {print $2}' | head -n1 || true)

    # ── 3. Profilo NM attivo su eth0 ─────────────────────────────────────
    active_conn=$(nmcli -g GENERAL.CONNECTION device show eth0 2>/dev/null || echo "unknown")

    # ── 4. Logica di rilevamento e switch ────────────────────────────────

    if [[ "$ip4" == 192.168.10.* ]]; then
        # Subnet router casa
        if ping -c1 -W2 192.168.10.1 >/dev/null 2>&1; then
            gw_fail_count=0
            no_ip_count=0
            if [[ "$current_mode" != "router" ]]; then
                log "Rilevato router casa (eth0=$ip4, gw ok). Attivo eth0-dhcp."
                nmcli connection up eth0-dhcp >/dev/null 2>&1 || log "Errore eth0-dhcp"
                current_mode="router"
            fi
        else
            gw_fail_count=$((gw_fail_count + 1))
            if [[ $gw_fail_count -ge 2 ]]; then
                log "IP router ($ip4) ma gw 192.168.10.1 non risponde. Provo ICS."
                nmcli connection up eth0-ics-static >/dev/null 2>&1 || log "Errore eth0-ics-static"
                current_mode="unknown"
                gw_fail_count=0
            fi
        fi

    elif [[ "$ip4" == 192.168.137.* ]]; then
        # Subnet ICS - SEMPRE forza eth0-ics-static se non e' gia' attivo
        if ping -c1 -W2 192.168.137.1 >/dev/null 2>&1; then
            gw_fail_count=0
            no_ip_count=0
            if [[ "$current_mode" != "ics" ]]; then
                log "Rilevata subnet ICS (eth0=$ip4, gw ok). Forzo profilo eth0-ics-static."
                nmcli connection up eth0-ics-static >/dev/null 2>&1 || log "Errore eth0-ics-static"
                current_mode="ics"
            elif [[ "$active_conn" != "eth0-ics-static" ]]; then
                # Profilo sbagliato attivo (es. eth0-dhcp ha preso IP ICS via DHCP Windows)
                log "Subnet ICS ma profilo attivo e' '$active_conn'. Correggo con eth0-ics-static."
                nmcli connection up eth0-ics-static >/dev/null 2>&1 || log "Errore eth0-ics-static"
            fi
        else
            gw_fail_count=$((gw_fail_count + 1))
            if [[ $gw_fail_count -ge 2 ]]; then
                log "IP ICS ($ip4) ma gw 192.168.137.1 non risponde. Provo DHCP router."
                nmcli connection up eth0-dhcp >/dev/null 2>&1 || log "Errore eth0-dhcp"
                current_mode="unknown"
                gw_fail_count=0
            fi
        fi

    else
        # Carrier UP ma nessun IP noto su eth0
        gw_fail_count=0
        no_ip_count=$((no_ip_count + 1))

        if [[ $no_ip_count -eq 2 ]]; then
            log "eth0: nessun IP da $no_ip_count cicli (~10s). Attivo eth0-ics-static."
            nmcli connection up eth0-ics-static >/dev/null 2>&1 || log "Errore eth0-ics-static"
        elif [[ $no_ip_count -eq 6 ]]; then
            log "eth0: nessun IP da $no_ip_count cicli (~30s). Riprovo eth0-dhcp."
            nmcli connection up eth0-dhcp >/dev/null 2>&1 || log "Errore eth0-dhcp"
        elif [[ $no_ip_count -ge 12 ]]; then
            log "eth0: nessun IP da $no_ip_count cicli (~60s). Riprovo eth0-ics-static."
            nmcli connection up eth0-ics-static >/dev/null 2>&1 || log "Errore eth0-ics-static"
            no_ip_count=0
        fi
    fi

    sleep 5
done
