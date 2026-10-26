# j5mini — controller minimale del braccio (Pi o Aura)

Un solo processo Python (~450 righe) al posto di ws-teleop + ponte SPI (o del container ROS 2) per la
**teleoperazione**: stessi messaggi WebSocket di ws-teleop verso l'app Quest e il viewer web, SPI e
UART verso lo STM32 come fa il Pi oggi.

| Fa | Come |
|---|---|
| Setpoint al braccio | SPI master 100 Hz, frame J5VR dall'ultimo intent; dopo 150 ms senza intent payload neutro (stesso modo e heartbeat, deadman rilasciato) |
| Diagnostica | una richiesta STATUS ogni 20 trasferimenti (5 Hz): deadman, armato, freeze, E-STOP, bypass |
| Comandi di stato | UART 115200 con `#seq`; risposte inoltrate ai client come `uart_response` |
| Configurazione | su `BOOT_READY` (o 1,5 s dopo l'avvio) invia **le stesse righe del Pi**: `SET_OFFSETS`, `SET_VR_PARAMS` (3 tentativi), `SET_JOINT_LIMITS`, `SET_PWM_CONFIG`, `IMUON` |
| Ingresso in HEAD/HYBRID/ASSIST | rimanda `SET_VR_PARAMS` (una volta ogni 2,5 s), come ws-teleop |
| Stato robot | `STATUS?` ogni 2 s |
| Telemetria ai client | 20 Hz: `robot_state`, `servo_deg_B..R`, quaternione IMU, `deadman_active`, `vr_armed`, `estop_active`, `estop_bypass`, contatori del link |
| Orologio | `time_sync` dall'app → `sudo -n date` se sfasato > 2 s (`set_clock`) |

**Non fa** (apposta): target ASSIST del gimbal (in modo 5 B/S/G restano fermi), IK/FK e twin, pagine
della dashboard, video, vision, collimatore, ROS 2, captive portal, demo e self-test IMU.

## Configurazione

`j5mini.json` si genera **sul Pi** con il codice del Pi, così le righe sono identiche a quelle di oggi:

```bash
~/raspberry5/.venv/bin/python3 export_config.py ~/raspberry5 > j5mini.json
```

`j5mini.example.json` è lo stesso file generato dalla copia di `config_runtime` nel repo (non dal Pi).
Sull'Aura: `"spi": {"device": "/dev/spidev1.0"}`, `"uart": {"device": "/dev/ttyS4"}` e il sender avviato
**senza** `--spi`/`--uart` (un solo proprietario per bus).

## Avvio

```bash
sudo systemctl stop jonny5-ws-teleop jonny5-spi-j5vr jonny5-spi-shm-bridge jonny5-ros2   # quelli presenti
python3 j5mini.py --config j5mini.json
```

Dipendenze: `websockets`, `spidev`, `pyserial` (già nel venv del Pi). Unit di esempio: `j5mini.service`.

## Prove senza hardware

```bash
python -m pytest minimal/j5mini -q      # 7 test contro il firmware vero simulato (firmware/stm32/tools/fw_sim)
```

Coprono: push della configurazione su `BOOT_READY` (tutte OK), armo automatico, HOME, manuale con
deadman, neutro dopo il silenzio del visore (uscite a 0), reinvio di `SET_VR_PARAMS` in HEAD, bit e
righe E-STOP, trasporto a 64 byte con firmware a 64, validazione degli intent come ws-teleop, e un
client WebSocket reale (come l'app) fino al firmware simulato.
