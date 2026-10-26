# j5mini — controller del braccio (Pi o Aura)

Un solo processo Python al posto di ws-teleop, del ponte SPI, del container ROS 2 e dei server HTTPS:
parla con l'app Quest come ws-teleop e con lo STM32 come il Pi.

| Fa | Come |
|---|---|
| Setpoint al braccio | SPI master 100 Hz, 64 byte, frame J5VR dall'ultimo intent; dopo 150 ms senza intent payload neutro (stesso modo e heartbeat, deadman rilasciato) |
| Diagnostica | una richiesta STATUS ogni 20 trasferimenti (5 Hz): deadman, armato, freeze, E-STOP, bypass |
| Risposte SPI | cercate negli ultimi due trasferimenti: il DMA dello STM32 può essere sfasato di qualche byte rispetto al CS (sul robot 3 byte); `spi_shift` in telemetria |
| Comandi di stato | UART 115200 con `#seq`; risposte inoltrate ai client come `uart_response` |
| Configurazione | su `BOOT_READY` (o 1,5 s dopo l'avvio) invia **le stesse righe del Pi**: `SET_OFFSETS`, `SET_VR_PARAMS` (3 tentativi), `SET_JOINT_LIMITS`, `SET_PWM_CONFIG` (la IMU è sempre accesa) |
| Ingresso in HEAD/HYBRID/ASSIST | rimanda `SET_VR_PARAMS` (una volta ogni 2,5 s), come ws-teleop |
| Stato robot | `STATUS?` ogni 2 s |
| Telemetria ai client | 20 Hz: `robot_state`, `servo_deg_B..R`, quaternione IMU, `deadman_active`, `vr_armed`, `estop_active`, `estop_bypass`, contatori del link, stato telecamere |
| Telecamere (app Quest) | `vr_zoom_state`, `cam_exposure`, `cam_mode`, `cam_focus` → `raspberry/tools/cam_stream.py` via UDP 5600/5601 come ws-teleop (zoom e luminosità rimandati ogni secondo); in telemetria `cam_zoom`, `cam_mode`, `cam_hfov`, `cam_fps`, `cam_ev`, `vr_zoom0/1` |
| Impostazioni | `get_settings` → `offsets`/`dirs` (barre dei giunti dell'app) |
| HTTPS 443 | `GET /api/controller-mappings` (mappatura dei pulsanti; l'app ha una copia interna) e `POST /api/refocus-cameras` (autofocus) |
| Inoltro | `cameras_refocus_triggered`, `vr_session_state`, `vr_zoom_command` agli altri client |
| Servizi opzionali | collimatore e visione (`raspberry/tools/`) sono client del WebSocket come l'app: j5mini inoltra `stereo_collimation`, `stereo_calibrate`, `vision_request`, `vision_result` e scrive gli angoli dei servo in `/dev/shm/j5vr_telemetry.json` (`telemetry_file`) per la visione |
| Orologio | `time_sync` dall'app → `sudo -n date` se sfasato > 2 s (`set_clock`) |
| Dashboard | `minimal/dashboard` servita sulla porta HTTPS (`https.dashboard`); `get_config` / `save_config` (offset, versi, limiti, righe VR e PWM → `j5mini.json` con `.bak`), `ws_ping`; in telemetria tutti i bit di diagnostica e il numero di client |
| Video della dashboard | `POST /whep/cam0` e `/whep/cam1` sulla porta HTTPS inoltrati al WHEP di MediaMTX (`https.whep`, default `http://127.0.0.1:8889`), così una pagina https può aprire il WebRTC; `cam_autofocus` = autofocus su tutte e due le camere |
| Simulatore | `--sim`: il firmware vero compilato per il PC al posto di SPI e UART, dashboard su `http://localhost:8558` (`--http-port`) |

**Non fa** (apposta): target ASSIST del gimbal (in modo 5 B/S/G restano fermi; il "punta" della visione
risponde `vision_point` con `ok: false`), IK/FK e twin, pagine della dashboard, ROS 2, captive portal,
demo e self-test IMU. Altri messaggi sono ignorati con una riga di log.

## Configurazione

`j5mini.json` (non versionato, accanto a `j5mini.py`). I percorsi relativi partono dalla sua cartella.
`raspberry/setup_pi.sh` lo crea da `j5mini.example.json` se manca, ma con offset e limiti di default:
sul robot con lo stack vecchio ancora installato conviene esportare quelli calibrati, con il codice del
Pi stesso (righe identiche a quelle di oggi):

```bash
cd ~/JONNY5_ROS2/minimal/j5mini            # la cartella di questo file nel checkout del Pi
~/raspberry5/.venv/bin/python3 export_config.py ~/raspberry5 > j5mini.json
cp ~/raspberry5/config_runtime/robot/controller_mappings.json .
```

Sull'Aura: `"spi": {"device": "/dev/spidev1.0"}`, `"uart": {"device": "/dev/ttyS4"}` e il sender avviato
**senza** `--spi`/`--uart` (un solo proprietario per bus).

## Avvio

Installato come servizio da `raspberry/setup_pi.sh` (`jonny5-j5mini`). A mano:

```bash
python3 j5mini.py --config j5mini.json
```

Dipendenze: `websockets`, `spidev`, `pyserial` (pacchetti Debian `python3-websockets`, `python3-spidev`,
`python3-serial`).

## Prove senza hardware

```bash
cd firmware/stm32 && python -m pytest ../../minimal/j5mini -q      # 21 test, firmware vero simulato
```

Coprono: push della configurazione su `BOOT_READY` (tutte OK), armo automatico, HOME, manuale con
deadman, neutro dopo il silenzio del visore (uscite a 0), reinvio di `SET_VR_PARAMS` in HEAD, bit e
righe E-STOP, trasporto a 64 byte (default) e a 128 con firmware a 128, validazione degli intent come
ws-teleop, comandi telecamere e zoom residuo, le due API HTTPS, e un client WebSocket reale (come
l'app) fino al firmware simulato, con `get_settings`; inoltro fra app e servizi opzionali; modo 0 della
PREPARAZIONE dell'app; impostazioni della dashboard (andata e ritorno, validazione, salvataggio con `.bak`),
pagine statiche senza uscire dalla cartella, backend `--sim`.
