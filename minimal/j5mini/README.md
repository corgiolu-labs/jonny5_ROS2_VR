# j5mini — controller del braccio (Pi o Aura)

Un solo processo Python al posto di ws-teleop, del ponte SPI, del container ROS 2 e dei server HTTPS:
parla con l'app Quest come ws-teleop e con lo STM32 come il Pi.

| Fa | Come |
|---|---|
| Setpoint al braccio | SPI master 100 Hz, 64 byte, frame J5VR dall'ultimo intent; dopo 150 ms senza intent payload neutro (stesso modo e heartbeat, deadman rilasciato) |
| Diagnostica | una richiesta STATUS ogni 20 trasferimenti (5 Hz): deadman, armato, freeze, E-STOP, bypass |
| Movimento fluido | in HEAD/HYBRID/ASSIST il quaternione della testa e i target B/S/G dell'ASSIST vengono rigiocati `smoothing_ms` (default 30) indietro, interpolati sui timestamp del visore: l'app manda 60 intent/s da un ciclo a 90 Hz (11 e 22 ms) e il Wi-Fi aggiunge jitter (misurato 8-28 ms), il firmware segue ogni target in pochi ms e il braccio andava a scatti. Grip, pulsanti e stick restano immediati. `0` lo spegne; `smoothed` in telemetria |
| Risposte SPI | cercate negli ultimi due trasferimenti: il DMA dello STM32 può essere sfasato di qualche byte rispetto al CS (sul robot 3 byte); `spi_shift` in telemetria |
| Comandi di stato | UART 115200 con `#seq`; risposte inoltrate ai client come `uart_response` |
| Configurazione | su `BOOT_READY` (o 1,5 s dopo l'avvio) invia **le stesse righe del Pi**: `SET_OFFSETS`, `SET_VR_PARAMS` (3 tentativi), `SET_JOINT_LIMITS`, `SET_PWM_CONFIG` (la IMU è sempre accesa) |
| Ingresso in HEAD/HYBRID/ASSIST | rimanda `SET_VR_PARAMS` (una volta ogni 2,5 s), come ws-teleop |
| ASSIST (modo 5) | il gimbal del vecchio Pi (`assist_gimbal.py`, serve `numpy`): la camera segue la testa 1:1, base, spalla e gomito aiutano il polso (e seguono la posizione della testa), predizione del movimento e correzione con la IMU; target B/S/G nel frame J5VR (byte 36-45, marcatore `I`). Grip rilasciati: fermo; ripremuti: nuovi riferimenti, nessun salto. Parametri in `"assist"` (default = quelli del robot); `"assist": {"mode": "off"}` lo spegne |
| Orizzonte | `{"type": "horizon_lock", "on": true}` (tasto "Orizzonte" dell'app), **in tutti i modi**: j5mini manda al firmware `HORIZON 1 <versi dei giunti> <trim>`, il firmware porta il roll del polso al valore che tiene l'immagine a livello per la posa reale di B/S/G/Y/P (il roll gira attorno all'asse ottico, la camera guarda sempre dove la porta il modo); a braccio fermo j5mini confronta la verticale della IMU del polso (montaggio da `imu_ee_mount.json`) e corregge il trim a passi (±8°). Rimandato dopo un reset dello STM32; in ASSIST anche il gimbal livella il suo target. `horizon_lock` / `horizon_trim` in telemetria, default `"horizon_lock": false` |
| Punta l'oggetto | `vision_result` con `aim` (grilletto destro in ASSIST) → il gimbal porta la camera sull'oggetto, esito a tutti come `vision_point`; arrivato, chiede una nuova analisi da vicino. Alla fine il polso rientra nell'inseguimento in 0,6 s e l'aiuto del braccio riparte in 2 s (prima scattava subito); l'aiuto della base/gomito parte in modo graduale oltre la banda del polso (`BAND_SOFT_DEG`) |
| Ritardi braccio/polso | dalla campagna IMU del 26/10: la base risponde in ~0,09 s, il polso in ~0,17 s. Il polso viene risolto sul braccio *voluto* e il comando B/S/G esce in ritardo di 0,08 s (`ARM_DELAY_S`), così arrivano alla camera insieme (prima la camera seguiva la base per ~0,2 s a ogni partenza) |
| Campagna IMU | `{"type": "imu_campaign", "on": true, "label": "..."}` registra desiderato/comandato/firmware/IMU in `logs/imu_campaign/*.jsonl` (per misurare gli scatti) |
| Stato robot | `STATUS?` ogni 2 s |
| PARK automatico | visore in un modo di teleop (2-5) con i grip rilasciati per `settings.auto_park_s` secondi (default 60, 0 = mai): j5mini manda `PARK` da solo, una volta per periodo di inattività, se il robot è IDLE, nessuna demo in corso e il braccio non è già lì. Ripremere i grip o uscire dai modi azzera il conto. `auto_park_in` in telemetria (secondi restanti), messaggio `auto_park` ai client |
| Posa iniziale | a ogni `BOOT_READY` del firmware, dopo le righe di configurazione, manda `SETPOSE_INIT <gradi fisici>`: l'ultima posa vista in telemetria se il firmware si è resettato con j5mini in funzione, altrimenti (accensione) la posa PARK, dove il braccio riposa da spento. Il firmware la prende come posa in cui il braccio si trova (nessun movimento), così il primo HOME / VR pose è una traiettoria lenta da lì invece di un salto dei servo. Non viene mandata quando è j5mini a riavviarsi con il firmware già posizionato; se però il firmware riporta 90 su tutti i giunti (acceso prima del Pi, che impiega un minuto: BOOT_READY perso) la manda lo stesso. Regola: **prima di spegnere, PARK** |
| Demo | `{"type": "demo", "on": true}` dalla dashboard, sulle pose di `settings.demo` (posa virtuale, velocità %, profilo; default la vecchia `demo_steps` del Pi). `settings.demo_style` `"smooth"` (default dal 20/11/2026): una curva continua che passa per le pose senza fermarsi (PCHIP per giunto, mai oltre l'escursione delle pose, ogni tratto dura quanto il suo `SETPOSE`) più un'oscillazione del roll di `settings.demo_roll_deg` (default 20°, ridotta allo spazio del giunto, periodo 9 s), mandata come `SETPOSE_T_HR` da 40 ms 25 volte al secondo; parte e si ferma rallentando (1,5 s). `"steps"`: i passi come `SETPOSE` uno alla volta, il successivo a `SETPOSE_DONE`, in ciclo. Si ferma con `on: false` (fermandosi dov'è), quando il robot esce da IDLE (STOP, SAFE, fungo), se un passo è rifiutato o resta senza `SETPOSE_DONE` per 15 s. `demo` in telemetria (`{step, n}` o `null`), messaggi `demo` con `on`/`reason` |
| Segui | `{"type": "track", "on": true, "x", "y"}` dalla dashboard (x/y in pixel dell'occhio sinistro 1920×1080, default il centro): prende il box YOLO più piccolo che contiene il punto (altrimenti il più vicino entro 200 px) fra quelli arrivati dalla testa stereo nell'ultimo secondo, e muove yaw e pitch del polso (`settings.track.joints`) per tenerlo al centro. Ogni box diventa una direzione assoluta: la posa comandata `lag_s` prima dell'istante di cattura (il `ts_us` del sender) più lo scarto dal centro diviso `px_per_deg` (default `[17.1, -17.1]`, misurato sul robot il 21/11/2026). Un filtro alfa-beta ne stima la velocità e il comando punta dove l'oggetto sarà fra latenza + `lag_s`, con al massimo 35°/s e 150°/s², una zona morta di 1° e `SETPOSE_T_HR` da 40 ms come la demo fluida. Associazione: stessa classe, il box più vicino alla posizione attesa. Box tagliati dal bordo (`edge_px`): un asse tagliato da entrambi i lati non misura niente, da un lato si ricostruisce dal lato visibile e dall'ultima dimensione intera (`size_memory_s`); la velocità cambia al massimo di `max_dv_deg_s` per rilevazione, non si aggiorna se il box cambia area di oltre il 40 % e la previsione usa |v| − `v_dead_deg_s` (contro l'avanti-indietro, 21/11/2026). La base (`base_joint`, default 0) entra quando lo yaw del polso è a più di `base_dead_deg` (15°) dal centro: gira al massimo a `base_max_deg_s` (20°/s) per riportare il polso verso il centro, e il polso compensa. Quanto vale ogni giunto in pixel e la base in gradi di polso si calcola dalla posa con `dashboard/kinematics.json` (`camera_axes_coefficients`: 1 in HOME, yaw del polso 0,77 al parcheggio); senza cinematica valgono `px_per_deg` e `base_gain`. In telemetria `track.base`, `track.scale`, `track.base_gain`. Perso da 0,8 s: fermo dov'è; da 8 s: smette. Si ferma con `on: false` (trattiene la posa), fuori da IDLE o con il deadman premuto. Demo e auto-park bloccati mentre segue. `track` in telemetria (`state` segue/perso, `cls`, `box`, `err_deg`, `speed_deg_s`, `lost_ms`), messaggi `track` con `on`/`reason`; `npu_detections` ha `age_ms` |
| PARK / VR pose | `PARK` e `TELEOPPOSE` dai client diventano un `SETPOSE` alle pose salvate (`settings.park` / `settings.vr`, gradi virtuali, default quelle del robot; velocità `speed_pct`, default 40 % come HOME), come ws-teleop: le PARK/TELEOPPOSE del firmware vanno tutte e due a una posa di fabbrica. Risposta con il comando del client e `teleop_pose_ack` (l'app del visore lo aspetta fino a 5 s) |
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

**Non fa** (apposta): twin, pagine della dashboard del vecchio Pi, ROS 2, captive portal, self-test
IMU. Altri messaggi sono ignorati con una riga di log.

## Configurazione

`j5mini.json` (non versionato, accanto a `j5mini.py`). I percorsi relativi partono dalla sua cartella.
`raspberry/setup_pi.sh` lo crea da `j5mini.example.json` se manca; offset, versi, limiti, pose PARK/VR, velocità
e parametri VR si modificano poi dalla dashboard (Impostazioni e Teleop → "Salva in j5mini.json", con copia `.bak`).

Sull'Aura: `"spi": {"device": "/dev/spidev1.0"}`, `"uart": {"device": "/dev/ttyS4"}` e il sender avviato
**senza** `--spi`/`--uart` (un solo proprietario per bus).

## Avvio

Installato come servizio da `raspberry/setup_pi.sh` (`jonny5-j5mini`). A mano:

```bash
python3 j5mini.py --config j5mini.json
```

Dipendenze: `websockets`, `spidev`, `pyserial` (pacchetti Debian `python3-websockets`, `python3-spidev`,
`python3-serial`); `numpy` per l'ASSIST (senza, il modo 5 muove solo il polso).

## Prove senza hardware

```bash
cd firmware/stm32 && python -m pytest ../../minimal/j5mini -q      # firmware vero simulato
```

Coprono: push della configurazione su `BOOT_READY` (tutte OK), armo automatico, HOME, manuale con
deadman, neutro dopo il silenzio del visore (uscite a 0), reinvio di `SET_VR_PARAMS` in HEAD, bit e
righe E-STOP, trasporto a 64 byte (default) e a 128 con firmware a 128, validazione degli intent come
ws-teleop, comandi telecamere e zoom residuo, le due API HTTPS, e un client WebSocket reale (come
l'app) fino al firmware simulato, con `get_settings`; inoltro fra app e servizi opzionali; modo 0 della
PREPARAZIONE dell'app; impostazioni della dashboard (andata e ritorno, validazione, salvataggio con `.bak`),
pagine statiche senza uscire dalla cartella, backend `--sim`; ASSIST: la testa muove anche la base, grip
rilasciati fermo, ripresi senza salto.
