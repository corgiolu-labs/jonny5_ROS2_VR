# JONNY5 minimale: inventario delle funzioni, tagli proposti, nuova dashboard

Data: 27/10/2026, dopo le prove sull'hardware del 26/10 (j5mini 4eeaa92, firmware 10c6560, app Quest 1.4.4, app telefono 0.3).
Scopo: elencare tutto ciò che fa il firmware STM32 e il lato Raspberry, dire cosa è morto o doppio, e progettare la
dashboard definitiva (PC + telefono). Nessuna modifica al codice è stata fatta con questo documento: ogni taglio è
una proposta da approvare.

## 0. Sintesi

| Parte | Dimensione oggi | Morto / doppio | Taglio proposto |
|---|---|---|---|
| Firmware STM32 (`firmware/stm32/src`, 33 file) | 7.645 righe | ~440 righe + 430 nel simulatore (`tools/fw_sim/orig`) | ~6 % del sorgente, 2 comandi UART, 1 modo, 1 bug (bit "guard" appiccicoso) |
| Raspberry (`minimal/j5mini` + `raspberry/tools` + script/servizi) | 8.792 righe (Python 4.947) | ~250 righe sicure + 520 opzionali (via RTP diretta) | funzioni non chiamate, tool di migrazione, rami di script mai usati |
| Dashboard nuova | 1.735 righe, 75 KB, zero librerie | — | da riprogettare: mancano 6 cose essenziali della vecchia |
| Dashboard vecchia (`JONNY5_ROS2/web`) | ~22.700 righe, 1,66 MB (three.js + Chart.js) | — | riferimento per stile e funzioni |

Verdetto: lo stack minimale è già pulito. Il grosso del lavoro utile non è tagliare ma **rimettere nella dashboard le
sei funzioni essenziali della vecchia** e **rifarla per PC e telefono** con un solo certificato e una palette coerente
con l'app.

## 1. Firmware STM32 (Zephyr, STM32G474)

### 1.1 Moduli e funzioni pubbliche

| File | Righe | Cosa fa | Funzioni pubbliche |
|---|---|---|---|
| `core/main.c` | 48 | boot, loop principale 10 ms (UART, notifiche) | `main` |
| `core/state_machine.c` | 72 | 3 stati SAFE / IDLE / STOPPED | `init`, `get_state`, `set_safe`, `set_idle` (solo da SAFE), `set_stopped`, `is_movement_allowed` |
| `core/estop.c` | 331 | fungo su PC6: debounce 10 ms, latch STOPPED, bypass a tempo, notifiche al Pi | `estop_init`, `estop_poll_tick` (1 kHz), `estop_is_active`, `estop_line_active`, `estop_bypass_set/active/remaining_s`, `estop_notify_poll` |
| `core/fatal_diag.c` | 27 | handler fatal Zephyr, conta i crash | `k_sys_fatal_error_handler` |
| `core/rt_loop.c` | 875 | thread RT 1 kHz su TIM6: watchdog SPI, stati, armamento VR, freeze, cambio modo, pulsante PC13; thread IMU 400 Hz | `rt_loop_init`, `rt_loop_start` |
| `imu/imu.c` | 355 | BNO085: recupero bus I2C, scan, snapshot double-buffer | `imu_init`, `imu_update_orientation`, `imu_get_quaternion`, `imu_get_snapshot`, `imu_i2c_bus_recovery`, `imu_is_orientation_valid`, `imu_is_available` |
| `imu/bno085.c` | 324 | driver SHTP minimo (rotation vector 100 Hz) | `bno085_init`, `bno085_poll_quat`, `bno085_is_ready` |
| `servo/j5vr_actuation.c` | 996 | stato giunti condiviso, limitatore di velocità + filtro, Orizzonte, interpolazione tra frame SPI, modi MANUAL/HYBRID/ASSIST | `update_max_vel_buttons`, `set_wrist_vel_head`, `set_arm_vel_head`, `apply_desired`, `horizon_set`, `horizon_is_on`, `init`, `center_all_servos` (HOME), `go_teleop_pose`, `smooth_between_frames`, `set_joint_limits`, `apply_setpoint_incremental` (modo 2), `apply_mode5_arm_head` (modo 5), `apply_hybrid` (modo 4) |
| `servo/j5vr_head.c` | 392 | head tracking: errore di quaternione → yaw/pitch/roll, EMA, parametri VR | `init_defaults`, `get_params`, `get_state`, `reset_head_calib`, `set_vr_params` (34 argomenti), `apply_head_tracking` (modo 3), `apply_head_orientation_only` (modi 4/5) |
| `servo/j5vr_manual.c` | 37 | stick int16 → float con zona morta | `int16_to_normalized`, `_dz` |
| `servo/j5vr_quat_utils.c` | 114 | algebra dei quaternioni | `from_j5vr`, `normalize`, `multiply`, `conjugate`, `to_twist_ypr_deg` |
| `servo/j5vr_setpose.c` | 478 | traiettorie punto-punto RTR3/RTR5/BB/BCB, latch SETPOSE_DONE | `abort`, `is_active`, `tick`, `go_setpose` (vel %), `request_relax_digital_on_finish`, `go_setpose_time` (interi), `go_setpose_time_f` (float), `notify_poll` |
| `servo/servo_control.c` | 430 | PWM 6 canali (50/330 Hz), offset, config PWM | `get_period_hz`, `set_pwm_config`, `init`, `set_angle` (u16), `set_angle_f`, `get_last_pulse_us`, `get_angle`, `get_angle_f`, `disable_all`, `relax_digital`, `get_offset_deg`, `get_teleop_deg`, `offset_set` |
| `spi/hal_spi_slave.c` | 561 | SPI1 slave HAL, DMA circolare 2×64 B, thread di servizio, watchdog DMA 3 s | `init` (SYS_INIT), `is_ready`, callback HAL |
| `spi/j5_protocol.c` | 333 | parsing payload J5VR, costruzione TELEMETRY/STATUS | `latest_snapshot`, `parse_payload`, `fill_tx_telemetry`, `build_frame` |
| `spi/spi_frames.c` | 288 | finestra/scelta/dispatch dei frame (compilato anche nel simulatore) | `last_frame_age_ms`, `ingest_half`, `reset` |
| `uart/uart_control.c` | 821 | USART3 ASCII: ring buffer IRQ, parser `#seq`, 24 comandi | `send_unsolicited`, `init`, `process` |

### 1.2 Comandi UART e chi li usa

Mittenti: **J** = j5mini, **G** = assist_gimbal, **D** = dashboard, **Q** = app Quest, **P** = app telefono, **T** = solo test.

| Comando | Effetto | Chi lo manda |
|---|---|---|
| `STATUS?` | `STATUS:SAFE/IDLE/STOPPED` | J (poll 2 s), D |
| `ENABLE` | SAFE → IDLE (rifiutato da STOPPED) | Q, P, D |
| `SAFE` | → SAFE, annulla SETPOSE | Q, P, D |
| `STOP` | → STOPPED, annulla SETPOSE | Q, P, D |
| `RESET` | identico a SAFE | **solo D** → alias doppio |
| `ESTOP?` | stato fungo/bypass | solo D (diagnostica) |
| `ESTOP_BYPASS ON [min] / OFF` | ignora il fungo per N minuti (max 120) | Q |
| `HYBRID ENABLE` | **non fa nulla**, risponde OK | Q |
| `HYBRID DISABLE` | non fa nulla | **nessuno** |
| `VR?` | dump diagnostico 320 byte | solo D (diagnostica; motivo dello stack main a 2048 B) |
| `HOME` | SETPOSE agli offset, RTR5 40 %, rilascio P/R a fine corsa | Q, P, D, J |
| `PARK` / `TELEOPPOSE` | posa teleop nel firmware; **j5mini le intercetta** e manda il SETPOSE dalle pose di `j5mini.json` | Q, P (PARK anche diretto dal J5Link dello stereo-head) |
| `HEADZERO` | azzera la calibrazione testa | Q, G |
| `HORIZON 0/1 [dirs trim]` | livello automatico del roll | J |
| `RELAX_DIGITAL` | PWM = 0 su pitch/roll | solo D |
| `SETPOSE_T_HR ×6 ms planner` | traiettoria a tempo, decimi di grado | D, P |
| `SETPOSE_T ×6 ms planner` | traiettoria a tempo, gradi interi | **nessuno** (solo testo segnaposto nella dashboard) |
| `SETPOSE ×6 vel% planner` | traiettoria a velocità | J (pose PARK/VR) |
| `SET_OFFSETS`, `SET_JOINT_LIMITS`, `SET_VR_PARAMS`, `SET_PWM_CONFIG` | configurazione | J (boot + rinvio nei modi 3/4/5), D |

Righe spontanee: `BOOT_READY`, `ESTOP`, `ESTOP_CLEAR`, `ESTOP_BYPASS_EXPIRED`, `SETPOSE_DONE time_ms= vel_max= acc_max=`, `RELAX_DIGITAL`.

### 1.3 Stati, armamento, modi

- SAFE all'avvio; **SAFE → IDLE** automatico quando arrivano frame SPI freschi (≤ 500 ms) con heartbeat che avanza, oppure `ENABLE`.
- **IDLE → SAFE** per watchdog SPI (> 500 ms) o `SAFE`. **→ STOPPED** per `STOP` o fungo (debounce, disabilita tutto, annulla SETPOSE, disarma). Da STOPPED si esce solo con `SAFE`; `ENABLE` è rifiutato.
- Deadman = entrambi i grip. Armato sul fronte (grip + input, heartbeat > 0); disarmo al rilascio o fuori IDLE. Nel modo 2 il freeze scatta dopo 200 ms senza input. Ripremere i grip nei modi 3/4/5 azzera la calibrazione testa (il "clutch" del 26/10).
- Modi: 0 PREP (niente), 2 MANUAL (stick), 3 HEAD (visore → Y/P/R), 4 HYBRID (HEAD + stick su B/S/G), 5 ASSIST (B/S/G assoluti dal Pi + HEAD, con interpolazione tra frame). **Modo 1 POSE** e **modo 6 JOINT_STREAM** sono avanzi: nessun client li manda (il 6 era ROS 2).
- Orizzonte: in tutti i modi `horizon_apply` sovrascrive il roll a partire dagli angoli reali dei servi.

### 1.4 Frame SPI 64 byte

Letti dal firmware: mode, joy/pitch/yaw, intensity, heartbeat, quaternione, pulsanti, blocco `I` del modo 5 (B/S/G in centesimi di grado).
**Mai letti**: `grip`, `priority`, `safe_mask`, `mode5_target_id` e due flag del blocco `I` (parsati e ignorati).
Telemetria in risposta: 28 byte di accel/gyro/temp **sempre a zero** (il driver minimo legge solo il rotation vector), quaternione, 6 angoli servo, periodo RT. j5mini ignora i 28 byte a zero. Il Quest legge un byte `gripEcho` sempre 0 e aspetta un ack `T` che il firmware non scrive mai (già annotato in MINIMAL.md §6).

### 1.5 Tagli proposti nel firmware

Gruppo A, sicuri (nessun client li usa, verificato con grep su j5mini, dashboard, app Quest, app telefono, test):

| Cosa | Dove | Righe |
|---|---|---|
| `servo_set_angle()` versione u16 con printk `[PWM_US]` e `servo_last_sent_angle[]` (solo scritto) | `servo_control.c:212-285` e usi | ~80 |
| `imu_get_quaternion()` e la copia legacy `g_imu_quat` | `imu.c:34-39, 275-278, 329-342` | ~20 |
| `bno085_is_ready()` / `s_ready` | `bno085.c:321-324` | 5 |
| `j5vr_setpose_is_active()`, `j5vr_horizon_is_on()`, tipo `servo_setpoint_t`, `g_j5vr_last_rx_seq`, `fill_tx_telemetry_compact()` (azzera byte già azzerati), macro `J5VR_PAYLOAD_LEN`, doppia `RT_LOOP_PERIOD_MS` | vari | ~30 |
| `j5vr_apply_head_orientation_only` = copia di `j5vr_apply_head_tracking` meno due chiamate → una funzione con un flag | `j5vr_head.c:315-392` | ~75 |
| `j5vr_go_setpose_time` (interi) + comando `SETPOSE_T` (nessuno lo manda; resta `SETPOSE_T_HR`) | `j5vr_setpose.c:366-405`, `uart_control.c:546-592` | ~90 |
| `HYBRID DISABLE`, alias `RESET` (la dashboard userà `SAFE`) | `uart_control.c:344, 375` | ~10 |
| stub E-STOP per F446 (target rimosso) | `estop.c:288-331` | 44 |
| modo 6 JOINT_STREAM e logica modo 1 POSE | `rt_loop.c:230-235, 316-318, 340-358, 541-549, 634` | ~35 |
| blocco `[CTRL]` morto sotto `RT_LOOP_VERBOSE=0`, `servo_relax_digital()` commentata, `if (!deadman) {}` vuoto | `rt_loop.c:517-532, 562-564, 612` | ~20 |
| scan I2C 0x08-0x77 → una sola sonda a 0x4B (già fatta da `bno085_init`) | `imu.c:146-174` | ~25 |
| parsing di `grip`, `priority`, `safe_mask`, `mode5_target_id` | `j5_protocol.c:159-162, 178-180` | ~10 |
| `tools/fw_sim/orig/spi/*` (vecchia versione tenuta per `FW_SIM_ORIG_SRC`) | simulatore | 430 |

Gruppo B, da fare insieme all'app Quest: `HYBRID ENABLE` (il Quest lo manda ancora a `XrActivity.kt:513`): prima si toglie dall'app, poi dal firmware.

Gruppo C, configurazione Zephyr (`prj.conf`, overlay): `CONFIG_SENSOR=y` (nessuna API sensor usata), istanza PWM TIM2 "aux" senza alias nel sorgente, `zephyr,shell-uart` senza `CONFIG_SHELL`, `CONFIG_SPI=n` / `CONFIG_WATCHDOG=n` (default ridondanti), `CONFIG_ASSERT=y` (solo in una build di debug). Da **tenere**: console printk su USART1 (serve per il debug da ST-Link), LOG deferred (passare a sincrono caricherebbe il thread RT), `STACK_SENTINEL`, l'opzione 128 B (rollback).

Da **tenere anche se usati solo dalla dashboard**: `ESTOP?`, `VR?`, `RELAX_DIGITAL`: sono diagnostica che funziona.

Bug da correggere nello stesso giro: il bit di diagnostica 4 ("guard") resta a 1 dopo il primo freeze (`j5_protocol.c:227`, MINIMAL.md §13); la dashboard lo mostra come "guard" sempre acceso.

Commenti stantii da aggiornare: `uart_control.c:4` e `.h:12` dicono USART1 (è USART3); `j5vr_actuation.h:100-110` elenca i vecchi numeri dei modi; `bno085.h:30-31`; NOTES.md parla ancora di F446/USART2.

Stima: ~440 righe in `src` (6 %) + 430 nel simulatore. Ogni taglio del gruppo A/C richiede ricompilare, far girare `fw_sim` (pytest) e **riflashare**: il flash lo faccio solo con il tuo via libera.

## 2. Raspberry (j5mini e strumenti)

### 2.1 File e ruoli

| File | Righe | Cosa fa |
|---|---|---|
| `minimal/j5mini/j5mini.py` | 1.194 | il controller: `Core` (SPI 100 Hz, UART, intent, smoothing, Orizzonte), `Cameras` (UDP verso cam_stream), `Server` (wss 8557, HTTPS 443, telemetria 20 Hz, config), `Assist` (ponte verso il gimbal), backend `spidev`/`serial`/`SimRobot` |
| `minimal/j5mini/assist_gimbal.py` | 1.175 | gimbal ASSIST portato dal vecchio Pi: IK polso, aiuto braccio, posizione testa → spalla/gomito, predizione, trim IMU, puntamento, livello, campagna IMU |
| `minimal/j5mini/export_config.py` | 65 | migrazione una tantum dal vecchio albero (`controller.web_services`, non presente nel repo) |
| `minimal/j5mini/test_j5mini.py` | 851 | 27 test pytest sul simulatore |
| `raspberry/tools/cam_stream.py` | 1.093 | Picamera2 → x264 → RTSP; zoom, EV, modo, AF, snapshot, master/follower di esposizione e fuoco |
| `raspberry/tools/stereo_collimator.py` | 703 | collimazione cam0→cam1, distanza stereo (calibrazione a due punti), offset/gain di cam1 |
| `raspberry/tools/vision_service.py` | 519 | YOLO11n + distanza + OCR su richiesta (`vision_request` / `vision_result`) |
| `raspberry/tools/rtp_push.py` / `.c` + `scripts/video_direct.sh` | 520 | via video RTP diretta senza MediaMTX (solo manuale, nessun servizio) |
| `setup_pi.sh`, 6 unit systemd, rete, `start_mediamtx.sh`, `video_capture.sh` | 856 | installazione e avvio |
| `minimal/dashboard/*` | 1.735 | dashboard da banco |

Servizi vivi: `jonny5-j5mini`, `mediamtx`, `video-capture` (2× cam_stream), `sync-route`, `stereo-collimator`, `vision` (opzionale se c'è `.venv-vision`).

### 2.2 Messaggi WebSocket gestiti e chi li manda

| Tipo | Cosa fa | Mittente |
|---|---|---|
| intent (senza type) | `set_intent` + passo ASSIST nel modo 5 | Quest |
| `uart` | HEADZERO hook, PARK/TELEOPPOSE → pose, resto passa al firmware | Quest, telefono, dashboard |
| `set_vr_mode` | rinvia i parametri VR nei modi 3-5 | Quest |
| `horizon_lock` | Orizzonte on/off | Quest |
| `time_sync` | regola l'orologio del Pi | Quest |
| `get_settings` / `get_config` / `save_config` | configurazione | telefono, dashboard |
| `ws_ping` | latenza | dashboard |
| `vr_zoom_state`, `cam_exposure`, `cam_mode`, `cam_autofocus`, `cam_focus` | camere | Quest, telefono, dashboard |
| relay `stereo_collimation`, `vision_request`, `vision_result` | collimatore / visione ↔ Quest | tool del Pi |
| `imu_campaign` | registra IMU + comandi in `logs/imu_campaign` | **nessun client** (l'ho usato a mano il 26/10) |
| relay `stereo_calibrate` | calibrazione distanza | **nessun client** (usato a mano il 26/10) |
| relay `vr_zoom_command` | — | nessun mittente (il Quest lo leggerebbe) |
| relay `cameras_refocus_triggered`, `vr_session_state` | — | il Quest li manda, nessuno li legge |

Telemetria (20 Hz) che **nessuno legge**: `diag` grezzo, `horizon_trim`, `vr_zoom1`, `link.spi_shift`, `link.smoothed`; messaggi `ack`, `error` (intent non valido), `config.saveable`, `ws_pong.ts_*`, `teleop_pose_ack.id`, `vision_point.dist_after_mm`.

### 2.3 Tagli proposti sul Pi

Sicuri:

| Cosa | Dove | Righe |
|---|---|---|
| `find_frame()` (resta `find_frame_window`) | `j5mini.py:80-94` | 15 |
| `Assist.horizon_lock()` e `assist_gimbal.horizon_lock()` (mai chiamati) | `j5mini.py:999`, `assist_gimbal.py:472` | 5 |
| sonda di debug 1 Hz `pos_dbg` / `last_log` | `assist_gimbal.py:440-443, 998, 1020-1026` | 10 |
| parametri `mount_path` / `campaign_dir` di `configure()` mai passati | `assist_gimbal.py:94-107` | 5 |
| contabilità `vr_zoom[1]`, `config.saveable`, `ws_pong.ts_*`, `teleop_pose_ack.id` | `j5mini.py` | ~10 |
| `export_config.py` (migrazione finita, importa un albero che non c'è) | file | 65 |
| `--roi` / `--autofocus-range` di cam_stream (mai letti) + ramo `CAPTURE=rpicam` e sonda rpicam-vid in `video_capture.sh` | `cam_stream.py:249`, `video_capture.sh:52-98, 123-131` | ~60 |
| profili inesistenti in `start_mediamtx.sh` (solo `mediamtx_relay.yml` esiste), path `cam_g2g` nel relay | script, yml | ~45 |
| docstring falsa in `vision_service.py:9-10` ("j5mini non ha il gimbal"), commenti "vecchio Pi" | — | — |

Da decidere: `rtp_push.py/.c` + `video_direct.sh` (520 righe): la via RTP diretta non è in nessun servizio, il Quest ha il modo "udp". Proposta: spostarla in `raspberry/legacy/` così resta disponibile ma non confonde.

Da **tenere e rendere usabili dalla dashboard** (oggi raggiungibili solo a mano): `imu_campaign` (ci serve ancora per la campagna con Orizzonte), `stereo_calibrate` (calibrazione distanza), `horizon_trim`, `link.spi_shift`, `link.smoothed`, `error` (oggi un intent non valido viene segnalato a nessuno).

Costo zero, lasciare: le voci RELAY senza lettore, la formula virtuale↔fisico ripetuta in Python e JS (è una riga).

Stima: ~250 righe sicure (+520 opzionali spostate), nessun riflash: basta scp con diff e riavvio del servizio.

## 3. Dashboard: vecchia contro nuova

### 3.1 Cosa aveva la vecchia (13 pagine) e cosa c'è nella nuova (7 tab)

| Vecchia | Nuova | Note |
|---|---|---|
| Pill di stato (WS, SPI, UART, STM32, IMU, VR, modo, stato robot) | sì, più ricche (contatori, grafico 60 s) | |
| Temp IMU | no | il driver minimo non la legge; i 28 byte accel/gyro/temp sono a zero |
| **Comandi principali in home** (ENABLE=SAFE+ENABLE, STOP, HOME, PARK, VR pose) | solo nel tab Comandi | **da riportare in home** |
| IMU on/off | no | IMU sempre accesa nel minimale |
| **Demo** (sequenza di pose dal Pi) | **sì** (27/10 sera) | pulsante Demo nella fascia, passi in Impostazioni → Demo, `settings.demo` in j5mini.json |
| Calibra IMU (world bias) | no | nel minimale il visore non usa l'IMU del robot; l'IMU serve a Orizzonte e ASSIST con `imu_ee_mount.json`. Non serve più |
| IMU rpy relativa a HOME ("Display Ref") | no, solo grezza | **da riportare** (azzeramento al SETPOSE_DONE di HOME, lato client) |
| Posa del visore (rpy/quat) e heartbeat | solo modo e età dell'intent | **da riportare**: 4 float in più nella telemetria |
| Event log | sì (log UART con filtri) | da unire agli eventi di sistema |
| Giunti: slider + SETPOSE, planner, tempo/velocità | sì (solo a tempo, senza undo) | ok |
| Vista 3D (three.js 608 KB) | sì, canvas 2D con POE, tabella posa, manipolabilità | ok, più leggera |
| FK/IK, IK Live, Pick&Place, Digital Twin, Controllers, Self-test, test MJPEG/carico | no | didattica/ROS, escluse di proposito; nessuna le usa |
| Impostazioni VR (routing, HEAD ASSIST, velocità) | solo riga grezza `SET_VR_PARAMS` | da rendere leggibile (etichette sui 34 campi) |
| Zoom / fuoco / EV / sensore, video live WHEP | sì (più nitidezza, luma, snapshot) | ok |
| **Pose HOME/PARK/VR + velocità/profilo** in Impostazioni | no | **da riportare**: `save_config` oggi scrive solo offset/dir/limiti |
| Offset, limiti, PWM | sì | ok |
| Test: ping WS | sì (50 invece di 300) | ok |
| WS sullo stesso dominio (`/ws` su 443) | no: `wss://host:8557` diretto, **secondo certificato da accettare** | **da riportare**: tunnel `/ws` nel server HTTPS |
| Navbar 12 link, nessun viewport meta (su telefono è una pagina desktop rimpicciolita) | tab in una pagina, viewport ok | |

Nuove nella minimale e da tenere: checkbox di armamento dei comandi di movimento, badge E-STOP/bypass, bit deadman/armato/freeze/guard, periodo RT, grafico link 60 s, STOP con Esc/Spazio, console UART con storia e filtri, pose salvate nel browser con sequenze, misura di nitidezza per occhio, prova guidata del fungo, test escursione giunto, test link SPI, badge SIMULATO, Guida.

Funzioni del 26/10 che oggi si usano **solo dal Quest o a mano** e che la dashboard deve avere: Orizzonte on/off con trim, registrazione IMU, calibrazione distanza stereo (mm), puntamento con esito.

### 3.2 Demo

Nel vecchio Pi la demo era una lista `demo_steps` in `j5_settings.json` eseguita dal Pi come SETPOSE in sequenza. Nel minimale esiste già `pose_command()` (PARK/VR) e la dashboard ha "esegui sequenza" dalle pose salvate nel browser. Riportarla lato j5mini costa ~40 righe (`demo_steps` in `j5mini.json`, messaggio `demo start/stop`, stato in telemetria). Rimessa il 27/10 sera su richiesta (vedi README di j5mini e della dashboard).

## 4. Progetto della nuova dashboard

### 4.1 Palette: base della vecchia, accenti dell'app

| Ruolo | Valore | Da dove |
|---|---|---|
| sfondo / pannello / bordo | `#05070c` / `#111a28` / `rgba(61,157,255,.15)` | vecchia Control Room (`base.css`) |
| testo / attenuato | `#f6f8fb` / `#9db1cc` | vecchia |
| accento primario | `#1EC8A5` (teal) | icona dell'app telefono; nella vecchia era il secondario `#12c2b2` |
| accento secondario | `#3d9dff` | vecchia |
| ok / attenzione / errore / neutro | `#69F0AE` / `#FFD54F` / `#FF6E66` / `#B0BEC5` | stati dell'app telefono (`MainActivity.kt:313-317`) |
| STOP | `#D32F2F`, testo bianco bold | tasto STOP dell'app (`activity_main.xml:124-132`) |
| valori live (numeri) | mono `#00FF80` sul pannello scuro, solo per le letture | overlay statistiche dell'app |
| raggio / ombre | 10-12 px, ombra `0 4px 20px rgba(0,0,0,.4)` solo sui pannelli in rilievo | vecchia |
| hero | gradiente `rgba(34,72,141,.35) → rgba(30,200,165,.3)`, bordo sinistro 5 px teal | vecchia, con il teal nuovo |

Font: `system-ui, Segoe UI, Roboto`; mono `ui-monospace, Cascadia Code, Consolas` per log e numeri. Nessuna libreria, nessun font esterno: deve funzionare sull'hotspot senza internet.

### 4.2 Struttura

Una pagina, 6 tab: **Stato** (home), **Braccio**, **Camere**, **Teleop** (nuovo: Orizzonte, trim, modo visore, posa visore, puntamento, registrazione IMU, parametri VR con etichette), **Impostazioni**, **Test**. La Guida diventa una pagina separata (`guida.html`) per non pesare sul caricamento.

Home (Stato): hero compatto con le pill (robot, E-STOP, link Pi↔STM32, IMU, visore, camere), **riga dei comandi principali** come nella vecchia (ENABLE, SAFE, HOME, PARK, VR pose, Demo se la vuoi) con la checkbox di armamento, poi le card telemetria e il grafico. STOP sempre nell'header, in rosso, mai sotto scroll.

PC (≥ 1000 px): header fisso con pill + STOP, tab in alto, griglia a 3 colonne, log a destra in Comandi.

Telefono verticale (≤ 700 px): tab bar **in basso** con icona e etichetta corta (raggio del pollice), STOP fisso in alto a destra, card a colonna singola, bersagli di 44 px, slider dei giunti su due righe (già oggi), tastiera numerica per gli input (`inputmode`).

Telefono orizzontale nel WebView dell'app (il viewer è solo landscape, circa 850×360 px CSS): header su una riga, tab in alto compatte, 2 colonne, **angolo in basso a destra libero** per il tasto "✕ VIDEO" dell'app, `env(safe-area-inset-*)`. Con `?app=phone` la pagina nasconde brand e STOP web (l'app ha il suo STOP nativo a 52 dp) e usa lo sfondo del pannello dell'app.

### 4.3 Un solo certificato

Il server HTTPS di j5mini (443) accetta anche l'`Upgrade: websocket` su `/ws` e lo tunnella al listener WebSocket locale: la pagina parla con `wss://host/ws` sulla stessa origine. Il Quest continua su `wss://host:8557`. Risultato: PC e telefono accettano il certificato una volta sola; il WebView dell'app non deve più fidarsi di due porte.

### 4.4 Ottimizzazioni dell'app telefono (viewer)

- Sfondo del pannello dashboard = nuovo `--bg` (oggi `#0F1218`, `activity_main.xml:151`).
- URL della dashboard con `?app=phone`.
- Pannello GIUNTI (`joints_panel.xml`) e pannello TEST allineati alla stessa palette (teal per GO/HOLD attivo, stessi colori di stato).
- Nessuna altra modifica: il WebView già accetta il certificato del solo host del robot.

### 4.5 Peso e prestazioni

Resta senza librerie (oggi 75 KB; obiettivo < 100 KB con il tab Teleop). Grafico e 3D su canvas 2D. Telemetria a 20 Hz applicata al DOM con `requestAnimationFrame`, grafico ridisegnato 1 volta al secondo, log limitato a 600 righe, video WHEP fermato uscendo dal tab Camere, nitidezza calcolata su 320×180. Su telefono il peso vero è la decodifica WebRTC dei due occhi, non la pagina.

## 5. Stato (27/10/2026, sera)

- Punto 1 **fatto**: dashboard nuova + j5mini (tunnel `/ws`, posa del visore in telemetria, pose e velocità in
  `save_config`), 40 test verdi. App telefono **0.5** installata (27/10 pomeriggio): solo la dashboard in WebView,
  icona del visore, schermata d'errore nativa con ritentativo automatico; `?app=phone` nasconde solo il marchio.
  Jog per giunto (−/+/⇄, "← fisico") in Impostazioni → Giunti. Collegamenti sul desktop del PC con la stessa icona
  (hotspot e cavo). Da fare appena il robot è acceso: deploy sul Pi (diff + riavvio di `jonny5-j5mini`), prova dal
  PC e dal telefono.
- Punto 2 **fatto** per la parte Python (`find_frame`, `horizon_lock` doppio, sonda di debug del gimbal, parametri
  inutili di `configure`, `export_config.py`, docstring). **Rimandati** a un giro con il robot acceso, perché
  toccano i servizi video: `--roi`/`--autofocus-range` di cam_stream, ramo rpicam di `video_capture.sh`, profili
  di `start_mediamtx.sh`, spostamento di `rtp_push` in `legacy/`.
- Editor POE **fatto** (27/10 sera): Impostazioni → Cinematica, `get_kinematics` / `save_kinematics`, anteprima 3D, gimbal riconfigurato.
- Punto 3 **fatto** (27/10 pomeriggio, solo sul PC): gruppi A e C, bit guard corretto, commenti stantii; build Zephyr
  ok (Flash 76244 B, RAM 25922 B, `src/` 7093 righe), 23 test `fw_sim` verdi (nuovi: comandi rifiutati, modi
  sconosciuti, bit guard). **Flashato il 27/10 alle 18:20** (backup della flash precedente in `jonny5-backups/pre-cuts-20261027`)
  insieme al deploy sul Pi (`ws.local_port 8556` aggiunto). Provato sul robot: HOME, giro ASSIST con Orizzonte e puntamenti
  della visione, PARK/TELEOPPOSE, fungo (ESTOP → STOPPED, ESTOP_CLEAR senza ripartenza, SAFE+ENABLE), 45.000 frame SPI
  senza errori. Dettaglio in `firmware/stm32/MINIMAL.md`.
- Punto 4 (`HYBRID ENABLE`) **fatto** (27/10 sera): app Quest 1.4.5 non lo manda piu', firmware senza il comando (riflash).

## 5 bis. Ordine di esecuzione proposto

1. **Dashboard + j5mini** (nessun riflash): tunnel `/ws`, telemetria del visore e `horizon_trim`/`spi_shift`/`smoothed`, `save_config` con pose/velocità, tab Teleop, nuova grafica, `?app=phone`, test pytest aggiornati, deploy sul Pi con diff. Poi il viewer (palette + URL), build e installazione via adb.
2. **Tagli sul Pi** (gruppo sicuro, stesso deploy).
3. **Tagli nel firmware** (gruppi A e C, correzione del bit guard, commenti): build, `fw_sim` pytest, poi **riflash con il tuo ok** e ripetizione della prova del fungo e di un giro ASSIST.
4. `HYBRID ENABLE`: via dall'app Quest e poi dal firmware, in un giro successivo dell'app.
