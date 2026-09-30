# Piano di test fisico — JONNY5 (PR #2)

Procedura per validare sul robot vero le modifiche di questa PR, in ordine di rischio crescente.
Ogni fase ha un **criterio di superamento**. Se una fase fallisce, **fermati lì**: le fasi successive si basano su di essa.

## 0. Preparazione e sicurezza

- Braccio libero da ostacoli nel suo raggio d'azione, alimentazione servo separata e raggiungibile.
- **Fungo E-STOP a portata di mano** in tutte le fasi con movimento (fasi 2, 5, 6, 7, 8).
- Il Pi deve avere il branch `claude/festive-sagan-59u3bl` aggiornato.
- Sul Pi un solo processo può usare lo SPI (`/dev/spidev0.0`) e un solo processo la UART (`/dev/serial0`). Comandi utili:

```bash
sudo systemctl stop jonny5-spi-j5vr jonny5-ws-teleop       # ferma lo stack legacy
sudo systemctl start jonny5-spi-j5vr jonny5-ws-teleop      # lo riavvia
docker compose -f ros2_ws/deploy/docker-compose.yml down   # ferma lo stack ROS 2 in Docker
```

- Prima di toccare il robot, fai girare i test in simulazione sul Pi. Devono dare tutti `ALL PASS`:

```bash
cd ros2_ws && colcon build && source install/setup.bash
python3 tools/sim_e2e.py
```

Da registrare in ogni fase: l'output dei comandi, e un video breve per le fasi con movimento.

## 1. Compilazione e flash del firmware

```bash
cd firmware/stm32
pio run -e weact_g474_shield          # SHIELD rev3 (G474)
pio run -e weact_g474_shield -t upload
# in alternativa, senza PlatformIO: tools/build_zephyr.sh nucleo_g474re
```

- Apri la console seriale USART1 a 115200 baud. Attesi `[BOOT] JONNY5-4.0`, poi `[BOOT] RT loop 1kHz started`.
- **Superata se:** la compilazione termina senza errori e il boot completa.

Riferimento: su Zephyr 4.2.1 la build di questa PR usa FLASH 98.5 KB e RAM 28.2 KB sul G474, circa 190 B di RAM in più rispetto a `main`.

## 2. Regressione con lo stack legacy (protocollo v1)

Avvia lo stack di sempre: `jonny5-spi-j5vr` e `jonny5-ws-teleop` attivi, visore collegato.

| # | Prova | Atteso |
|---|---|---|
| 2.1 | Teleoperazione VR normale: modi MANUAL, HEAD, ASSIST (5) | Si comporta come prima della PR |
| 2.2 | In mode 5 muovi base, spalla e gomito | Il braccio segue il target, non va a 0° (bug corretto) |
| 2.3 | Dashboard: HOME e PARK | Movimento regolare, poi `SETPOSE_DONE` ricevuto |
| 2.4 | Durante HOME: `sudo systemctl stop jonny5-spi-j5vr` | Entro 0,5 s: SAFE, servo spenti, **HOME si interrompe** |
| 2.5 | Mentre tieni i grip premuti, chiudi la pagina VR sul visore: l'ultimo intent resta congelato. Poi `sudo systemctl stop jonny5-spi-j5vr`, attendi 1 s e dai `start` | Resta SAFE: **nessuna riattivazione** con l'heartbeat congelato. Riaprendo il visore, l'heartbeat riparte e il braccio si riattiva |
| 2.6 | Premi il fungo E-STOP durante un movimento | Stop immediato, stato STOPPED; al rilascio nessuna ripartenza automatica |
| 2.7 | Mentre muovi con VR, invia più volte `VR?` / `STATUS?` dalla dashboard | Nessuno scatto o esitazione del braccio (la UART non blocca più il loop a 1 kHz) |

**Superata se:** tutte le righe si comportano come atteso.

## 3. Collegamento SPI v2 (senza movimento)

Ferma lo stack legacy (vedi fase 0), poi:

```bash
python3 raspberry/tools/spi_v2_probe.py --protocol 1 --seconds 10     # regressione v1
python3 raspberry/tools/spi_v2_probe.py --seconds 30                  # v2 a 100 Hz
python3 raspberry/tools/spi_v2_probe.py --seconds 30 --rate 200       # v2 a 200 Hz
python3 raspberry/tools/j5_uart.py STATUS?
```

**Superata se:**
- v1 dà `PASS`;
- v2 dà `RESULT: PASS`: errori CRC ≤ 0,1%, sequenze perse ≤ 0,1%, telemetria su più del 90% dei trasferimenti.

Da annotare: `repeated`, cioè le risposte ripetute dello slave, e il periodo medio del loop RT, che deve stare intorno a 1000 µs.
Se v2 dà `NO TELEMETRY_V2`, prova `--transfer-len 64` e verifica che il firmware flashato sia quello nuovo.

## 4. Driver ROS 2, segni dei giunti (senza streaming)

Questa fase è **obbligatoria prima di qualsiasi comando articolare da ROS**. Se un verso è sbagliato, un comando ROS muove quel giunto al contrario.

```bash
ros2 launch jonny5_bringup bringup.launch.py use_mock_spi:=false protocol_version:=2
rviz2 -d $(ros2 pkg prefix jonny5_description)/share/jonny5_description/rviz/jonny5.rviz
```

1. Con il braccio in HOME, `/joint_states` deve dare circa 0 rad su tutti i giunti. Se un giunto è lontano da 0, correggi il suo offset in `servo_offsets_deg`.
2. Muovi un giunto alla volta con la UART e confronta con RViz. Gli angoli di SETPOSE_T sono **fisici** (gradi servo) e vanno scritti nell'ordine B S G Y P R.
   Parti dagli offset HOME attuali (100 88 93 95 90 95) e cambia un solo valore di +10°. Esempio per la base:
   ```bash
   python3 raspberry/tools/j5_uart.py ENABLE "SETPOSE_T 110 88 93 95 90 95 2000 RTR5"
   python3 raspberry/tools/j5_uart.py "SETPOSE_T 100 88 93 95 90 95 2000 RTR5"   # ritorno
   ```
   Il valore riportato in RViz deve essere circa `dir × 10°` (0,17 rad). Compila la tabella:

| Giunto | Verso reale = RViz? | Azione se no |
|---|---|---|
| base | | invertire `servo_dirs[0]` |
| spalla | | invertire `servo_dirs[1]` |
| gomito | | invertire `servo_dirs[2]` |
| polso yaw | | invertire `servo_dirs[3]` |
| polso pitch | | invertire `servo_dirs[4]` |
| polso roll | | invertire `servo_dirs[5]` |

Offset e versi vanno corretti **in entrambi** i file:
- `ros2_ws/src/jonny5_bringup/config/jonny5.params.yaml`, parametri `servo_offsets_deg` / `servo_dirs`;
- `ros2_ws/src/jonny5_description/urdf/jonny5.ros2_control.xacro`, parametri omonimi.

3. Controlla `/jonny5/status`. Deve riportare `state` IDLE o SAFE coerente con `j5_uart.py STATUS?`. Premendo il fungo deve comparire `estop_active: true`.

**Superata se:**
- in HOME tutti i giunti sono entro ±2°;
- la tabella è tutta "sì" (dopo le eventuali correzioni).

## 5. Streaming articolare J5IK, primo movimento comandato da ROS

Con lo stesso launch della fase 4, E-STOP in mano:

```bash
python3 raspberry/tools/j5_uart.py ENABLE HOME          # STM32 in IDLE, braccio in HOME (giunti circa 0 rad)
ros2 topic echo --once /joint_states                    # leggi la posa attuale
# pubblica ESATTAMENTE la posa attuale (qui HOME = tutti 0):
ros2 topic pub -r 50 /jonny5/joint_commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0, 0, 0, 0, 0]}"
ros2 service call /jonny5/joint_stream/enable std_srvs/srv/SetBool "{data: true}"
# poi cambia un solo valore alla volta, a passi piccoli: 0.05, 0.1, 0.2 rad
```

L'abilitazione viene **rifiutata**, con il motivo nella risposta del servizio, in tre casi:
- il comando dista più di 0,05 rad dalla posa attuale;
- il comando è più vecchio di 0,5 s;
- l'STM32 non è in IDLE, oppure l'E-STOP è attivo.

| # | Prova | Atteso |
|---|---|---|
| 5.1 | Passi di 0,05 rad su un giunto alla volta | Il giunto si muove nel verso giusto, alla velocità limitata dal firmware |
| 5.2 | Ferma il `ros2 topic pub` | Il braccio **resta fermo** sull'ultimo comando (il driver continua a inviarlo) |
| 5.3 | `Ctrl+C` sul launch del driver | Entro 0,1 s il braccio si ferma; dopo 0,5 s SAFE, servo spenti |
| 5.4 | Chiama `joint_stream/enable` con `false` | Il braccio torna al percorso VR/IDLE e i servo si spengono |
| 5.5 | Premi il fungo durante lo streaming | Stop immediato; `estop_active: true` in `/jonny5/spi/telemetry`; il log dice che lo streaming è disabilitato |
| 5.6 | Rilascia il fungo, poi `j5_uart.py SAFE ENABLE` | Il braccio **non si muove da solo**: bisogna richiamare `joint_stream/enable` |
| 5.7 | Durante lo streaming: `j5_uart.py SAFE` | Stop; nessun riarmo automatico anche se lo streaming continua |
| 5.8 | Pubblica un comando lontano (0,5 rad) e chiama `enable` | Abilitazione rifiutata: "command is ... rad from the current pose" |

**Superata se:** tutte le righe si comportano come atteso, con `crc_errors_*` e `rx_seq_gaps` fermi a 0 o quasi.

## 6. ros2_control

Ferma il launch della fase 4/5.

```bash
ros2 launch jonny5_bringup control.launch.py mock_hardware:=false
ros2 control list_controllers        # tre controller attivi
ros2 control list_hardware_interfaces
python3 raspberry/tools/j5_uart.py ENABLE   # l'arm si arma SOLO con questo comando esplicito
```

Nel log deve comparire `STM32 IDLE and command at the current pose: joint streaming armed`.

Poi invia un goal piccolo e lento (4 s):

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
 "{trajectory: {joint_names: [base_joint, shoulder_joint, elbow_joint, wrist_yaw_joint, wrist_pitch_joint, wrist_roll_joint],
   points: [{positions: [0.1, 0.1, 0.2, 0, -0.1, 0], time_from_start: {sec: 4}}]}}"
```

- All'avvio il braccio **non deve scattare**, perché il comando parte dalla posa attuale.
- Il goal deve finire con `SUCCEEDED`.
- Premi il fungo, oppure `j5_uart.py SAFE`: il log deve mostrare `STM32 left IDLE ... joint streaming paused` e il braccio resta fermo.
- Dopo il rilascio e `j5_uart.py SAFE ENABLE`: lo streaming si riarma **senza scatti** e il log mostra `armed`.
- Stacca il connettore SPI (a braccio fermo): entro 0,5 s il log deve mostrare `SPI link lost` e l'hardware va in errore.
  - Per ripartire: `ros2 control set_hardware_component_state JONNY5 active`, poi riattiva i controller con `ros2 control switch_controllers --activate joint_trajectory_controller`.

**Superata se:** il goal va a buon fine, SAFE/E-STOP mettono in pausa senza riarmo automatico, e il link perso manda l'hardware in errore.

## 7. MoveIt 2

```bash
ros2 launch jonny5_moveit_config move_group.launch.py mock_hardware:=false rviz:=true
python3 raspberry/tools/j5_uart.py ENABLE   # arma lo streaming (sempre esplicito)
```

- Da RViz (pannello MotionPlanning), gruppo `arm`: pianifica verso `ready` con velocità 0,1 e poi esegui.
- Poi pianifica e torna verso `home`.

**Superata se:** i movimenti sono fluidi e senza collisioni, e la posa `ready` è sensata sul robot vero. Se non lo è, annota i valori da mettere nell'SRDF.

## 8. Teleoperazione VR con MoveIt Servo

```bash
sudo systemctl stop jonny5-ws-teleop      # la pagina VR usa il proxy /ws -> porta 8557
ros2 launch jonny5_moveit_config servo.launch.py mock_hardware:=false vr_port:=8557
python3 raspberry/tools/j5_uart.py ENABLE   # arma lo streaming (sempre esplicito)
```

1. Porta il braccio in `ready` (fase 7, oppure un goal alla JTC).
2. Apri sul visore la solita pagina VR: il suo `/ws` arriva al bridge ROS 2 sulla 8557.
3. Con entrambi i grip premuti, stick a metà corsa (modalità `joint`, default):

| Input | Movimento atteso |
|---|---|
| stick sinistro a destra / sinistra | base gira a destra / sinistra |
| stick sinistro avanti / indietro | spalla in avanti / indietro |
| stick destro su / giù | avambraccio su (l'utensile sale) / giù |
| stick destro a destra / sinistra | polso yaw gira a destra / sinistra |

La modalità cartesiana (`teleop_mode:=twist`) è sperimentale: su questo braccio è mal
condizionata quasi ovunque (vedi MOVEIT.md).

4. Rilascia un grip: il braccio si ferma entro 0,15 s.
5. Chiudi il visore o spegni il Wi-Fi: il braccio si ferma.

Da annotare:
- se Servo si ferma troppo presto o troppo tardi vicino alle singolarità, vanno ritarate le soglie in `jonny5_moveit_config/config/servo.yaml` (ora 150/400);
- se le velocità sono troppo alte o troppo basse, il parametro è `max_joint_vel` del nodo `jonny5_intent_to_servo` (`max_linear` in modalità `twist`).

Esito del 30/09/2026, senza visore (messaggi del visore simulati da script sul bridge 8557):
i quattro versi sono corretti, il rilascio di un grip ferma il braccio (1–3° di coda con la
telemetria a gradi interi), il flusso interrotto lo ferma in circa 0,35 s. Resta da fare
con il visore vero: sensazione, velocità, intuitività.

**Superata se:** i movimenti sono intuitivi e il deadman funziona sempre.

## 9. (Opzionale) Watchdog hardware IWDG

Attivalo solo dopo che le fasi 1–8 sono andate bene.

1. In `firmware/stm32/zephyr/prj.conf` imposta `CONFIG_WATCHDOG=y` e `CONFIG_J5_HW_WATCHDOG=y`. Il timeout è 250 ms e si cambia con `CONFIG_J5_HW_WATCHDOG_TIMEOUT_MS`.
2. Ricompila e fai il flash. Al boot la console deve mostrare `[WDT] IWDG armed, timeout 250 ms`.
3. Lascia il robot acceso 15 minuti in uso normale: VR e UART `VR?` ripetuti. Non devono comparire reboot, cioè `[BOOT]` ripetuti in console.

**Superata se:** l'IWDG si arma e non causa reset spuri.
Se compaiono reset, riporta `CONFIG_WATCHDOG=n` e segnalalo con il log della console.

## Criteri di interruzione immediata

Premi l'E-STOP e fermati se succede uno di questi casi:
- movimento non comandato, oppure nel verso opposto a quello atteso;
- scatto all'attivazione di un controller;
- `crc_errors` o `rx_seq_gaps` in crescita continua;
- il braccio non si ferma al rilascio del deadman o allo stop del driver.

## Da riportare dopo il test

- Esito di ogni fase, cioè le tabelle compilate.
- Output di `spi_v2_probe.py` per v1, v2 a 100 Hz e v2 a 200 Hz.
- Eventuali correzioni a offset e versi, e la posa `ready` reale.
- I log dei launch che hanno dato errore (percorso nel terminale, oppure `~/.ros/log`).
