# Firmware minimale STM32 (branch `fw/minimal`, 26/10/2026)

Copia **potata** del firmware che gira sul robot, non una riscrittura: i percorsi usati in produzione
sono rimasti parola per parola, è stato tolto solo ciò che è morto, sperimentale o di un'altra scheda.
**Non flashato.** Si flasha come sempre (`pio run -e weact_g474_shield -t upload` dalla junction
`C:\Users\pinco\J5MIN\firmware\stm32`) dopo la prova sul banco.

## Base

| | |
|---|---|
| Firmware sul robot oggi | `feature/video-direct-rtp` @ `6e8694e` (flash del 19/10 11:46): fix di `main` + bypass E-STOP, **senza** servo sub-grado |
| Base del minimale | merge `22b6a46` = quello + `fw/servo-subdegree` (`61d792c`, flashato il 18/10 e perso il 19/10) |
| Differenze di comportamento volute | uscita servo sub-grado (risoluzione 1 µs ≈ 0,09° invece di 1°), come il 18/10; SPI a **64 byte** invece di 128 (sotto); niente protocollo v2 né J5IK (ROS 2, sotto) |

## Cosa è stato tolto

| Tolto | Perché |
|---|---|
| ASSIST v2 RAW (`assist_v2_raw.*`, blocchi `#if` in `hal_spi_slave.c`, tipi 0x06/0x07, IDMK) | `CONFIG_ASSIST_V2_RAW_MODE=n`: mai compilato |
| boundary buffers (`boundary_buffers.*`) | scritti a ogni half DMA, mai letti |
| pick & place (`pickplace.*`, comandi `PP1/PP2/PP?`) | sul G474 non esiste l'hardware: rispondeva sempre `ERR PPn_HW`; ora `ERR UNKNOWN` |
| `motion_planner.*` | `motion_planner_stop_all()` era vuota |
| watchdog IWDG (`hw_watchdog.*`, Kconfig) | disattivato in tutti i branch |
| target F446 (env PlatformIO, `zephyr/overlays/`, fallback in `j5_board.h`) | il robot ha lo shield G474 |
| `j5vr_to_servo_setpoint()` + costanti CENTER/RANGE | "validazione" che restituiva sempre `true`, risultato mai usato |
| conversioni quaternione mai chiamate, getter HYBRID mai chiamato, wake-up tap SETPOSE (disattivato), helper interi | codice morto |
| blocchi di debug a 0 (`J5VR_DEBUG_VERBOSE`, `J5_PROTO_RX_DEBUG`, `UART_VR_PARAMS_DEBUG`), diagnostica fp32 | compilati fuori |
| commenti sbagliati (TIM8/TIM1, I2C1, USART1/2, numeri dei modi, testo corrotto in `rt_loop.c`) | corretti con l'hardware reale: TIM4/TIM17/TIM3/TIM5, I2C2, USART3 controllo / USART1 console |

| protocollo v2 con CRC (`TELEMETRY_V2` 0x08, contatori gap/CRC) e frame J5IK (0x05, modo 6 JOINT_STREAM, `pose_known`) | li usava solo lo stack ROS 2, tolto il 26/10 (j5mini parla v1). Un frame v2 o J5IK ora è un frame non valido: risposta STATUS 0xEE e nessun "rinfresco" del watchdog SPI. Dal 27/10 un J5VR con modo 6 e' un modo sconosciuto come gli altri: nessuna attuazione, riarmo come sempre |

| comandi `IMUON` / `IMUOFF` | la IMU è sempre accesa (era già il default all'avvio); j5mini non manda più `IMUON` e lo salta nelle configurazioni vecchie |

Tagli del 27/10/2026 (gruppi A e C di `minimal/INVENTARIO_E_TAGLI.md`, nessun client li usava; verificati con grep su
j5mini, dashboard, app Quest, app telefono, test), **non ancora flashati**:

| Tolto | Note |
|---|---|
| `SETPOSE_T` a gradi interi (`j5vr_go_setpose_time`) | resta `SETPOSE_T_HR` in decimi di grado; `SETPOSE_T ...` ora risponde `ERR UNKNOWN` |
| `HYBRID DISABLE`, alias `RESET` di `SAFE` | `HYBRID ENABLE` resta (lo manda l'app Quest prima del modo 4) |
| modo 1 POSE e modo 6 JOINT_STREAM nel RT loop | un modo sconosciuto non attua nulla (prima il fallback era MANUAL) e non impedisce il riarmo |
| `servo_set_angle()` a interi con il log `[PWM_US]`, `servo_last_sent_angle[]` | resta `servo_set_angle_f()` |
| `imu_get_quaternion()` e la copia `g_imu_quat`, `bno085_is_ready()` | `imu_get_snapshot()` e' l'unica lettura |
| scan I2C 0x08-0x77 all'avvio | una sonda sola a 0x4B (`imu_probe_bno085`) |
| `j5vr_apply_head_orientation_only` (copia di `j5vr_apply_head_tracking`) | una funzione, `j5vr_apply_head(s, apply_servos)` |
| stub E-STOP per board senza alias DT (F446) | senza alias la build si ferma con `#error` |
| campi `grip`, `safe_mask`, `target_id` del J5VR (parsati, mai letti); `priority` resta (lo legge HYBRID) | il layout del frame non cambia |
| `j5vr_setpose_is_active`, `j5vr_horizon_is_on`, `servo_setpoint_t`, `g_j5vr_last_rx_seq`, `fill_tx_telemetry_compact`, `J5VR_PAYLOAD_LEN`, doppia `RT_LOOP_PERIOD_MS`, blocco `[CTRL]` e commenti morti in `rt_loop.c` | codice morto |
| `CONFIG_SENSOR`, `CONFIG_ASSERT`, `CONFIG_SPI=n` / `CONFIG_WATCHDOG=n` ridondanti, `zephyr,shell-uart`, istanza PWM TIM2 "aux" | configurazione Zephyr |
| `tools/fw_sim/orig/` e il test differenziale | il minimale diverge dall'originale apposta da qui in poi |

Corretto nello stesso giro: il bit 4 di diag (`guard`) segue il freeze invece di restare a 1 (stranezza 13 qui sotto).
Build Zephyr: Flash 76244 B, RAM 25922 B; `src/` 7093 righe; `python -m pytest tools/fw_sim -q` 23 test.

Rimasti, anche se poco usati: IMU BNO085 (solo telemetria, sempre accesa), `HYBRID ENABLE` (risponde OK, lo manda l'app Quest), `VR?`,
pulsante centrale PC13, log `J5VR_DEBUG_MINIMAL` (attivo).

Spostato senza modifiche: la logica dei frame SPI (finestra, scelta del frame, risposte, contatori) da
`hal_spi_slave.c` a `spi_frames.c`, perché giri identica anche nel simulatore. In `hal_spi_slave.c`
restano HAL, DMA e thread.

| | Originale | Minimale |
|---|---|---|
| righe in `src/` | 9.675 (41 file) | 7.486 (33 file) |
| flash / RAM | 96.908 / 28.292 B | 92.768 / 27.470 B (a 64 byte, senza v2/J5IK e IMUON/IMUOFF) |

## Come è stata verificata l'equivalenza (senza hardware)

1. Ogni passo di potatura ricompilato: dove il codice tolto era già fuori compilazione il binario è
   rimasto **identico al byte** (96.908 B dopo `#if`, 95.200 B dopo i blocchi di debug).
2. **Simulatore** `tools/fw_sim/`: i sorgenti veri del firmware (state machine, E-STOP, passo del RT
   loop, protocollo e frame SPI, UART, attuazione, HEAD, SETPOSE, `servo_control.c` con PWM a 1 MHz)
   compilati sul PC con stub di Zephyr; SPI emulato byte per byte come il DMA circolare.
3. **Test differenziale** (fino al 27/10): lo stesso scenario (configurazione come il Pi, armo automatico, HOME,
   manuale su tutti gli assi, HEAD/HYBRID/ASSIST, payload neutro, richieste STATUS, tutte le
   SETPOSE, PARK/TELEOPPOSE, RELAX, timeout SPI, E-STOP e bypass; circa 2.000 trasferimenti SPI,
   48 comandi UART) girava sul minimale e sull'originale estratto da `22b6a46`, a 64 e a 128 byte:
   **risposte UART, frame SPI e impulsi PWM identici**. Con i tagli del 27/10 (tabella sopra) il
   minimale diverge apposta e il confronto e' stato tolto; lo scenario resta come prova di corsa e i
   test di comportamento coprono i tagli (comandi rifiutati, modi sconosciuti, bit guard).
   `python -m pytest tools/fw_sim -q` (23 test).

Non coperti dal simulatore (solo hardware): inizializzazione HAL/DMA/IRQ di SPI1, driver BNO085,
timer TIM6, PWM reali. Questi file non sono stati toccati se non nei commenti.

## 128 o 64 byte?

**Ne bastano 64.** I byte 64–127 sono sempre zero in trasmissione e ignorati in ricezione, sia sul
firmware sia sul Pi: sono un residuo dell'esperimento ASSIST v2 (frame da 128 con dati oltre il 64),
abbandonato. L'unico vincolo è che **master e slave usino la stessa lunghezza**: il DMA circolare dello
STM32 lavora a metà da `SPI_FRAME_LEN` e non si risincronizza sul fronte di CS.

| Firmware | Master | Risultato (simulatore) |
|---|---|---|
| 128 (flashato il 19/10) | 128 (Pi `J5_SPI_CANONICAL_128=1`, sender `--spi-len 128`) | ok, una risposta per trasferimento |
| **64 (`fw/minimal`, `CONFIG_J5_CANONICAL_PADDED_128_MODE=n`)** | **64** | ok |
| 128 | 64 | i comandi passano, ma risposta valida solo un trasferimento su due |

**Dal 26/10 `fw/minimal` è a 64 su tutti i lati**: `prj.conf` con `=n`; `j5mini` (unico master
SPI sul Pi in questo branch) e il sender della testa stereo a 64 di default. Il test differenziale gira a entrambe le lunghezze: alla
stessa lunghezza minimale e originale sono identici.

**Frizione dei grip (26/10, dal robot).** In HEAD/HYBRID/ASSIST il riferimento della testa si riprende a ogni
pressione dei grip (fronte del deadman): testa e posa di quel momento. Prima restava quello dell'ingresso nel modo,
così girando la testa a grip rilasciati il braccio saltava alla ripresa. Test `test_grips_pressed_again_*`.

**Orizzonte (26/10).** `HORIZON <0|1> [dB dS dG dY dP dR trim_deg]`: in ogni modo VR `j5vr_actuation_apply_desired()` porta
il ROLL al valore che mette a livello l'asse orizzontale della camera per la posa reale dei servo (cinematica
Rz Ry Ry Rz Ry Rx, versi del Pi), entra/esce in 0,5 s; il trim viene dalla IMU (lato Pi). Con l'Orizzonte il MANUAL
non azzera piu' il roll a ogni tick. Test `test_horizon_*`.

**Interpolazione fra frame SPI (26/10, scatti in ASSIST).** In HEAD/HYBRID/ASSIST il quaternione del visore e i
target B/S/G scorrono linearmente da un frame al successivo nell'intervallo misurato fra i frame (5-30 ms),
`j5vr_smooth_between_frames()` sullo snapshot del RT loop; grip, pulsanti e stick restano quelli del frame. Cambio di
modo o frame fermi da 100 ms: salto diretto. Test `test_targets_are_interpolated_between_spi_frames`.

**Sfasamento del DMA (trovato sul robot il 26/10).** Il DMA circolare dello slave non si riallinea sul CS:
se si arma a metà di un trasferimento, ogni risposta esce in ritardo di qualche byte e si spezza su due
trasferimenti (sul robot: 3 byte, poi 0 dopo un reset dello STM32). La ricezione dello STM32 regge (finestra
a cavallo dei due half), ma il master deve cercare il frame **negli ultimi due trasferimenti**: a 128 byte il
vecchio Pi ci riusciva per caso (il frame sfasato stava ancora nei 128), a 64 no. `j5mini` e il sender della
testa stereo lo fanno (`spi_shift` / `shift` nelle statistiche); il simulatore lo riproduce con `sim_spi_phase()`.

**Da fare insieme**: chi flasha `fw/minimal` deve installare anche lo stack minimale sul Pi
(`raspberry/setup_pi.sh`, README del repo). Con il firmware del 19/10 il Pi resta a
128. L'opzione `=y` resta nel codice per tornare indietro senza modifiche.

## Comportamenti strani mantenuti apposta (da decidere, non cambiati)

Il minimale si comporta **come l'originale**, quindi anche questi restano. Ognuno è visibile nel simulatore.

1. **Limiti di velocità di PITCH/ROLL ignorati** fuori dal modo 6: con il filtro 0,35 il polso segue la
   media esponenziale del target e `vel_pitch/vel_roll` non contano.
2. **In HEAD/HYBRID la spalla/gomito/base restano senza PWM** quando non c'è input sugli stick (latch
   "servo off"): tengono solo grazie alla tenuta degli LDX-218.
3. **Deadman rilasciato = tutte le uscite PWM a 0** (servo liberi), non "tieni posizione".
4. **SAFE non è stabile**: con un flusso J5VR e heartbeat che avanza il firmware torna IDLE da solo
   in ~1 ms. Lo stop affidabile è `STOP` (STOPPED) o l'E-STOP.
5. Il heartbeat è controllato solo come `> 0`; la "freschezza" è garantita solo dal timeout SPI di 500 ms.
6. **Nessun ack di TELEOPPOSE** nei frame STATUS (`'T',1`): il Pi e l'app lo aspettano e proseguono
   dopo 5 s.
7. `PARK` e `TELEOPPOSE` vanno alla stessa posa.
8. `SET_JOINT_LIMITS` con min > max risponde OK ma non applica nulla.
9. `SET_VR_PARAMS` senza la parte intera cambia l'instradamento del polso (default diversi dal boot).
10. La prima SETPOSE dopo il boot parte dalle `desired_positions` (offset), non dalla posa reale:
    possibile scatto.
11. Pulsante PC13: la doppia inversione fa scattare HOME probabilmente al **rilascio** (cablaggio ignoto).
12. Il DMA SPI si riavvia ogni 3 s quando il master tace (riga di log ogni volta).
13. ~~Il bit 4 di diag (`guard`) resta a 1 per sempre dopo il primo freeze.~~ Corretto il 27/10: segue il freeze
    (test `test_guard_bit_follows_the_freeze`).

## Strumenti

- `tools/fw_sim/` — simulatore e test (`README.md` lì).
- `../../minimal/j5mini/` — controller minimale per Pi/Aura che parla con questo firmware.
