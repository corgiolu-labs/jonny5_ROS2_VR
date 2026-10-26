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
| Unica differenza di comportamento voluta | uscita servo sub-grado (risoluzione 1 µs ≈ 0,09° invece di 1°), come il 18/10 |

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

Rimasti, anche se poco usati: protocollo v2 con CRC (lo stack ROS 2 lo usa), J5IK / modo 6,
IMU BNO085 (solo telemetria), `HYBRID ENABLE/DISABLE` (rispondono OK, il Pi li manda), `VR?`,
pulsante centrale PC13, log `J5VR_DEBUG_MINIMAL` (attivo).

Spostato senza modifiche: la logica dei frame SPI (finestra, scelta del frame, risposte, contatori) da
`hal_spi_slave.c` a `spi_frames.c`, perché giri identica anche nel simulatore. In `hal_spi_slave.c`
restano HAL, DMA e thread.

| | Originale | Minimale |
|---|---|---|
| righe in `src/` | 9.675 (41 file) | 7.995 (33 file) |
| flash / RAM | 96.908 / 28.292 B | 95.152 / 27.963 B |

## Come è stata verificata l'equivalenza (senza hardware)

1. Ogni passo di potatura ricompilato: dove il codice tolto era già fuori compilazione il binario è
   rimasto **identico al byte** (96.908 B dopo `#if`, 95.200 B dopo i blocchi di debug).
2. **Simulatore** `tools/fw_sim/`: i sorgenti veri del firmware (state machine, E-STOP, passo del RT
   loop, protocollo e frame SPI, UART, attuazione, HEAD, SETPOSE, `servo_control.c` con PWM a 1 MHz)
   compilati sul PC con stub di Zephyr; SPI emulato byte per byte come il DMA circolare.
3. **Test differenziale**: lo stesso scenario (configurazione come il Pi, armo automatico, HOME,
   manuale su tutti gli assi, HEAD/HYBRID/ASSIST, payload neutro, v1/v2, richieste STATUS, tutte le
   SETPOSE, PARK/TELEOPPOSE, RELAX, timeout SPI, E-STOP e bypass, J5IK; 2.039 trasferimenti SPI,
   48 comandi UART) gira sul minimale e sull'originale estratto da `22b6a46`: **risposte UART, frame
   SPI e impulsi PWM identici**. `python -m pytest tools/fw_sim -q` (13 test).

Non coperti dal simulatore (solo hardware): inizializzazione HAL/DMA/IRQ di SPI1, driver BNO085,
timer TIM6, PWM reali. Questi file non sono stati toccati se non nei commenti.

## 128 o 64 byte?

**Ne bastano 64.** I byte 64–127 sono sempre zero in trasmissione e ignorati in ricezione, sia sul
firmware sia sul Pi: sono un residuo dell'esperimento ASSIST v2 (frame da 128 con dati oltre il 64),
abbandonato. L'unico vincolo è che **master e slave usino la stessa lunghezza**: il DMA circolare dello
STM32 lavora a metà da `SPI_FRAME_LEN` e non si risincronizza sul fronte di CS.

| Firmware | Master | Risultato (simulatore) |
|---|---|---|
| 128 (oggi) | 128 (Pi `J5_SPI_CANONICAL_128=1`, sender default) | ok, una risposta per trasferimento |
| 64 (`CONFIG_J5_CANONICAL_PADDED_128_MODE=n`) | 64 | ok |
| 128 | 64 | i comandi passano, ma risposta valida solo un trasferimento su due |

Passare a 64 è una scelta di pulizia, non una necessità: va fatta insieme sui due lati (firmware `=n`,
Pi senza `J5_SPI_CANONICAL_128`, sender `--spi-len 64`). Finché il Pi resta a 128 conviene lasciare 128.

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
    possibile scatto. `pose_known` significa solo "una SETPOSE completata".
11. Pulsante PC13: la doppia inversione fa scattare HOME probabilmente al **rilascio** (cablaggio ignoto).
12. Il DMA SPI si riavvia ogni 3 s quando il master tace (riga di log ogni volta).
13. Il bit 4 di diag (`guard`) resta a 1 per sempre dopo il primo freeze.

## Strumenti

- `tools/fw_sim/` — simulatore e test (`README.md` lì).
- `../../minimal/j5mini/` — controller minimale per Pi/Aura che parla con questo firmware.
