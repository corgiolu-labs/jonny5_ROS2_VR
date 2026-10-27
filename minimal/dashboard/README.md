# Dashboard JONNY5 (control room)

Pannello web per stato, comandi, teleoperazione, impostazioni e test del braccio, per PC e telefono (anche dentro
l'app del telefono), pensato anche come banco di prova per chi studia la robotica su un 6R. Una pagina statica (HTML + JavaScript, nessuna libreria esterna: il robot non
ha internet) servita da `j5mini`, che parla con lo STM32. Gli stessi messaggi WebSocket dell'app del visore.

## Aprirla

| Dove | Indirizzo |
|---|---|
| Robot, dall'hotspot JONNY5 | `https://10.42.0.1/` |
| Robot, dal PC sul cavo | `https://192.168.137.221/` |
| Senza robot (simulatore) | `python3 minimal/j5mini/j5mini.py --sim` poi `http://localhost:8558/` |

Sul robot il certificato è autofirmato: la prima volta il browser chiede di accettarlo, una volta sola. La pagina
parla con j5mini sulla stessa origine (`wss://<robot>/ws`, tunnel verso il WebSocket locale); se il tunnel non
risponde prova `wss://<robot>:8557` (in quel caso va accettato anche lì). `?ws=wss://host:porta` forza un altro
indirizzo; `?app=phone` nasconde solo la riga del marchio (l'app del telefono è questa pagina e basta, STOP compreso).

Stile: base scura della vecchia Control Room con gli accenti dell'app del telefono (teal, verde/ambra/rosso degli
stati, numeri live in verde mono). Su telefono verticale le schede stanno in basso; in orizzontale (WebView
dell'app) tutto è compatto e l'angolo in basso a destra resta libero per il tasto "✕ VIDEO" dell'app.

Il simulatore (`--sim`) compila il **firmware vero** per il PC (`firmware/stm32/tools/fw_sim`, serve `gcc`) e lo fa
girare in tempo reale al posto di SPI e UART: stati, limiti, traiettorie, E-STOP e risposte sono quelli del robot.
Le impostazioni salvate in simulazione restano in memoria (non riscrive `j5mini.example.json`).

## Schede

| Scheda | Cosa |
|---|---|
| Stato | fascia con i comandi principali (ENABLE = SAFE+ENABLE, SAFE, HOME, PARK, VR pose, Orizzonte), stato del firmware e bit di diagnostica (E-STOP, deadman, armato, freeze, guard), qualità del link SPI/UART (anche fase DMA e frame interpolati), visore (modo, heartbeat, posa della testa), IMU grezza o riferita a HOME, telecamere e collimatore, angoli dei giunti, grafico degli ultimi 60 s |
| Camere | le due camere dal vivo (WebRTC da MediaMTX, segnalazione attraverso j5mini sulla porta HTTPS), esito per occhio (≥ 20 fps, < 1 % perdite), risoluzione, bitrate, jitter, decodifica, nitidezza e luminosità a confronto, istantanee; autofocus, tele/grandangolo, zoom, fuoco, luminosità; calibrazione della distanza stereo (`stereo_calibrate`) |
| Teleop | Orizzonte con il trim dall'IMU, posa del visore e HEADZERO, visione e puntamento (`vision_request`), registrazione IMU (`imu_campaign`), i 34 campi di `SET_VR_PARAMS` con le etichette (Applica / Salva) |
| Braccio | 6 slider in gradi virtuali dentro i limiti, "Vai" con tempo e profilo (`SETPOSE_T_HR`), modalità "segui gli slider", pose salvate nel browser (sequenze, esporta/importa), vista 3D misurato/target, posa dell'end-effector (mm, RPY) e manipolabilità |
| Comandi | pulsanti dei comandi del firmware, console UART con cronologia, registro delle risposte e dei messaggi spontanei |
| Impostazioni | offset (HOME), verso, limiti fisici, pose PARK / VR pose con velocità e profilo (eseguite da j5mini), cinematica POE (assi di vite S, posa M, punti di disegno → `kinematics.json`, anteprima nella vista 3D, passata al gimbal ASSIST), righe `SET_VR_PARAMS` / `SET_PWM_CONFIG`; "Applica" le manda al firmware acceso, "Salva" le scrive in `j5mini.json` (con `.bak`): le pose valgono subito, il resto dal prossimo avvio |
| Test | latenza WebSocket, qualità del link SPI su 10 s, escursione di un giunto (con i numeri di `SETPOSE_DONE`), prova guidata del fungo di emergenza |
| Guida | pagina a parte (`guida.html`): convenzioni, stati, teleoperazione, cinematica, profili, esercizi per studenti |

## Sicurezza

- **STOP** sempre visibile (anche Esc o barra spaziatrice).
- I comandi che muovono il braccio sono spenti finché non si attiva "comandi di movimento"; si spengono se il
  collegamento cade e restano bloccati mentre il visore comanda (intent negli ultimi 1 s).
- I target sono limitati ai limiti dei giunti anche nella pagina; il firmware li applica comunque, insieme al fungo.

## File

| File | |
|---|---|
| `index.html`, `style.css`, `guida.html` | struttura, stile, guida |
| `app.js` | collegamento, schede, test |
| `kin.js` | cinematica POE (diretta, Jacobiano spaziale, manipolabilità, RPY, angoli virtuali/fisici), senza dipendenze: si usa anche da node |
| `view3d.js` | vista 3D su canvas |
| `camtest.js` | prova delle camere: WHEP, statistiche, nitidezza (varianza del laplaciano), esito |
| `kinematics.json` | assi di vite `S`, posa `M`, punti per il disegno (dal modello dello stack del Pi) |
| `test_kin.py` | `kin.js` contro un riferimento numpy (FK, RPY, Jacobiano vs differenze finite, punti sugli assi) |

Messaggi verso `j5mini` oltre a quelli dell'app: `get_config` → `config`, `save_config` → `config_saved`,
`get_kinematics` → `kinematics`, `save_kinematics` → `kinematics_saved`,
`ws_ping` → `ws_pong`, `cam_autofocus`, `stereo_calibrate`, `imu_campaign`, `vision_request`, `horizon_lock`; HTTPS `POST /whep/cam0|cam1` (offerta SDP, inoltrata al WHEP di MediaMTX
su `127.0.0.1:8889`; il video arriva poi in UDP 8189, aperta sul Wi-Fi dal firewall); le righe UART passano con `{"type":"uart","cmd":...}` e le risposte arrivano a tutti come
`uart_response` / `uart_unsolicited`.

## Prove

```bash
python -m pytest minimal/dashboard -q                                   # cinematica (servono node e numpy)
cd firmware/stm32 && python -m pytest ../../minimal/j5mini -q            # j5mini, anche le parti della dashboard
```
