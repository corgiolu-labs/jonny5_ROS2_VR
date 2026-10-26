# Dashboard JONNY5 (banco)

Pannello web minimale per diagnostica, comandi, impostazioni e test del braccio, pensato anche come banco di prova
per chi studia la robotica su un 6R. Una pagina statica (HTML + JavaScript, nessuna libreria esterna: il robot non
ha internet) servita da `j5mini`, che parla con lo STM32. Gli stessi messaggi WebSocket dell'app del visore.

## Aprirla

| Dove | Indirizzo |
|---|---|
| Robot, dall'hotspot JONNY5 | `https://10.42.0.1/` |
| Robot, dal PC sul cavo | `https://192.168.137.221/` |
| Senza robot (simulatore) | `python3 minimal/j5mini/j5mini.py --sim` poi `http://localhost:8558/` |

Sul robot il certificato è autofirmato: la prima volta il browser chiede di accettarlo. La pagina si collega al
WebSocket su `:8557`; se resta "non collegato", aprire anche `https://<robot>:8557` e accettare lì.
`?ws=wss://host:porta` forza un altro indirizzo.

Il simulatore (`--sim`) compila il **firmware vero** per il PC (`firmware/stm32/tools/fw_sim`, serve `gcc`) e lo fa
girare in tempo reale al posto di SPI e UART: stati, limiti, traiettorie, E-STOP e risposte sono quelli del robot.
Le impostazioni salvate in simulazione restano in memoria (non riscrive `j5mini.example.json`).

## Schede

| Scheda | Cosa |
|---|---|
| Stato | stato del firmware e bit di diagnostica (E-STOP, deadman, armato, freeze, guard), qualità del link SPI/UART, chi comanda (visore), IMU, telecamere e collimatore, angoli dei giunti, grafico degli ultimi 60 s |
| Camere | le due camere dal vivo (WebRTC da MediaMTX, segnalazione attraverso j5mini sulla porta HTTPS), esito per occhio (≥ 20 fps, < 1 % perdite), risoluzione, bitrate, jitter, decodifica, nitidezza e luminosità a confronto, istantanee; autofocus, tele/grandangolo, zoom, fuoco, luminosità |
| Braccio | 6 slider in gradi virtuali dentro i limiti, "Vai" con tempo e profilo (`SETPOSE_T_HR`), modalità "segui gli slider", pose salvate nel browser (sequenze, esporta/importa), vista 3D misurato/target, posa dell'end-effector (mm, RPY) e manipolabilità |
| Comandi | pulsanti dei comandi del firmware, console UART con cronologia, registro delle risposte e dei messaggi spontanei |
| Impostazioni | offset (HOME), verso, limiti fisici, righe `SET_VR_PARAMS` / `SET_PWM_CONFIG`; "Applica" le manda al firmware acceso, "Salva" le scrive in `j5mini.json` (con `.bak`) e valgono dal prossimo avvio |
| Test | latenza WebSocket, qualità del link SPI su 10 s, escursione di un giunto (con i numeri di `SETPOSE_DONE`), prova guidata del fungo di emergenza |
| Guida | convenzioni, stati, cinematica, profili, esercizi per studenti |

## Sicurezza

- **STOP** sempre visibile (anche Esc o barra spaziatrice).
- I comandi che muovono il braccio sono spenti finché non si attiva "comandi di movimento"; si spengono se il
  collegamento cade e restano bloccati mentre il visore comanda (intent negli ultimi 1 s).
- I target sono limitati ai limiti dei giunti anche nella pagina; il firmware li applica comunque, insieme al fungo.

## File

| File | |
|---|---|
| `index.html`, `style.css` | struttura e stile |
| `app.js` | collegamento, schede, test |
| `kin.js` | cinematica POE (diretta, Jacobiano spaziale, manipolabilità, RPY, angoli virtuali/fisici), senza dipendenze: si usa anche da node |
| `view3d.js` | vista 3D su canvas |
| `camtest.js` | prova delle camere: WHEP, statistiche, nitidezza (varianza del laplaciano), esito |
| `kinematics.json` | assi di vite `S`, posa `M`, punti per il disegno (dal modello dello stack del Pi) |
| `test_kin.py` | `kin.js` contro un riferimento numpy (FK, RPY, Jacobiano vs differenze finite, punti sugli assi) |

Messaggi verso `j5mini` oltre a quelli dell'app: `get_config` → `config`, `save_config` → `config_saved`,
`ws_ping` → `ws_pong`, `cam_autofocus`; HTTPS `POST /whep/cam0|cam1` (offerta SDP, inoltrata al WHEP di MediaMTX
su `127.0.0.1:8889`; il video arriva poi in UDP 8189, aperta sul Wi-Fi dal firewall); le righe UART passano con `{"type":"uart","cmd":...}` e le risposte arrivano a tutti come
`uart_response` / `uart_unsolicited`.

## Prove

```bash
python -m pytest minimal/dashboard -q                                   # cinematica (servono node e numpy)
cd firmware/stm32 && python -m pytest ../../minimal/j5mini -q            # j5mini, anche le parti della dashboard
```
