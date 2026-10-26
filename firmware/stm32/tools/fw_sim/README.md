# fw_sim — il firmware JONNY5 sul PC

Compila i **sorgenti veri** del firmware (`src/`) come libreria per il PC e li fa girare a 1 kHz in
tempo simulato, con hardware finto: orologio, pin GPIO (E-STOP PC6, pulsante PC13), UART di
controllo, PWM a 1 MHz come i timer del G474, DMA SPI circolare emulato byte per byte, IMU impostabile.
Non compilati: `hal_spi_slave.c` (HAL/DMA), `imu.c`/`bno085.c`, `main.c`, `fatal_diag.c`.

```bash
cd firmware/stm32
python tools/fw_sim/fw_sim.py build            # build/fw_sim.dll (Windows, gcc MinGW) o libfw_sim.so
python -m pytest tools/fw_sim -q               # 14 test
```

Test differenziale contro l'originale: estrarre i sorgenti dell'originale e indicarli in `FW_SIM_ORIG_SRC`.

```bash
git archive 22b6a46 firmware/stm32/src | tar -x -C /tmp/orig
FW_SIM_ORIG_SRC=/tmp/orig/firmware/stm32/src python -m pytest tools/fw_sim -q
```

L'originale non ha un suo `spi_frames.c` (la logica stava in `hal_spi_slave.c` insieme all'HAL): per
compilarlo si usa la copia congelata in `orig/spi/`, spostata parola per parola e con v2/J5IK.

Uso da Python (lo stesso che usano i test e `minimal/j5mini/test_j5mini.py`):

```python
import fw_sim
s = fw_sim.Sim()                       # boot: BOOT_READY, SAFE, PWM a 0
s.command("SET_OFFSETS 100 88 93 95 90 95")   # -> ['OK SET_OFFSETS']
r = s.spi(fw_sim.pad(fw_sim.j5_frame(0x04, 1, fw_sim.j5vr_payload(mode=2, hb=1)), s.frame_len))
s.run(10)                              # 10 ms di firmware
s.pwm(), s.angles(), s.state()         # impulsi µs, angoli comandati, SAFE/IDLE/STOPPED
s.estop(True)                          # fungo premuto
```

Ogni `Sim()` carica una copia nuova della libreria: lo stato statico del firmware riparte da zero.
