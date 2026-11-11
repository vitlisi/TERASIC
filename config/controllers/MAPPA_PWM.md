# Mappa ruote fisiche — convenzione v2, 28 ottobre 2026

**Da questa revisione SX significa SINISTRA e DX DESTRA fisiche**, guardando
nello stesso verso di avanzamento del robot (come se si fosse a bordo).
Guardandolo frontalmente, la propria sinistra/destra appare invertita.

La definizione unica usata dai programmi è
`src_cpp/terasic_reale/hardware/robot_wheel_map.h`.
Identificativo nei nuovi log di stato: **`wheel_convention=physical_sx_dx_v2`**.

| Comando/colonna NUOVI | Ruota fisica | Periodo PWM | Duty alto | Comando | Encoder |
|---|---|---|---|---|---|
| `sx`, `pwm_sx_*`, `enc_sx_raw` | **Sinistra** | `0x60` | `0x64` | `0x68` | `0x20` |
| `dx`, `pwm_dx_*`, `enc_dx_raw` | **Destra** | `0x70` | `0x74` | `0x78` | `0x40` |

Gli indirizzi sono offset dalla base MMIO **`0xFF200000`**. Esempio:
PWM sinistra periodo `0xFF200060`, duty `0xFF200064`, comando `0xFF200068`.
Non sono numeri di pin di un connettore: per quelli serve lo schema hardware.

## Cosa cambia insieme

- Controller PD, LQR, path/endurance e H∞ e diagnostici usano lo stesso header.
- Encoder nell'ordine fisico: in avanti SX conta negativo, DX positivo.
  Configurazioni: `encoder_sx_sign -1`, `encoder_dx_sign 1`, CPR1325 invariato.
- L'estimatore conserva distanza/velocità longitudinali e usa yaw positivo
  quando la ruota destra avanza rispetto alla sinistra, coerente con MuJoCo.
  Rispetto ai vecchi dati yaw/yaw_rate cambiano segno; la fusione gyro dei
  profili operativi passa alla stessa convenzione (`yaw_gyro_sign 1`).
- Le matrici numeriche K dei profili correnti non cambiano: sono simmetriche
  nel longitudinale e antisimmetriche nello yaw. Scambiare righe e invertire
  le colonne yaw/yaw_rate lascia quei K invariati (`K_v2=P*K_v1*T`).
  Non applicare questa conclusione a matrici future non simmetriche.
- H∞ mantiene matrici e ordine export `[PWM_R,PWM_L]`: il core restituisce
  `[PWM_L,PWM_R]`, ora realmente instradati alla sinistra e alla destra fisiche.
- PWM, riferimento pitch fisso e limiti di arresto restano invariati.
- Configurazioni v1 con segni encoder vecchi sono rifiutate dai nuovi binari.

La revisione richiede una nuova validazione fisica di stabilizzazione/yaw:
non significa che sia risolta la caduta osservata. Durante installazione e
verifiche offline non vengono avviati i motori.

## Log storici: NON rinominati

I file acquisiti prima della revisione conservano **nomi FPGA legacy**:

| Vecchia etichetta | Ruota fisica | Base PWM |
|---|---|---|
| `SX` | Destra | `0x70` |
| `DX` | Sinistra | `0x60` |

Le tre prove `pwm_port_*_20261028*.status.log` sono **legacy**, anteriori
all'installazione v2. Non interpretarle secondo l'intestazione della tabella nuova.
Verifica visiva: impulso vecchio SX all'8% per300ms → ruota destra, encoder
SX +107. Vecchio DX → ruota sinistra, encoder DX −143 e −145 nella ripetizione.
Tutte COMPLETATO, comandi finali 0x06, RUN=0 e duty azzerati.
Le osservazioni dell'operatore concordano con i verbali del 16 ottobre.

Nei vecchi CSV: per convertirli in una nuova analisi, scambiare le coppie
SX/DX e invertire yaw/yaw_rate, conservando i dati originali e dichiarando
la trasformazione. Non mescolare vecchi config e nuovi binari.
