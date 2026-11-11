> Nota del 24 ottobre 2026: documento tecnico conservato. I vecchi log citati sono stati rimossi; le prestazioni storiche non sostituiscono le nuove prove.

# Connessioni per sviluppo e prove del robot

Obiettivo: caricare e provare codice sulla DE10-Nano, aprire una shell, recuperare CSV e avere una console di recupero, con pochi cavi nelle prove a terra. Il PD continua a girare localmente sulla DE10-Nano: il collegamento non entra nel ciclo di controllo.

## Aggiornamento 16 ottobre 2026: dongle USB MT7601U

Il dongle `148f:7601` è stato riconosciuto caricando temporaneamente sulla
DE10 i moduli cfg80211, mac80211, mt7601u e le dipendenze crypto compilati
sul PC dai sorgenti Terasic 4.5. Nessun kernel, bitstream o file di avvio
è stato sostituito. Dopo la prova iniziale sono stati installati soltanto i
dieci moduli mancanti in `/lib/modules/4.5.0/updates/terasic_wifi_trial`,
registrati con depmod; `/etc/modules-load.d/terasic-wifi.conf` richiede
mt7601u al boot. Dopo il riavvio sono stati verificati caricamento automatico,
riconnessione a Router66 e SSH Wi-Fi con Ethernet scollegata (carrier=0).

Su hotspot WPA2 a 2,4 GHz Router66 sono riusciti associazione, IP, SSH via
Wi-Fi e trasferimento bidirezionale di 1 MiB con hash identici. PC
10.117.7.132, DE10 10.117.7.166; questi sono indirizzi DHCP osservati, non
indirizzi garantiti. In circa un minuto: 120 ping, nessuna perdita, massimo
126,806 ms. Ethernet era ancora fisicamente collegata, ma SSH e route
confermavano l'uso di wlan0. Nessuna prova motori eseguita.

Restano da approfondire gli avvisi EEPROM emessi dal driver e da verificare
i tempi del controllo sotto traffico. Le prove di collegamento senza Ethernet
e riavvio sono riuscite; nessuna prova motori via Wi-Fi ancora eseguita.
Per eduroam (account UNINA, sede ospite Poznań) la configurazione non è
stata completata. Stato dettagliato e backup:
[registro Wi-Fi](../../maintenance/README.md).
Le sezioni del 7 ottobre sotto riportano la situazione precedente.

## Configurazione consigliata

**Al banco:** pacco Terasic, Ethernet diretta PC–DE10 per SSH/SCP e UART USB per console di avvio e recupero. Ethernet e UART hanno ruoli diversi: SSH funziona solo quando Linux e la rete sono attivi; la UART permette di vedere anche bootloader, messaggi di avvio e problemi di rete. Per compilare, caricare sorgenti ed estrarre log usare Ethernet. Per le prove del robot in movimento non lasciare cavi lunghi verso il PC.

**Per prove mobili:** piccolo router/access point alimentato sul robot, collegato con un cavo Ethernet corto alla porta `eth0` della DE10; PC connesso via Wi-Fi alla rete dedicata del router. La DE10 resta un normale host Ethernet e continuano a funzionare SSH, SCP e gli script dei log, senza modificare il kernel, il programma PD o l'ESP32. Non c'e' un cavo che va dal robot al PC, ma rimangono sul robot un cavetto Ethernet e l'alimentazione del router. La UART si collega di nuovo quando serve recuperare la console di avvio. Questa e' la scelta piu' pratica per questo progetto se si accetta un piccolo modulo aggiuntivo.

Per un router con porta **LAN** e Wi-Fi per i client, usare la sua normale modalita' router con rete locale dedicata, lasciando la porta WAN scollegata. Collegare `eth0` della DE10 alla porta LAN; il PC si collega al Wi-Fi dello stesso router. Configurare un indirizzo DE10 stabile nella sottorete LAN del router e verificarlo dopo il riavvio prima di togliere la UART. La modalita' "access point" di alcuni router presume invece un'altra rete Ethernet a monte: non e' la topologia autonoma proposta qui. Un esempio documentato con porta LAN e client Wi-Fi e' il GL.iNet Mango; il produttore indica alimentazione **5 V / 2 A**, quindi serve un'uscita 5 V regolata e dimensionata. Non collegarlo direttamente ai 12 V del pacco Terasic e non presumere che la porta USB della DE10 possa alimentarlo senza misurarne il margine. Fissare router e cavetto al telaio, lontano dalle ruote, e controllare ingombro, peso e autonomia. Il peso e la posizione del router possono cambiare il baricentro: ripetere le prove sostenute del PD dopo il montaggio.

Il Wi-Fi puo' cadere o congestionarsi: durante una prova il PD deve restare locale e l'interruttore BAL raggiungibile. Una shell SSH via router da' gli stessi comandi della shell SSH via cavo, ma **non** sostituisce la vera console UART quando Linux non si avvia. Prima di scollegare i cavi verso il PC, provare login, upload, download, avvio/arresto e riavvio con il robot fermo e i motori disabilitati.

Fonti per la topologia: [guida GL.iNet alla connessione LAN e Wi-Fi](https://docs.gl-inet.com/router/en/4/user_guide/gl-mt300n-v2/), [specifiche elettriche del Mango](https://www.gl-inet.com/en-ca/products/gl-mt300n-v2), [Terasic, porte Ethernet e UART](https://www.terasic.com.tw/wiki/BAL_Battery_Package_02_Getting_Start_Guide).

## Stato verificato il 7 ottobre 2026

- La DE10-Nano usa Linux 4.5.0. Il kernel attuale riporta `# CONFIG_CFG80211 is not set`, `# CONFIG_PPP is not set`, `# CONFIG_SLIP is not set` e `# CONFIG_TUN is not set`. `wpa_supplicant` e `pppd` esistono nello spazio utente, ma da soli non aggiungono i driver kernel mancanti.
- L'ESP32 del kit e' collegato alla **FPGA**, non compare come interfaccia `wlan0` o porta seriale Linux. Nel progetto Terasic la UART Bluetooth della FPGA e' accessibile alla CPU HPS tramite registri memory mapped; il baud rate documentato e' 115200. Linux vede soltanto `eth0` e `lo` come interfacce di rete.
- Il PC ha Bluetooth attivo e ha rilevato un dispositivo `CAR_00327`, coerente con il nome usato dal kit. Il servizio seriale e lo scambio dati bidirezionale con questo esemplare non sono ancora stati verificati. Non sono stati cambiati firmware ESP32, bitstream FPGA, kernel o configurazione di avvio.
- Stato storico al 7 ottobre 2026, prima dell'estrazione locale di `pitch_pd_core.h`: il sorgente `pitch_pd.cpp` presente sulla scheda e' stato confrontato con quello locale prima della sola correzione del testo `Uso`; entrambi accettano una durata `run` fino a 30 s. Il binario ha concluso normalmente una prova di 20 s con `Kp=5`, `Kd=0.8`, limite 15%.

## Confronto delle strade

| Percorso | File, test e log | Console di avvio | Valutazione |
| --- | --- | --- |
| **Ethernet PC–DE10** | SSH/SCP completi e gia' verificati; stabile per sviluppo al banco. | No. | Migliore per trasferimento e debug di Linux; cavo verso il robot. |
| **UART USB PC–DE10** | Shell Linux quando disponibile; trasferire file e CSV richiede strumenti aggiuntivi ed e' scomodo. | Si', anche per bootloader/kernel. | Tenere come console di recupero; non come canale principale per log. |
| **Router Wi-Fi sul robot + Ethernet corto alla DE10** | SSH/SCP e script esistenti; rete normale per Linux senza cambiare il kernel. | No. | Scelta consigliata per prove mobili; richiede router, alimentazione 5 V adeguata e fissaggio. |
| **Chiavetta Wi-Fi USB sulla DE10-Nano** | SSH/SCP e rete Linux normale dopo modifica del kernel. | No. | Soluzione integrata, ma il kernel attuale non ha `cfg80211`; richiede driver e test di avvio. |
| **ESP32 Bluetooth di fabbrica** | Canale seriale potenziale; file e CSV richiedono protocollo e programmi appositi. | No: e' collegato alla FPGA, non alla console HPS. | Non adatto come unico canale di sviluppo. Servizio seriale su questo esemplare ancora da verificare. |
| **ESP32 ponte Wi-Fi verso SSH** | SSH/SCP plausibili dopo nuovo firmware e ponte HPS–FPGA. | No. | Progetto sperimentale con limite UART; non ancora operativo. |

Se l'obiettivo e' **robustezza di debug**, usare Ethernet e UART al banco. Se l'obiettivo e' **liberta' di movimento con gli stessi strumenti**, usare il router sul robot e conservare la UART come recupero da collegare all'occorrenza. La chiavetta USB potrebbe diventare una soluzione piu' compatta dopo il lavoro sul kernel. L'ESP32 resta un esperimento separato.

## ESP32 come Wi-Fi vero e proprio

Il manuale Terasic dice che il firmware di fabbrica usa Bluetooth e UART verso la FPGA, non Wi-Fi come interfaccia Linux. Per un collegamento IP tramite ESP32 servirebbero un nuovo firmware ESP32, un trasporto fra FPGA e Linux e una rete lato HPS. Espressif fornisce un esempio PPP su UART (`eppp_link`), ma su questa immagine Linux PPP e' disabilitato; inoltre la UART dell'ESP32 non e' esposta direttamente come `/dev/tty*` alla HPS. Anche ESP-Hosted per Linux non fornisce Wi-Fi sul solo trasporto UART. Questa strada richiede lavoro aggiuntivo su firmware, FPGA/bridge e kernel. La guida Terasic per aggiornare l'ESP32 prevede una configurazione temporanea della FPGA e il ponticello di programmazione; non e' una semplice impostazione software da dare via SSH.

### Progetto del ponte Wi-Fi per SSH/SCP

```text
PC -- Wi-Fi protetto --> ESP32 (access point e server TCP)
                         |
                         UART 115200 bit/s
                         v
                       FPGA -- registri UART --> programma ponte HPS
                                                |
                                                TCP locale 127.0.0.1:22
                                                v
                                              sshd sulla DE10-Nano
```

Il firmware ESP32 di fabbrica va sostituito. Una possibilita' da provare e' ESP-AT: documenta la modalita' SoftAP e un server TCP con passaggio trasparente alla UART. L'alternativa e' firmware ESP-IDF dedicato. Sulla HPS occorre leggere/scrivere la UART della FPGA e inoltrare i byte a una connessione locale `sshd`, gestendo buffer, connessioni interrotte e avvio/arresto. Il ponte iniziale gestirebbe un client alla volta: `ssh` e `scp` andrebbero usati in sequenza. Verificare che i byte del protocollo e il controllo di flusso funzionino davvero prima di affidargli file o comandi.

Questa soluzione conserva le funzioni piu' utili del cavo (login, caricamento sorgenti, avvio prove, recupero CSV), ma non equivale a collegare Ethernet a Linux: `eth0` non avra' un IP Wi-Fi, `ping` alla DE10 e altri servizi di rete non passeranno automaticamente. Il limite fisico della UART a 115200 bit/s e' circa 11,5 kB/s prima degli overhead; il trasferimento reale sara' piu' lento. IP, SSID e porta TCP dei comandi verranno definiti solo dopo l'implementazione, non vanno presunti attivi ora.

Sequenza di sviluppo: conservare una copia del firmware originale e una via di recupero cablata; provare prima uno scambio bidirezionale di byte senza motori, poi SSH con chiavi, poi SCP di file piccoli e CSV con confronto SHA-256, infine riavvio e interruzione della radio. Solo dopo queste prove si scollegano Ethernet e UART. Il PD resta locale alla HPS e il BAL rimane l'arresto fisico: la connessione Wi-Fi non e' un dispositivo di sicurezza.

### Protezione del PC e della scheda sulle reti Wi-Fi

1. Configurare il router o l'ESP32 come rete Wi-Fi dedicata con WPA2-Personal e password lunga e unica; se dispositivo e PC lo supportano, WPA3. Evitare una rete aperta o un ponte verso la LAN domestica. Il PC deve collegarsi come client; non serve esporre un server SSH sul PC, abilitare inoltro IP o condividere Internet. Impostare anche una password amministrativa diversa da quella predefinita sul router.
2. Sulla DE10, verificare prima l'accesso con una chiave SSH, poi disabilitare password e password vuote per `sshd`. Conservare Ethernet/UART finche' il login con chiave funziona via Wi-Fi. Non cambiare l'autenticazione in una sessione remota senza una via di recupero. Verificare l'impronta della chiave host SSH al primo collegamento e non disabilitare `StrictHostKeyChecking`.
3. Sul PC, limitare le connessioni in ingresso dalla rete del robot con il firewall. I comandi `ssh` e `scp` partono dal PC e richiedono soltanto connessioni in uscita. Prima di collegarsi, controllare `ss -lnt` e lo stato del firewall (`sudo ufw status verbose` se UFW e' installato). Il 7 ottobre 2026 il PC aveva una porta `*:80` in ascolto: identificarne il servizio con `sudo ss -lntp` e limitarlo alla sola interfaccia necessaria o bloccarlo dalla rete del robot.
4. Docker non e' richiesto per `ssh`/`scp` e non sostituisce autenticazione, cifratura o firewall. Per compilare firmware con strumenti non fidati puo' essere utile un ambiente isolato; evitare l'accesso al socket Docker, `--privileged`, `--network host` e il montaggio dell'intera home o delle chiavi SSH. Docker rootless riduce i privilegi del daemon, ma non rende sicura da sola la radio.

Riferimenti per questo progetto: [Espressif ESP-AT, server TCP passthrough](https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/Basic_AT_Commands.html), [Espressif, configurazione SoftAP](https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/Wi-Fi_AT_Commands.html), [OpenSSH `sshd_config`](https://man.openbsd.org/sshd_config), [Docker Engine security](https://docs.docker.com/engine/security/), [Docker rootless](https://docs.docker.com/engine/security/rootless/).

## Verifiche prima di staccare i cavi verso il PC

1. Se si sceglie il router: alimentarlo con 5 V regolati e corrente adatta al modello; fissarlo, collegare la DE10 alla porta LAN e impostare una sottorete dedicata con IP DE10 persistente. Provare la connessione con motori disabilitati.
2. Provare prima il trasferimento di un file piccolo, poi verificare l'hash SHA-256 sul PC e sulla scheda; ripetere in entrambe le direzioni. Il caricamento del sorgente e lo scarico dei CSV devono riuscire senza il cavo Ethernet verso il PC.
3. Provare SSH, avvio/arresto del PD, riavvio della DE10 e recupero dei log. Il controllo PD resta sulla HPS; un'interruzione radio non deve togliere la possibilita' di arrestare fisicamente il robot tramite BAL.
4. Conservare un cavo UART e uno Ethernet come recupero finche' questi passaggi sono verificati piu' volte. Se in futuro si vuole sviluppare il ponte ESP32, verificare prima il canale UART FPGA–HPS a motori spenti e preparare firmware e programma ponte separatamente.

Fonti tecniche: [manuale hardware Terasic](https://www.terasic.com.tw/wiki/Word2wiki_bal_hard_src), [guida utente Terasic](https://www.terasic.com.tw/wiki/Word2wiki_bal_user_gide_src), [guida Terasic all'aggiornamento ESP32](https://www.terasic.com.tw/wiki/Self_Balancing_Robot_ESP32_Update_Guide), [Espressif eppp_link](https://github.com/espressif/esp-protocols/blob/master/components/eppp_link/README.md), [Espressif ESP-Hosted Linux](https://github.com/espressif/esp-hosted-linux).
