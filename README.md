# RTAI_ABS

Implementazione  di  un  sistema  di  controllo  che  simula  le  operazioni  di  un  anti-lock  braking  system
(ABS) su di un impianto che simula la velocit di rotazione di due ruote.  L’impianto accetta in ingresso
il segnale di attuazione di accelerazione, decelerazione, frenata o nessuna-azione.
La  realizzazione  ha  previsto  la  progettazione  e  lo  sviluppo  di  un  sistema  di  controllo  composto  da
sensori, attuatori e controllori.  In particolare, i controllori rilevano l’eventuale situazione di bloccaggio
e si coordinano tra loro per attivare il meccanismo di ABS. Il corretto funzionamento dei controllori,
specie in termini dei tempi di calcolo,  supervisionato da due watchdog che lanciano un allarme in caso
di controllore spento o malfunzionante.
Progettazione:  OMG MARTE
Sviluppo:  Linux patch RTAI