# HES DigitalTwin — Digital Twin di un Esoscheletro per la Mano

<!-- PLACEHOLDER: screenshot di RViz con il modello dell'esoscheletro in movimento (assembly_with_hand), possibilmente con il pannello /joint_states visibile -->

Digital Twin ROS 2 di un esoscheletro per la riabilitazione della mano (Hand Exoskeleton System), sviluppato come tesi magistrale. Il progetto comprende modello dinamico ridotto a 1-DOF con chiusura cinematica, architettura di controllo a cascata ammittanza-traiettoria, sistema di sicurezza funzionale a tre livelli, modulo di fault detection model-based a quattro observer e framework di fault injection per la validazione sistematica.

---

## Indice

- [Architettura del sistema](#architettura-del-sistema)
- [Pacchetti ROS 2](#pacchetti-ros-2)
- [Prerequisiti](#prerequisiti)
- [Installazione](#installazione)
- [Launch files](#launch-files)
- [Configurazione](#configurazione)
- [Topic principali](#topic-principali)
- [Servizi ROS 2](#servizi-ros-2)
- [Fault Injection](#fault-injection)
- [Struttura del workspace](#struttura-del-workspace)
- [Note per lo sviluppo futuro](#note-per-lo-sviluppo-futuro)

---

## Architettura del sistema

Il sistema segue un'architettura a livelli con separazione netta tra plant, controllo, enforcement e supervisione.

<!-- PLACEHOLDER: diagramma a blocchi dell'architettura completa, mostrando il flusso dei topic tra dynamics → admittance_controller → trajectory_controller → exo_bridge → dynamics, con observer e state_machine collegati lateralmente. Idealmente lo stesso schema usato nella tesi. -->



### Flusso con bridge e safety manager

Quando il bridge è attivo, tutti i segnali di controllo passano attraverso `ExoBridge`, che applica limiti di coppia/velocità in base alla modalità corrente. La `StateMachine` monitora lo stato del bridge e può forzare transizioni di sicurezza.

---

## Pacchetti ROS 2

### `exoskeletron_description`

Modello URDF dell'esoscheletro con mesh STL e configurazioni RViz.

- `urdf/assembly.urdf` — modello solo meccanismo (senza mano)
- `urdf/assembly_with_hand.urdf` — modello completo con falangi (palmo, prossimale, mediale, distale)
- `meshes/` — STL dei link: body, link_AC, link_BC, link_CE, rear_crank, connecting_rod, slider_t, slider_f, thimble, e componenti della mano
- `rviz/` — configurazioni di visualizzazione per RViz2
- `launch/display_exo_solo.launch.py` — visualizzazione del solo meccanismo
- `launch/display_exo_hand.launch.py` — visualizzazione con mano

Il modello gestisce le **catene cinematiche chiuse** tramite una strategia di apertura della catena: i giunti di chiusura sono sostituiti da coppie di frame di riferimento la cui coincidenza è imposta a livello software dal risolutore di chiusura.

### `exoskeletron_dynamics`

Plant dinamico ridotto a 1-DOF. Nodo principale: `ExoDynamicsControlTest`.

**Equazione del moto ridotta:**

```
M_eff · θ̈ = τ_m + τ_pass + τ_ext − proj − τ_fric − τ_damp
```

dove `M_eff = Bᵀ M B + Jm`, `proj = Bᵀ(M Ḃ θ̇ + h)`, e `B = dq/dθ` è il vettore cinematico istantaneo calcolato via proiezione Jacobiana.

**Caratteristiche principali:**
- Chiusura cinematica risolta ad ogni step via `scipy.optimize.least_squares` (metodo TRF con bounds)
- Modello passivo delle falangi tipo Fung (rigidezza esponenziale + smorzamento)
- Attrito viscoso + Coulomb (tanh) + smorzamento addizionale
- Gestione finecorsa con stato AT_LIMIT e logica di release
- Integrazione Euler esplicito a 1 kHz, pubblicazione a 200 Hz
- Modello Pinocchio caricato da URDF

**Executables:**
- `dynamics_stripped` — versione semplificata (tau_ext come override scalare)
- `exo_dynamics` — versione completa con wrench esterna cartesiana

### `exoskeletron_control`

Architettura di controllo a cascata con due nodi indipendenti.

**Admittance Controller** (outer loop) — Modello virtuale M-D-K che converte la forza utente in un riferimento di traiettoria:

```
M · θ̈_v + D · θ̇_v + K · (θ_v − θ_eq) = τ_ext_theta
```

L'output `[θ_v, θ̇_v, θ̈_v]` è il riferimento per l'inner loop. Supporta il meccanismo di freeze/unfreeze per la sincronizzazione con il bridge in modalità STOP.

**Trajectory Controller** (inner loop) — PD + feed-forward completo derivato dal modello del plant:

```
τ_ff = M_eff · θ̈_ref + proj − τ_pass + fric_visc · θ̇_ref + damping · θ̇_ref
τ_m  = Kp · e_θ + Kd · e_θ̇ + τ_ff
```

L'attrito Coulomb **non** viene compensato nel feed-forward (causa sovracompensazione). In modalità STOP, il controller mantiene solo la componente gravitazionale statica (`g_proj` snapshot pre-stop).

### `exoskeletron_supervision`

**ExoBridge** — Gateway di sicurezza tra controller e plant. Tutte le coppie e i riferimenti di traiettoria passano attraverso questo nodo.

**Modalità operative:**
- `nominal` — passthrough trasparente
- `torque_limit` — clamp coppia a `±override_tau_limit`
- `compliant` — clamp coppia, velocità e accelerazione
- `stop` — coppia zero o holding statico (configurabile via `stop_mode`)

**Funzionalità:**
- Watchdog per-canale con timeout configurabile e periodo di grazia all'avvio
- Pubblicazione freeze/unfreeze per l'admittance controller
- Client per richiesta safe stop alla state machine
- Status completo su `/exo_bridge/status` (9 campi, inclusa coppia pre-clamp per il downgrade)

### `exoskeletron_safety_manager`

State machine plugin-based in C++ che gestisce la sicurezza funzionale.

**Stati di sicurezza:**
- `FAULT_MONITOR` — monitoraggio attivo, nessuna restrizione
- `COMPLIANT_MODE` — limiti di coppia e velocità ridotti
- `TORQUE_LIMIT_MODE` — solo limite di coppia
- `SAFE_STOP` — arresto di sicurezza (latchabile)
- `SENSOR_DEGRADED_MODE` — riservato per integrazione futura con gli observer

**Meccanismi di protezione:**
- Escalation basata su soglie di coppia e velocità con debounce configurabile
- Downgrade automatico opzionale (contatore positivi/negativi con peso asimmetrico)
- Supervisione liveness del bridge (timeout su `/exo_bridge/status`)
- Coerenza modalità bridge (mismatch counter con soglia)
- Fault latching con reset esplicito via servizio

### `exoskeletron_observers`

Modulo di fault detection model-based con quattro livelli complementari.

| Observer | Principio | Canale coperto | Latenza |
|---|---|---|---|
| **Luenberger** | Predice θ̂, θ̇_hat e misura l'innovazione | Encoder (canale 3), Attuatore (canale 2) | ~1–1.5 s |
| **Inversione ammittanza** | Ricostruisce τ̂_ext dal riferimento di traiettoria | Sensore forza (canale 0) | ~10–50 ms |
| **Momento generalizzato** | Stima discrepanza coppia senza derivare θ̈ | Tutti i canali di coppia | ~40–60 ms |
| **Derivata τ_ext** | Monitora \|Δτ_ext/Δt\| per salti non fisiologici | Canale 0 (impulsi) | ~5 ms (1 campione) |

> **Stato attuale:** il modulo è operativo e pubblica residui in tempo reale, ma **non è ancora collegato alla catena decisionale della safety**. L'integrazione tramite `SENSOR_DEGRADED_MODE` è il prossimo passo.

### `exoskeletron_faults`

Framework di fault injection controllata per la validazione del sistema di sicurezza.

**Canali di iniezione:**

| Canale | Topic | Scenario |
|---|---|---|
| 0 | `/exo_dynamics/tau_ext_theta` | Guasto sensore forza / bias wrench |
| 1 | `/trajectory_ref` | Outer loop bloccato / riferimento corrotto |
| 2 | `/torque` | Errore inner loop / coppia non corrispondente |
| 3 | `/joint_states` | Guasto encoder / drift posizione |

**Tipi di fault:** `none`, `offset`, `noise`, `freeze`, `scale`, `spike`

Tutti i parametri sono modificabili a runtime via `ros2 param set` senza riavviare il nodo.

### `exoskeletron_safety_msgs`

Messaggi e servizi custom:
- `SafetyStatus.msg` — stato completo della state machine (stato, fault, downgrade, bridge supervision)
- `SetMode.srv` — richiesta cambio modalità al bridge (`nominal`/`compliant`/`torque_limit`/`stop`)

### `exoskeletron_utils`

Utility per testing e sviluppo:
- `external_wrench_pub` — pubblica wrench sinusoidale cartesiana
- `external_wrench_sine` — pubblica tau_ext sinusoidale scalare
- `external_wrench_step` — pubblica tau_ext a gradino (ramp-and-hold)
- `GUI` — GUI Tkinter minimale per comandare coppia via slider
- `logger` — logger CSV multi-topic per analisi offline

### `exoskeletron_bringup`

Launch files e configurazioni YAML centralizzate.

---

## Prerequisiti

- **ROS 2 Humble** (o successivo)
- **Python 3.10+**
- **Pinocchio** (`pip install pin` oppure dal PPA robotpkg)
- **NumPy**, **SciPy**
- Pacchetti ROS 2: `robot_state_publisher`, `rviz2`, `pluginlib`

---

## Installazione

```bash
# Clona nella cartella src del tuo workspace ROS 2
cd ~/ros2_ws/src
git clone https://github.com/<your-org>/HES_DigitalTwin.git

# Installa le dipendenze
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

---

## Launch files

### `testing.launch.py` — Test base del plant + controllo

Avvia dynamics, admittance controller, trajectory controller, RViz, e input step. Nessun bridge né safety.

```bash
ros2 launch exoskeletron_bringup testing.launch.py
```

### `control_loop_only.launch.py` — Solo loop di controllo

Simile a `testing.launch.py` ma usa `dynamics_stripped` (versione semplificata del plant).

```bash
ros2 launch exoskeletron_bringup control_loop_only.launch.py
```

### `testing_bridge.launch.py` — Sistema completo con bridge e safety

Avvia l'intero stack: plant, controller, bridge, state machine (con ritardo 5s), e opzionalmente il fault injector.

```bash
ros2 launch exoskeletron_bringup testing_bridge.launch.py
```

Per configurare la fault injection, modificare le variabili nella sezione `CONFIGURAZIONE FAULT INJECTION` in testa al file:

```python
FAULT_CHANNEL   = 0        # 0=tau_ext, 1=traj_ref, 2=torque, 3=joint_states
FAULT_TYPE      = 'offset'
FAULT_MAGNITUDE = -0.5
FAULT_ACTIVE    = False     # Attivare a runtime con ros2 param set
```

### `injection_testing.launch.py` — Test fault injection senza bridge

Plant + controller + observer + fault injector, senza bridge né safety manager.

```bash
ros2 launch exoskeletron_bringup injection_testing.launch.py
```

---

## Configurazione

Tutti i parametri sono centralizzati in `exoskeletron_bringup/config/`:

| File | Descrizione |
|---|---|
| `dynamics_params.yaml` | Attrito (viscoso, Coulomb, eps), smorzamento, inerzia motore, limiti cinematici |
| `exo_bridge_params.yaml` | Rate, limiti coppia per modalità, comportamento in STOP, watchdog |
| `observer_params.yaml` | Poli Luenberger, parametri ammittanza per inversione, guadagno momentum, soglia rate |
| `safety_params.yaml` | Soglie coppia/velocità per escalation e downgrade, debounce, liveness bridge |

> **Importante:** i parametri di attrito in `observer_params.yaml` **devono corrispondere** a quelli in `dynamics_params.yaml`, altrimenti i residui degli observer saranno sistematicamente biased.

---

## Topic principali

### Plant

| Topic | Tipo | Descrizione |
|---|---|---|
| `/joint_states` | `JointState` | Posizione, velocità, coppia di tutti i giunti |
| `/exo_dynamics/debug` | `Float64MultiArray` | 15 campi diagnostici (θ, θ̇, θ̈, τ_m, solver, limit, ...) |
| `/exo_dynamics/ff_terms` | `Float64MultiArray` | `[M_eff, proj, g_proj, tau_pass_theta]` |
| `/exo_dynamics/tau_ext_theta` | `Float64` | Coppia esterna proiettata su θ |

### Controllo

| Topic | Tipo | Descrizione |
|---|---|---|
| `/trajectory_ref` | `Float64MultiArray` | `[θ_ref, θ̇_ref, θ̈_ref]` |
| `/torque` | `Float64` | Coppia comandata al plant |
| `/admittance/debug` | `Float64MultiArray` | 11 campi diagnostici ammittanza |
| `/traj_ctrl/debug` | `Float64MultiArray` | 13 campi diagnostici trajectory controller |

### Bridge e Safety

| Topic | Tipo | Descrizione |
|---|---|---|
| `/exo_bridge/status` | `Float64MultiArray` | 9 campi: is_stop, is_limited, θ, θ̇, θ_hold, τ_out, τ_ext, mode_id, τ_raw |
| `/exo_bridge/mode` | `String` | Modalità corrente del bridge |
| `/admittance/freeze` | `Bool` | Freeze/unfreeze dell'admittance controller |
| `/safety_manager/status` | `SafetyStatus` | Stato completo della state machine |

### Observer

| Topic | Tipo | Descrizione |
|---|---|---|
| `/observer/state_residual` | `Float64` | Innovazione Luenberger |
| `/observer/torque_residual` | `Float64` | Residuo inversione ammittanza |
| `/observer/momentum_residual` | `Float64` | Residuo momento generalizzato |
| `/observer/tau_ext_rate_alarm` | `Float64` | 1.0 se \|dτ/dt\| > soglia |
| `/observer/debug` | `Float64MultiArray` | 21 campi diagnostici completi |

---

## Servizi ROS 2

| Servizio | Tipo | Descrizione |
|---|---|---|
| `/bridge/set_mode` | `SetMode` | Cambia modalità bridge (nominal/compliant/torque_limit/stop) |
| `/safe_stop_request` | `SetBool` | Richiede safe stop alla state machine |
| `/compliant_mode_request` | `SetBool` | Attiva/disattiva compliant mode |
| `/torque_limit_request` | `SetBool` | Attiva/disattiva torque limit mode |
| `/reset_safety_request` | `Trigger` | Reset della state machine (clear fault latch) |

**Esempio — attivare compliant mode:**

```bash
ros2 service call /compliant_mode_request std_srvs/srv/SetBool "{data: true}"
```

---

## Fault Injection

### Attivazione a runtime

```bash
# Configura il tipo di fault
ros2 param set /fault_injector fault_type offset
ros2 param set /fault_injector fault_magnitude 2.0

# Attiva
ros2 param set /fault_injector fault_active true

# Monitora
ros2 topic echo /fault_injector/status

# Disattiva
ros2 param set /fault_injector fault_active false
```

### Remapping

Il fault injector si inserisce come proxy trasparente tra produttore e consumatore del topic. I launch file gestiscono automaticamente il remapping in base al canale scelto. Lo schema è:

```
[Produttore] ──► /topic_raw ──► [FaultInjector] ──► /topic_faulted ──► [Consumatore]
```

---

## Struttura del workspace

```
HES_DigitalTwin/
├── exoskeletron_bringup/          # Launch files e configurazioni YAML
│   ├── config/
│   │   ├── dynamics_params.yaml
│   │   ├── exo_bridge_params.yaml
│   │   ├── observer_params.yaml
│   │   └── safety_params.yaml
│   └── launch/
│       ├── testing.launch.py
│       ├── testing_bridge.launch.py
│       ├── control_loop_only.launch.py
│       └── injection_testing.launch.py
├── exoskeletron_control/          # Admittance + Trajectory controller
├── exoskeletron_description/      # URDF, mesh STL, configurazioni RViz
├── exoskeletron_dynamics/         # Plant dinamico (Pinocchio-based)
├── exoskeletron_faults/           # Framework fault injection
├── exoskeletron_observers/        # Observer per fault detection
├── exoskeletron_safety_manager/   # State machine C++ (pluginlib)
├── exoskeletron_safety_msgs/      # Messaggi e servizi custom
├── exoskeletron_supervision/      # ExoBridge (gateway di sicurezza)
└── exoskeletron_utils/            # GUI, logger, generatori di input
```

---

## Note per lo sviluppo futuro

1. **Integrazione observer → safety:** collegare i residui degli observer alla state machine tramite lo stato `SENSOR_DEGRADED_MODE` (già riservato nell'enum). Serve: calibrazione soglie, logica decisionale, validazione sistematica.

2. **Hardware-in-the-loop:** l'integrazione con il prototipo fisico è prevista tramite l'architettura WebSocket distribuita già sviluppata dal partner industriale. I requisiti di interfaccia (canali, sensori, latenze) sono documentati nella tesi.

3. **Multi-dito:** l'architettura è attualmente single-finger. L'estensione a tre dita richiede la replicazione dei nodi dynamics/control per ciascun canale e la gestione della coordinazione nella state machine.

4. **Identificazione parametri:** i parametri di attrito e inerzia sono attualmente stimati. Un'identificazione sperimentale su hardware migliorerebbe la fedeltà del digital twin e la sensibilità degli observer.

---

## Riferimenti

- Bartalucci, *"A Kinaesthetic Hand Exoskeleton System Toward Robot-augmented Rehabilitation Therapies in the Health 4.0 era"* — tesi di riferimento per il design meccanico
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) — libreria di dinamica dei corpi rigidi
- [ROS 2 Humble](https://docs.ros.org/en/humble/) — framework robotico