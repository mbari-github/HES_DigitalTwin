"""
neural_fdi_testing.launch.py
=============================

Launch file per il test del sistema Neural FDI.

Avvia il Digital Twin completo (plant + control loop) con:
  - Fault injector (configurabile su Ch0 o Ch3)
  - Nodo Neural FDI (COLLECT → TRAIN → INFERENCE)
  - Input sinusoidale per eccitare la dinamica

Flusso tipico di utilizzo:

  1. Lanciare con FAULT_ACTIVE = False
     → Il nodo FDI raccoglie dati healthy (60s), addestra le reti,
       e passa automaticamente a INFERENCE.

  2. Una volta in INFERENCE, attivare il fault a runtime:
       ros2 param set /fault_injector fault_active true

  3. Osservare la diagnosi:
       ros2 topic echo /neural_fdi/diagnosis

Remapping
---------
Il nodo Neural FDI deve leggere i segnali FAULTATI (post-injection),
esattamente come faceva l'observer model-based. In questo modo:
  - In fase COLLECT (fault disattivato): legge segnali puliti → dati healthy
  - In fase INFERENCE (fault attivato): legge segnali corrotti → rileva il fault

Canali supportati: solo Ch0 (load cell) e Ch3 (encoder).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ── Package paths ──
    description_pkg = get_package_share_directory('exoskeleton_description')
    bringup_pkg = get_package_share_directory('exoskeleton_bringup')
    neural_fdi_pkg = get_package_share_directory('exoskeleton_neural_fdi')

    # ── URDF ──
    urdf_file = os.path.join(
        description_pkg, 'urdf', 'assembly_with_hand.urdf'
    )
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # ── Config files ──
    neural_fdi_params = os.path.join(
        neural_fdi_pkg, 'config', 'neural_fdi_params.yaml'
    )

    # ============================================================
    # FAULT INJECTION CONFIGURATION
    # ============================================================
    # Canali supportati per il Neural FDI:
    #   0 → /exo_dynamics/tau_ext_theta  (load cell)
    #   3 → /joint_states                (encoder)
    #
    # I canali 1 e 2 (trajectory_ref, torque) NON sono monitorati
    # dal Neural FDI e non devono essere usati con questo launch.

    FAULT_CHANNEL       = 3             # 0 = load cell, 3 = encoder
    FAULT_TYPE          = 'offset'
    FAULT_MAGNITUDE     = -0.1          
    FAULT_ACTIVE        = False         # Attivare a runtime dopo il training!
    NOISE_STD           = 0.1
    SPIKE_DURATION      = 0.05
    TARGET_INDEX        = 0
    FAULT_JS_FIELD      = 'position'    # 'position', 'velocity', o 'both'
    JOINT_NAME          = 'rev_crank'

    # ============================================================
    # NEURAL FDI CONFIGURATION
    # ============================================================
    COLLECT_DURATION    = 60.0          # secondi di raccolta dati healthy
    SKIP_COLLECT        = True         # True se CSV già esistente
    SKIP_TRAIN          = True         # True se modelli .pt già esistenti

    # ============================================================
    # REMAPPING LOGIC
    # ============================================================

    # ── Fault injector remaps ──
    fault_injector_remaps = {
        0: [
            ('/exo_dynamics/tau_ext_theta',         '/exo_dynamics/tau_ext_theta_raw'),
            ('/exo_dynamics/tau_ext_theta_faulted',  '/exo_dynamics/tau_ext_theta_faulted'),
        ],
        3: [
            ('/joint_states',         '/joint_states'),
            ('/joint_states_faulted', '/joint_states_faulted'),
        ],
    }

    # ── Dynamics node remaps ──
    dynamics_remaps = []
    if FAULT_CHANNEL == 0:
        # Il dynamics pubblica tau_ext_theta su _raw, il fault injector
        # lo legge e pubblica su _faulted
        dynamics_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_raw')
        )

    # ── Admittance controller remaps ──
    admittance_remaps = []
    if FAULT_CHANNEL == 0:
        # L'admittance legge tau_ext faultato (così la closed-loop
        # propaga il fault come farebbe nel sistema reale)
        admittance_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_faulted')
        )
    if FAULT_CHANNEL == 3:
        admittance_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ── Trajectory controller remaps ──
    trajectory_remaps = []
    if FAULT_CHANNEL == 3:
        trajectory_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ── RViz remaps ──
    rviz_remaps = []
    if FAULT_CHANNEL == 3:
        rviz_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ── Neural FDI remaps ──
    # Il nodo FDI deve leggere i segnali faultati (come l'observer).
    # Legge sempre da:
    #   /joint_states              → posizione e velocità encoder
    #   /torque                    → coppia motore
    #   /exo_dynamics/tau_ext_theta → forza esterna
    #
    # Quando il fault è iniettato su un canale, il consumatore a valle
    # (controller) è già remappato sul topic _faulted. Il nodo FDI
    # deve fare lo stesso, così vede i segnali come li vede il controller.
    neural_fdi_remaps = []
    if FAULT_CHANNEL == 0:
        neural_fdi_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_faulted')
        )
    if FAULT_CHANNEL == 3:
        neural_fdi_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ============================================================
    # NODES
    # ============================================================

    return LaunchDescription([

        # ── Robot State Publisher ──
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        ),

        # ── Dynamics (plant simulation) ──
        Node(
            package='exoskeleton_dynamics',
            executable='exo_dynamics',
            name='dynamics',
            output='screen',
            remappings=dynamics_remaps,
        ),

        # ── RViz ──
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', os.path.join(
                    description_pkg, 'rviz', 'exo_display.rviz'
                ),
            ],
            remappings=rviz_remaps,
        ),

        # ── External input (sinusoidal torque) ──
        # L'input sinusoidale eccita la dinamica durante la raccolta
        # dati healthy. Frequenza e ampiezza di default sono configurate
        # nel nodo stesso.
        Node(
            package='exoskeleton_utils',
            executable='external_wrench_pub',
            name='input',
            output='screen',
            parameters=[{
                'frequency': 0.05,      # Hz — ciclo lento
                'tau_min': -0.8,        # Nm — picco flessione
                'tau_max': 0.0,         # Nm — zero torque
            }],
        ),

        # ── Admittance Controller (outer loop) ──
        Node(
            package='exoskeleton_control',
            executable='admittance_controller',
            name='admittance_controller',
            output='screen',
            parameters=[{
                'force_deadband': 0.0,
            }],
            remappings=admittance_remaps,
        ),

        # ── Trajectory Controller (inner loop) ──
        Node(
            package='exoskeleton_control',
            executable='trajectory_controller',
            name='trajectory_controller',
            output='screen',
            remappings=trajectory_remaps,
        ),

        # ── Fault Injector ──
        Node(
            package='exoskeleton_faults',
            executable='fault_injector',
            name='fault_injector',
            output='screen',
            parameters=[{
                'channel':          FAULT_CHANNEL,
                'fault_active':     FAULT_ACTIVE,
                'fault_type':       FAULT_TYPE,
                'fault_magnitude':  FAULT_MAGNITUDE,
                'noise_std':        NOISE_STD,
                'spike_duration':   SPIKE_DURATION,
                'target_index':     TARGET_INDEX,
                'fault_js_field':   FAULT_JS_FIELD,
                'joint_name':       JOINT_NAME,
                'publish_rate':     200.0,
            }],
            remappings=fault_injector_remaps[FAULT_CHANNEL],
        ),

        # ── Neural FDI Node ──
        Node(
            package='exoskeleton_neural_fdi',
            executable='fdi_node',
            name='neural_fdi_node',
            output='screen',
            parameters=[
                neural_fdi_params,
                {
                    'collect_duration': COLLECT_DURATION,
                    'skip_collect':     SKIP_COLLECT,
                    'skip_train':       SKIP_TRAIN,
                },
            ],
            remappings=neural_fdi_remaps,
        ),
    ])