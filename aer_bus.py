from typing import Dict, List, Optional, Any, Set, Tuple
import json
from enum import Enum
import multiprocessing as mp
from queue import Empty
import zmq
from component_schema import VALID_CONTROLLERS
from controller_template import Controller
from controller_pid_point_mass import PIDPointMassController
from controller_holonomic import HolonomicPIDController
from controller_cascade_PID_six_dof import CascadedPIDSixDOFController


# Maps controller name strings (from VALID_CONTROLLERS / ATC.controller_map keys)
# to their concrete Controller subclass. Add new inline controllers here.
CONTROLLER_CLASS_MAP: Dict[str, type] = {
    'PIDPointMassController':    PIDPointMassController,
    'PIDHolonomicController':    HolonomicPIDController,
    'CascadedPIDSixDOFController': CascadedPIDSixDOFController,
}

# Controller names that signal "RL training mode" — no internal action is generated;
# the gym environment supplies actions via external_actions in SimulatorManager.step().
RL_CONTROLLER_NAMES: Set[str] = {'RL'}


class ExecutionMode(Enum):
    INLINE = "inline"       # Controller runs in same Python process
    PROCESS = "process"     # Controller in separate Python subprocess (multiprocessing.Queue)
    EXTERNAL = "external"   # Controller in remote process/machine (ZeroMQ REQ-REP)


class AerBus:
    """
    Central controller orchestrator for UrbanNav.

    Supports three execution modes:
      - INLINE:   controller defined in simulator codebase, called directly each step
      - PROCESS:  controller in a spawned subprocess, communicates via multiprocessing.Queue
      - EXTERNAL: controller in a remote process (e.g. MATLAB, C++, ORCA-RVO2),
                  communicates via ZeroMQ REQ-REP sockets

    RL training mode:
      When a UAV's controller_name is in RL_CONTROLLER_NAMES, AerBus skips action
      generation for that UAV. The gym environment supplies actions via
      external_actions in SimulatorManager.step(), which are merged with internal
      actions via map_actions_to_uavs() before being dispatched to DynamicsEngine.
    """

    def __init__(self, config, controller_uav_map: Dict[str, List[int]],
                 uav_dict: Dict[int, Any], mode: str = 'deployment'):

        self.config = config
        # mapping: uav_id -> UAV object
        self.uav_dict = uav_dict
        # mapping: controller_name_str -> [uav_id1, uav_id2, ...] (from ATC.controller_map)
        self.controller_uav_map = controller_uav_map

        # INLINE controllers: uav_id -> Controller instance (one per UAV, stateful)
        self.controller_obj_map: Dict[int, Controller] = {}

        # PROCESS controllers: controller_name -> [uav_ids]
        self.process_uav_map: Dict[str, List[int]] = {}
        self.process_queues: Dict[str, Dict[str, mp.Queue]] = {}
        self.processes: Dict[str, mp.Process] = {}

        # EXTERNAL (ZeroMQ) controllers: controller_name -> {socket, port, uav_ids}
        self.zmq_context = None
        self.external_sockets: Dict[str, Dict] = {}

        # RL UAVs: actions come from gym via external_actions, skip in get_actions()
        self.rl_uav_ids: Set[int] = set()

    # ------------------------------------------------------------------
    # Registration
    # ------------------------------------------------------------------

    def register_controller(self, controller_name: str, uav_ids: List[int],
                             mode: ExecutionMode,
                             instance: Optional[Controller] = None) -> None:
        """
        Low-level wiring primitive. Wires one controller (by name and mode) to
        the given UAV IDs. Called by register_uav_controllers() and can also be
        called at runtime to add a controller mid-simulation.

        Args:
            controller_name: String key identifying the controller type.
            uav_ids:         List of UAV IDs this controller is responsible for.
            mode:            ExecutionMode (INLINE, PROCESS, or EXTERNAL).
            instance:        Pre-built Controller object (required for INLINE).
        """
        if mode == ExecutionMode.INLINE:
            if instance is None:
                raise ValueError(f'INLINE mode requires a Controller instance '
                                 f'for controller "{controller_name}"')
            for uav_id in uav_ids:
                self.controller_obj_map[uav_id] = instance

        elif mode == ExecutionMode.PROCESS:
            self._spawn_controller_process(controller_name, instance, uav_ids)
            self.process_uav_map[controller_name] = uav_ids

        elif mode == ExecutionMode.EXTERNAL:
            self._setup_external_socket(controller_name)
            self.external_sockets[controller_name]['uav_ids'] = uav_ids

        print(f'[AerBus] Registered "{controller_name}" in {mode.name} mode '
              f'for UAVs: {uav_ids}')

    def register_uav_controllers(self) -> None:
        """
        Batch orchestrator called once by SimulatorManager.reset().

        Iterates over controller_uav_map (populated by ATC._set_uav()), determines
        the execution mode for each controller name, creates instances, and
        delegates wiring to register_controller().

        Mirrors the pattern of DynamicsEngine.register_uav_dynamics() and
        PlannerEngine.register_uav_planners().
        """
        for controller_name, uav_id_list in self.controller_uav_map.items():

            if controller_name not in VALID_CONTROLLERS:
                raise ValueError(
                    f'Unknown controller "{controller_name}". '
                    f'Valid options: {sorted(VALID_CONTROLLERS)}'
                )

            if controller_name in RL_CONTROLLER_NAMES:
                # RL training mode: no internal action generation.
                # The gym environment will supply actions via external_actions.
                for uav_id in uav_id_list:
                    self.rl_uav_ids.add(uav_id)
                print(f'[AerBus] "{controller_name}" → RL mode for UAVs: {uav_id_list}')

            elif controller_name in CONTROLLER_CLASS_MAP:
                # INLINE: one stateful instance per UAV (PID keeps prev_yaw_error, etc.)
                for uav_id in uav_id_list:
                    instance = CONTROLLER_CLASS_MAP[controller_name](self.config.simulator.dt) #added dt for controller that needs to be synced with global dt
                    self.register_controller(controller_name, [uav_id],
                                             ExecutionMode.INLINE, instance=instance)

            else:
                # EXTERNAL or PROCESS: one connection per controller type serving
                # all its UAVs as a state bundle each step.
                # Default to EXTERNAL (ZeroMQ); subclass or config can override to PROCESS.
                mode = ExecutionMode.EXTERNAL
                self.register_controller(controller_name, uav_id_list,
                                         mode, instance=None)

    # ------------------------------------------------------------------
    # Step
    # ------------------------------------------------------------------

    def get_actions(self, plan_dict: Dict[int, List]) -> Dict[int, Tuple[float, float]]:
        """
        Generate control actions for all internally-controlled UAVs.

        Called each step by SimulatorManager._step_uavS(). The returned dict is
        then merged with external gym actions via map_actions_to_uavs() before
        being dispatched to DynamicsEngine.step().

        Args:
            plan_dict: Dict[uav_id, List[Point]] from PlannerEngine.get_plans().
                       plan_dict[uav_id][0] is the current target waypoint.

        Returns:
            Dict[uav_id, (accel_cmd, yaw_rate_cmd)] for all internally-controlled
            UAVs. RL UAVs are intentionally omitted.
        """
        actions_dict: Dict[int, Tuple[float, float]] = {}

        # --- INLINE controllers ---
        for uav_id, controller in self.controller_obj_map.items():
            if uav_id not in self.uav_dict or uav_id not in plan_dict:
                continue
            uav = self.uav_dict[uav_id]
            target_pos = plan_dict[uav_id][0]   # first waypoint from planner
            actions_dict[uav_id] = controller.get_control_action(uav, target_pos)

        # --- PROCESS controllers ---
        for controller_name, uav_id_list in self.process_uav_map.items():
            state_bundle = {uid: self._extract_uav_state(uid)
                            for uid in uav_id_list if uid in self.uav_dict}
            queues = self.process_queues[controller_name]
            queues['state'].put(state_bundle)
            try:
                action_bundle = queues['action'].get(timeout=0.1)
                actions_dict.update(action_bundle)
            except Empty:
                print(f'[AerBus] WARNING: process controller "{controller_name}" timed out')

        # --- EXTERNAL (ZeroMQ) controllers ---
        for controller_name, socket_info in self.external_sockets.items():
            uav_ids = socket_info.get('uav_ids', [])
            state_bundle = {uid: self._extract_uav_state(uid)
                            for uid in uav_ids if uid in self.uav_dict}
            socket_info['socket'].send_json(state_bundle)
            try:
                action_bundle = socket_info['socket'].recv_json(flags=zmq.NOBLOCK)
                actions_dict.update(action_bundle)
            except zmq.Again:
                print(f'[AerBus] WARNING: external controller "{controller_name}" '
                      f'not responding')

        # RL UAVs intentionally omitted — gym supplies via external_actions in step()
        return actions_dict

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _extract_uav_state(self, uav_id: int) -> Dict:
        """Serialize a UAV's state for transport to PROCESS or EXTERNAL controllers."""
        uav = self.uav_dict[uav_id]
        return {
            'id':      uav_id,
            'x':       uav.current_position.x,
            'y':       uav.current_position.y,
            'speed':   uav.current_speed,
            'heading': uav.current_heading,
            'vx':      uav.vx,
            'vy':      uav.vy,
        }

    def _spawn_controller_process(self, controller_name: str,
                                   controller: Optional[Controller],
                                   uav_ids: List[int]) -> None:
        """Spawn a subprocess for a PROCESS-mode controller."""
        state_queue = mp.Queue()
        action_queue = mp.Queue()

        process = mp.Process(
            target=self._controller_process_worker,
            args=(controller, state_queue, action_queue)
        )
        process.start()

        self.process_queues[controller_name] = {
            'state':  state_queue,
            'action': action_queue,
        }
        self.processes[controller_name] = process

    @staticmethod
    def _controller_process_worker(controller, state_queue: mp.Queue,
                                    action_queue: mp.Queue) -> None:
        """Worker running in a separate process for PROCESS-mode controllers."""
        while True:
            try:
                state_bundle = state_queue.get(timeout=1.0)
                if state_bundle is None:    # shutdown signal
                    break
                action_bundle = controller.get_control_action(state_bundle)
                action_queue.put(action_bundle)
            except Empty:
                continue
            except Exception as e:
                print(f'[AerBus worker] Error: {e}')
                action_queue.put({})

    def _setup_external_socket(self, controller_name: str) -> None:
        """Create a ZeroMQ REP socket for an EXTERNAL-mode controller."""
        if self.zmq_context is None:
            self.zmq_context = zmq.Context()

        socket = self.zmq_context.socket(zmq.REP)
        port = 5555 + len(self.external_sockets)
        socket.bind(f'tcp://*:{port}')

        self.external_sockets[controller_name] = {
            'socket': socket,
            'port':   port,
        }
        print(f'[AerBus] External controller "{controller_name}" '
              f'listening on port {port}')

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Reset all controllers (important for RL episode boundaries)."""
        for controller in self.controller_obj_map.values():
            if hasattr(controller, 'reset'):
                controller.reset()

        for queues in self.process_queues.values():
            queues['state'].put({'reset': True})

        for socket_info in self.external_sockets.values():
            socket_info['socket'].send_json({'reset': True})
            socket_info['socket'].recv_json()   # wait for ACK

    def shutdown(self) -> None:
        """Clean shutdown of all subprocess and ZeroMQ resources."""
        for process in self.processes.values():
            process.terminate()
            process.join(timeout=1.0)

        for socket_info in self.external_sockets.values():
            socket_info['socket'].close()

        if self.zmq_context:
            self.zmq_context.term()


# ---------------------------------------------------------------------------
# ActionMerger (future)
# ---------------------------------------------------------------------------
# When multiple controllers claim the same UAV (e.g. a safety override layer
# on top of a learned policy), ActionMerger will resolve conflicts using a
# priority list before the action dict is dispatched to DynamicsEngine.
# Not implemented yet — SimulatorManager.map_actions_to_uavs() handles the
# simpler case of merging internal vs. gym-supplied actions today.
