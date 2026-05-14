import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# =============================================================================
# TELEMETRY GUI (OPTIMIZED FOR 50Hz SIMULATION)
# =============================================================================
class DroneTelemetry:
    def __init__(self):
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(show=True, title="Drone Telemetry")
        self.win.resize(750, 650)
        self._last_gui_update = 0  # Timer to prevent GUI lag
 
    def add_plot_row(self, title, curve_names):
        p = self.win.addPlot(title=title)
        p.setXRange(0, 20, padding=0)
        p.showGrid(x=True, y=True)
        p.setLabel('bottom', "Time", units='s')
        vb = self.win.addViewBox()
        self.win.ci.layout.setColumnMaximumWidth(1, 80)
        legend = pg.LegendItem(offset=(0, 0))
        legend.setParentItem(vb)
        curves = []
        colors = ['r', 'g', 'b']
        for i, name in enumerate(curve_names):
            c = p.plot(pen=colors[i], name=name)
            legend.addItem(c, name)
            curves.append(c)
        self.win.nextRow()
        return p, curves
 
    def init_plots(self):
        self.p1, [self.pe_c] = self.add_plot_row("Pos Error (m)", ["Err"])
        self.p2, [self.ye_c] = self.add_plot_row("Yaw Error (rad)", ["Err"])
        self.p3, self.vc_cs = self.add_plot_row("Body Vel Cmd (m/s)", ["vx_c", "vy_c", "vzc_c"])
        self.p4, self.va_cs = self.add_plot_row("World Act Vel (m/s)", ["vx", "vy", "vz"])
        self.t_integral = 0.0
        self.last_target = None
        self.data = {k: [] for k in ['t', 'pe', 'ye', 'vxc', 'vyc', 'vzc', 'vxa', 'vya', 'vza']}

    def update(self, ctrl, dt, pe, ye, v_cmd, v_act, target):
        if self.last_target != tuple(target):
            self.data = {k: [] for k in ['t', 'pe', 'ye', 'vxc', 'vyc', 'vzc', 'vxa', 'vya', 'vza']}
            self.t_integral = 0.0
            self.last_target = tuple(target)
        
        self.t_integral += dt
        d = self.data
        d['t'].append(self.t_integral)
        d['pe'].append(pe); d['ye'].append(ye)
        for i, k in enumerate(['vxc', 'vyc', 'vzc']): d[k].append(v_cmd[i])
        for i, k in enumerate(['vxa', 'vya', 'vza']): d[k].append(v_act[i])
        
        # --- GUI THROTTLING ---
        # Update plots at 10Hz (0.1s) even if controller runs at 50Hz (0.02s)
        now = time.time()
        if now - self._last_gui_update > 0.1:
            self._last_gui_update = now
            t = np.array(d['t'])
            self.pe_c.setData(t, d['pe'])
            self.ye_c.setData(t, d['ye'])
            for i, c in enumerate(self.vc_cs): c.setData(t, d[['vxc', 'vyc', 'vzc'][i]])
            for i, c in enumerate(self.va_cs): c.setData(t, d[['vxa', 'vya', 'vza'][i]])
            self.app.processEvents()

telemetry = DroneTelemetry()
telemetry.init_plots()

# =============================================================================
# CASCADED PID CONTROLLER (REPRODUCED FROM PDF)
# =============================================================================
def controller(state, target_pos, dt, wind_enabled=False):
    # Consolidate all parameters for easy viewing
    TUNABLES = {
        "outer_kp_xy": 1.530567863366658 if wind_enabled else 2.565216,
        "outer_kd_xy": 0.9698080780031638 if wind_enabled else 0.156266,
        "outer_kp_z": 2.5883019100492044 if wind_enabled else 3.159015,
        "inner_kp_xy": 0.7390612815796493 if wind_enabled else 1.207636,
        "inner_kd_xy": 0.028386500076500523 if wind_enabled else 0.142323,
        "inner_kp_z": 1.6755877906089918 if wind_enabled else 1.719384,
        "alpha_v": 0.3632529222761293 if wind_enabled else 0.257418,
        "alpha_cmd": 0.3 if wind_enabled else 0.059864,
        "v_int_clip": np.array([0.4, 0.4, 0.08]) if wind_enabled else np.array([0.25, 0.25, 0.06]),
        "yaw_kp": 0.6, "vmax_xy": 0.45, "vmax_z": 0.18,
        "max_delta_vxy": 0.25, "max_delta_vz": 0.06,
        "startup_seconds": 1.2, "startup_cycles": 15,
        "hover_vz": 0.12, "min_safe_alt": 0.35, "on_ground_z": 0.08,
        "measured_v_clip": np.array([0.6, 0.6, 0.25])
    }

    if not hasattr(controller, "_init"):
        controller._init = True
        controller.prev_pos = None
        controller.vel_filter = np.zeros(3)
        controller.prev_vel_cmd = np.zeros(3)
        controller.prev_pos_err = np.zeros(3)
        controller.pos_int = np.zeros(3)
        controller.vel_int = np.zeros(3)
        controller.prev_vel_err = np.zeros(3)
        controller.last_target = None
        controller.t_since_target_change = 0.0
        controller._last_print_time = 0.0
        controller._summary_done = False
        controller.mark_pos_errs = []; controller.mark_yaw_errs = []
        controller._window_summary_done = False
        controller._startup_time = None; controller._startup_cycles = 0

    dt = float(max(dt, 1e-6))
    s = np.asarray(state).flatten()
    curr_p = s[:3].astype(float)
    if controller.prev_pos is None: controller.prev_pos = curr_p.copy()
    current_yaw = float(s[5]) if s.size >= 6 else (float(s[3]) if s.size >= 4 else 0.0)

    tx, ty, tz, tyaw = target_pos
    targ_p = np.array([tx, ty, tz], dtype=float)
    targ_yaw = float(tyaw)

    # New Target Logic
    new_target = (round(float(tx), 6), round(float(ty), 6), round(float(tz), 6), round(float(tyaw), 6))
    if controller.last_target is None or new_target != controller.last_target:
        controller.last_target = new_target
        controller.t_since_target_change = 0.0
        controller.vel_filter[:] = 0.0; controller.prev_vel_cmd[:] = 0.0
        controller.pos_int[:] = 0.0; controller.vel_int[:] = 0.0
        controller.prev_pos_err[:] = 0.0; controller.prev_vel_err[:] = 0.0
        controller._summary_done = False; controller._window_summary_done = False
        controller.mark_pos_errs = []; controller.mark_yaw_errs = []
        controller._startup_time = time.time(); controller._startup_cycles = 0
        print(f"[controller] New Target -> {new_target}")
    else:
        controller.t_since_target_change += dt

    # Startup logic
    if (time.time() - (controller._startup_time or 0)) < TUNABLES["startup_seconds"] or controller._startup_cycles < TUNABLES["startup_cycles"]:
        controller._startup_cycles += 1
        safe_vz = TUNABLES["hover_vz"]
        controller.prev_pos = curr_p.copy()
        controller.prev_vel_cmd = np.array([0.0, 0.0, safe_vz])
        return (0.0, 0.0, safe_vz, 0.0)

    # --- Position Loop ---
    pos_err = targ_p - curr_p
    pos_err_der = (pos_err - controller.prev_pos_err) / dt
    controller.prev_pos_err = pos_err.copy()
    integrate_mask = (np.abs(pos_err) > np.array([0.02, 0.02, 0.04])).astype(float)
    controller.pos_int += (pos_err * integrate_mask) * dt
    controller.pos_int = np.clip(controller.pos_int, -0.6, 0.6)

    vel_des = np.zeros(3)
    vel_des[:2] = (TUNABLES["outer_kp_xy"] * pos_err[:2] + TUNABLES["outer_kd_xy"] * pos_err_der[:2])
    vel_des[2] = (TUNABLES["outer_kp_z"] * pos_err[2] + 0.40 * pos_err_der[2])
    
    hnorm = np.linalg.norm(vel_des[:2])
    if hnorm > TUNABLES["vmax_xy"]: vel_des[:2] *= (TUNABLES["vmax_xy"] / hnorm)
    vel_des[2] = np.clip(vel_des[2], -TUNABLES["vmax_z"], TUNABLES["vmax_z"])

    # --- Velocity Loop ---
    measured_v_world = np.clip((curr_p - controller.prev_pos) / dt, -TUNABLES["measured_v_clip"], TUNABLES["measured_v_clip"])
    controller.prev_pos = curr_p.copy()
    controller.vel_filter = TUNABLES["alpha_v"] * measured_v_world + (1.0 - TUNABLES["alpha_v"]) * controller.vel_filter
    curr_v = controller.vel_filter.copy()
    
    vel_err = vel_des - curr_v
    vel_err_der = (vel_err - controller.prev_vel_err) / dt
    controller.prev_vel_err = vel_err.copy()
    controller.vel_int = np.clip(controller.vel_int + vel_err * dt, -TUNABLES["v_int_clip"], TUNABLES["v_int_clip"])
    
    vel_pid = np.zeros(3)
    vel_pid[:2] = (TUNABLES["inner_kp_xy"] * vel_err[:2] + TUNABLES["inner_kd_xy"] * vel_err_der[:2])
    vel_pid[2] = (TUNABLES["inner_kp_z"] * vel_err[2] + 0.08 * vel_err_der[2])
    
    vel_cmd_world = curr_v + vel_pid
    
    # Delta smoothing
    delta = vel_cmd_world - controller.prev_vel_cmd
    delta[:2] = np.clip(delta[:2], -TUNABLES["max_delta_vxy"], TUNABLES["max_delta_vxy"])
    delta[2] = np.clip(delta[2], -TUNABLES["max_delta_vz"], TUNABLES["max_delta_vz"])
    vel_cmd_world = controller.prev_vel_cmd + delta
    vel_cmd_world = (TUNABLES["alpha_cmd"] * vel_cmd_world + (1.0 - TUNABLES["alpha_cmd"]) * controller.prev_vel_cmd)
    controller.prev_vel_cmd = vel_cmd_world.copy()

    # Safety and Yaw
    if curr_p[2] < TUNABLES["min_safe_alt"]:
        vel_cmd_world[:2] = 0.0
        if vel_cmd_world[2] < TUNABLES["hover_vz"]: vel_cmd_world[2] = TUNABLES["hover_vz"]

    yaw_err = ((targ_yaw - current_yaw + np.pi) % (2.0 * np.pi)) - np.pi
    yaw_rate_cmd = float(np.clip(TUNABLES["yaw_kp"] * yaw_err, -1.0, 1.0))

    # Frame rotation
    c, s_r = np.cos(-current_yaw), np.sin(-current_yaw)
    R = np.array([[c, -s_r, 0.0], [s_r, c, 0.0], [0.0, 0.0, 1.0]])
    out_vel = R.dot(vel_cmd_world)

    # --- Phase 1: Debugging (0-10s) ---
    if controller.t_since_target_change <= 10.0:
        if time.time() - controller._last_print_time >= 0.05:
            controller._last_print_time = time.time()
            print(" === controller debug === ")
            print(f"t_since_target: {controller.t_since_target_change :.2f}s")
            print(f"pos_err: [{pos_err[0] :.3f}, {pos_err[1] :.3f}, {pos_err[2] :.3f}]")
            print(f"yaw_err: {yaw_err :.3f}")
            print(f"out_vel (body): [{out_vel[0] :.3f}, {out_vel[1] :.3f}, {out_vel[2] :.3f}]")
    elif not controller._summary_done:
        controller._summary_done = True
        inst_pe, inst_ye = np.linalg.norm(pos_err), abs(yaw_err)
        print("\n" + "#" * 80)
        print("### FINAL STATISTICAL SUMMARY (10s INSTANT) ###")
        print(f"Pos Error Instant: {inst_pe :.5f}m | Target < 0.01m | {'PASS' if inst_pe < 0.01 else 'FAIL'}")
        print(f"Yaw Error Instant: {inst_ye :.5f}rad | Target < 0.01 | {'PASS' if inst_ye < 0.01 else 'FAIL'}")
        print("#" * 80 + "\n")

    # --- Phase 2: Marking (10-20s) ---
    if 10.0 <= controller.t_since_target_change <= 20.0:
        controller.mark_pos_errs.append(np.linalg.norm(pos_err))
        controller.mark_yaw_errs.append(abs(yaw_err))
        if time.time() - controller._last_print_time >= 0.5:
            controller._last_print_time = time.time()
            print(f" ... COLLECTING MARKING DATA: {controller.t_since_target_change :.1f}s / 20.0s")
        if controller.t_since_target_change >= 19.99 and not controller._window_summary_done:
            controller._window_summary_done = True
            m_pe, s_pe = np.mean(controller.mark_pos_errs), np.std(controller.mark_pos_errs)
            m_ye, s_ye = np.mean(controller.mark_yaw_errs), np.std(controller.mark_yaw_errs)
            print("\n" + "=" * 80)
            print("### MARKING WINDOW SUMMARY (Averaged 10s-20s) ###")
            print(f"Pos Error Mean: {m_pe :.5f}m | Target < 0.01m | {'PASS' if m_pe < 0.01 else 'FAIL'}")
            print(f"Pos Error Std: {s_pe :.5f} | Target < 0.01 | {'PASS' if s_pe < 0.01 else 'FAIL'}")
            print(f"Yaw Error Mean: {m_ye :.5f} rad| Target < 0.01 | {'PASS' if m_ye < 0.01 else 'FAIL'}")
            print(f"Yaw Error Std: {s_ye :.5f} | Target < 0.001 | {'PASS' if s_ye < 0.001 else 'FAIL'}")
            print("=" * 80 + "\n")

    telemetry.update(controller, dt, np.linalg.norm(pos_err), yaw_err, out_vel, curr_v, target_pos)
    return (float(out_vel[0]), float(out_vel[1]), float(out_vel[2]), float(yaw_rate_cmd))

