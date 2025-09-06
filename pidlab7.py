import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
import sys, os

# --- Resource Path (for icons etc.) ---
def resource_path(relative_path):
    try:
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")
    return os.path.join(base_path, relative_path)

# --- Motor presets ---
# Values are illustrative; adjust to match your motor's datasheet if needed.
def get_motor_parameters(motor_type="custom", measurement_mode="SI"):
    # Conversion factors
    deg_to_rad = np.pi / 180.0
    rad_to_deg = 180.0 / np.pi
    kgm2_to_lbft2 = 23.73036  # 1 kg*m^2 = 23.73036 lb*ft^2
    Nm_to_lbft = 0.737562  # 1 Nm = 0.737562 lb*ft

    if motor_type == "NEO":
        params = {
            "motor_inertia_kgm2": 1.5e-6,
            "motor_resistance_Ohm": 0.02, # Typo fix: 0.6 is too high for a NEO
            "torque_constant_Nm_per_A": 0.027,
            "back_emf_constant_Vs_per_rad": 0.027,
            "plus_minus_voltage_limit": 12.0,
            "mechanical_viscous_damping": 0.01,
            "max_current_A": 80.0 # Illustrative value
        }
    elif motor_type == "Kraken":
        params = {
            "motor_inertia_kgm2": 6e-6,
            "motor_resistance_Ohm": 0.015, # Typo fix
            "torque_constant_Nm_per_A": 0.035,
            "back_emf_constant_Vs_per_rad": 0.035,
            "plus_minus_voltage_limit": 12.0,
            "mechanical_viscous_damping": 0.015,
            "max_current_A": 80.0 # Illustrative value
        }
    elif motor_type == "tiny brush": # Added specific parameters for "tiny brush"
        params = {
            "motor_inertia_kgm2": 5e-7,
            "motor_resistance_Ohm": 2.0,
            "torque_constant_Nm_per_A": 0.03,
            "back_emf_constant_Vs_per_rad": 0.03,
            "plus_minus_voltage_limit": 12.0,
            "mechanical_viscous_damping": 0.005,
            "max_current_A": 40.0 # Illustrative value
        }
    else: # This will now only be "Custom..."
        params = {
            "motor_inertia_kgm2": 5e-6,
            "motor_resistance_Ohm": 2.0,
            "torque_constant_Nm_per_A": 0.03,
            "back_emf_constant_Vs_per_rad": 0.03,
            "plus_minus_voltage_limit": 12.0,
            "mechanical_viscous_damping": 0.02,
            "max_current_A": 50.0 # Default custom value
        }

    if measurement_mode == "Imperial":
        # Convert all relevant parameters to Imperial units
        params["motor_inertia_kgm2"] *= kgm2_to_lbft2
        params["torque_constant_Nm_per_A"] *= Nm_to_lbft
        # Ke (V*s/rad) to (V*s/deg) by dividing by rad_to_deg (or multiplying by deg_to_rad)
        params["back_emf_constant_Vs_per_rad"] *= deg_to_rad
        # Mech damping (Nm/(rad/s)) to (lb*ft/(deg/s))
        params["mechanical_viscous_damping"] *= Nm_to_lbft / rad_to_deg
    return params


# --- Simulation (fixed physics) ---
def run_simulation(*args):
    loading_label.config(text="Loading simulation...")
    root.update_idletasks()

    ax1.cla()
    ax2.cla()
    ax3.cla()

    # Motor parameters
    motor_type_val = motor_type_var.get()
    measurement_mode_val = measurement_mode_var.get()
    deg_to_rad = np.pi / 180.0
    rad_to_deg = 180.0 / np.pi
    kgm2_to_lbft2 = 23.73036
    Nm_to_lbft = 0.737562
    Vs_per_rad_to_Vs_per_deg = 1 / rad_to_deg

    if motor_type_val == "Custom...":
        try:
            inertia = float(custom_inertia_entry.get() or 0.0)
            resistance = float(custom_resistance_entry.get() or 1.0)
            torque_const = float(custom_torque_entry.get() or 0.01)
            backemf_const = float(custom_backemf_entry.get() or torque_const)
            voltage_limit = float(custom_voltage_entry.get() or 12.0)
            mech_damping = float(custom_mech_damp_entry.get() or 0.01)
            max_current = float(custom_current_entry.get() or 50.0)
            if measurement_mode_val == "Imperial":
                inertia *= kgm2_to_lbft2
                torque_const *= Nm_to_lbft
                backemf_const *= Vs_per_rad_to_Vs_per_deg
                mech_damping *= Nm_to_lbft / rad_to_deg
            motor_params = {
                "motor_inertia_kgm2": inertia,
                "motor_resistance_Ohm": resistance,
                "torque_constant_Nm_per_A": torque_const,
                "back_emf_constant_Vs_per_rad": backemf_const,
                "plus_minus_voltage_limit": voltage_limit,
                "mechanical_viscous_damping": mech_damping,
                "max_current_A": max_current
            }
        except ValueError:
            loading_label.config(text="Invalid custom motor values")
            return
    else:
        motor_params = get_motor_parameters(motor_type_val, measurement_mode_val)

    # Gearbox gear ratio: interpreted as motor_rotations / output_rotations ("N:1")
    try:
        gear_ratio_val = float(gearbox_entry.get() or 1.0)
        if gear_ratio_val == 0:
            gear_ratio_val = 1.0
    except Exception:
        gear_ratio_val = 1.0

    # Extract motor constants (clear variable names)
    Jm = motor_params["motor_inertia_kgm2"]
    R = motor_params["motor_resistance_Ohm"]
    Kt_m = motor_params["torque_constant_Nm_per_A"]
    Ke_m = motor_params["back_emf_constant_Vs_per_rad"]
    Vlim = motor_params["plus_minus_voltage_limit"]
    mech_damping = motor_params.get("mechanical_viscous_damping", 0.01)
    max_current = motor_params.get("max_current_A", 50.0)

    # Derived output-side constants (reflecting gearbox)
    N = gear_ratio_val
    if R == 0:
        loading_label.config(text="Motor resistance must be non-zero")
        return

    # PID values
    try:
        kp_val = float(kp_entry.get() or 0.0)
        ki_val = float(ki_entry.get() or 0.0)
        kd_val = float(kd_entry.get() or 0.0)
    except ValueError:
        loading_label.config(text="Invalid PID values")
        return

    mode_val = mode_var.get()           # 'speed' or 'position'
    system_mode = system_mode_var.get() # 'Free PID Mode' or 'Vertical Load-Bearing PIDF Mode'
    timestep = 0.001
    try:
        acceptable_error_val = float(acceptable_error_entry.get() or 1.0)
    except ValueError:
        acceptable_error_val = 1.0

    # Feedforward & load params
    try:
        kf_val = float(ff_entry.get() or 0.0) # This is now ONLY the kF velocity feedforward
        if system_mode == "Vertical Load-Bearing PIDF Mode":
            if measurement_mode_val == "Imperial":
                # Mass in pounds, radius in feet+inches
                m_lb = float(mass_entry.get() or 0.0)
                ft = float(radius_feet_entry.get() or 0.0)
                inch = float(radius_inches_entry.get() or 0.0)
                r_ft = ft + (inch / 12.0)
                # Convert to SI for calculations
                m_val = m_lb * 0.45359237 # lb to kg
                r_val = r_ft * 0.3048     # ft to m
            else:
                m_val = float(mass_entry.get() or 0.0)
                r_val = float(radius_entry.get() or 0.0)
        else:
            m_val = 0.0
            r_val = 0.0
    except ValueError:
        loading_label.config(text="Invalid FF/load values")
        return

    g_const = 9.81  # m/s^2
    # Imperial gravity for ft/s^2
    g_const_imperial = 32.174  # ft/s^2
    
    # Motor inversion
    inverted = invert_var.get()

    # Targets: (value, time)
    targets_val = []
    for val_entry, time_entry in target_entries:
        try:
            val = float(val_entry.get())
            t = float(time_entry.get())
            targets_val.append((val, t))
        except:
            pass
    targets_val.sort(key=lambda x: x[1])
    if not targets_val:
        loading_label.config(text="")
        return

    # Simulation state
    pos = 0.0           # radians or degrees
    omega = 0.0         # rad/s or deg/s
    error_accum = 0.0
    prev_error = 0.0
    current_target_index = 0
    target_value = targets_val[current_target_index][0]

    time_log = []
    current_log = []
    control_log = []
    motor_current_log = []

    sim_duration = max(t[1] for t in targets_val) + 4.0
    steps = max(1, int(sim_duration / timestep))

    # Precompute electrical scaling constants
    a_out = (Kt_m * N) / R                  # Nm/lbft per Volt (at output)
    b_elec_out = (Kt_m * Ke_m * (N ** 2)) / R  # Nm/lbft per (rad/s or deg/s) (electrical damping at output)

    for step in range(steps):
        t = step * timestep
        # target switching
        if current_target_index + 1 < len(targets_val) and t >= targets_val[current_target_index + 1][1]:
            current_target_index += 1
            target_value = targets_val[current_target_index][0]

        # Gravity torque (correct sign): tau_g = - dU/dtheta = - m*g*r*cos(theta)
        if system_mode == "Vertical Load-Bearing PIDF Mode" and m_val != 0.0 and r_val != 0.0:
            if measurement_mode_val == "Imperial":
                tau_g = - m_val * g_const_imperial * r_val * np.cos(np.deg2rad(pos))
            else:
                tau_g = - m_val * g_const * r_val * np.cos(pos)
            u_ff_gravity = tau_g / a_out if a_out != 0.0 else 0.0
        else:
            tau_g = 0.0
            u_ff_gravity = 0.0

        # load inertia (point mass at radius r): J_load = m * r^2
        if system_mode == "Vertical Load-Bearing PIDF Mode" and m_val != 0.0 and r_val != 0.0:
            J_load = m_val * (r_val ** 2)
        else:
            J_load = 0.0

        # total output-side inertia
        J_total = J_load + Jm * (N ** 2)
        # guard tiny J_total to avoid numerical explosion
        if J_total < 1e-9:
            J_total = 1e-9

        # PID error (speed uses omega, position uses pos)
        measured = omega if mode_val == "speed" else pos
        error = target_value - measured

        # PID terms (basic)
        u_p = kp_val * error

        # Integrator with soft anti-windup: cap accumulated integral
        error_accum += error * timestep
        if ki_val != 0.0:
            # maximum integrator value that could produce full voltage
            I_max = abs(Vlim / max(ki_val, 1e-12))
            error_accum = max(min(error_accum, I_max), -I_max)
        u_i = ki_val * error_accum

        # Derivative
        u_d = kd_val * ((error - prev_error) / timestep) if timestep > 0 else 0.0
        prev_error = error
        
        # Velocity Feedforward term
        u_ff_vel = kf_val * target_value
        
        # Total voltage command before clamping and limiting
        u_total_pre = u_p + u_i + u_d + u_ff_vel + u_ff_gravity

        # Apply motor inversion if checkbox is checked
        if inverted:
            u_total_pre *= -1
            
        # Clamp to voltage limits (simple saturation)
        u_total = max(min(u_total_pre, Vlim), -Vlim)

        # Current Limiting (more accurate implementation)
        # Calculate the maximum allowed voltage based on the current limit and back-EMF
        # V_limit = R * I_limit + Ke * omega
        max_voltage_current_limit = R * max_current + Ke_m * abs(omega)
        u_total = max(min(u_total, max_voltage_current_limit), -max_voltage_current_limit)

        # Motor torques at output
        tau_motor_fromV = a_out * u_total
        tau_motor_from_omega = b_elec_out * omega

        # mechanical viscous damping
        tau_mech_damping = mech_damping * omega

        # dynamics: J_total * omega_dot = tau_motor_fromV - tau_motor_from_omega + tau_g - tau_mech_damping
        omega_dot = (tau_motor_fromV - tau_motor_from_omega + tau_g - tau_mech_damping) / J_total

        # integrate state (semi-implicit integration helps stability)
        # semi-implicit Euler: omega_{n+1} = omega_n + omega_dot * dt; pos_{n+1} = pos_n + omega_{n+1} * dt
        omega = omega + omega_dot * timestep
        pos = pos + omega * timestep

        # calculate motor current for logging after all voltages are set
        I_motor_calc = (u_total - Ke_m * omega) / R

        # record
        time_log.append(t)
        # Convert output for graphing if in Imperial mode
        if measurement_mode_val == "Imperial":
            current_log.append(omega if mode_val == "speed" else pos)
        else:
            current_log.append(omega if mode_val == "speed" else pos)
        control_log.append(u_total)
        motor_current_log.append(I_motor_calc)


    # Plot results
    if measurement_mode_val == "Imperial":
        label_output = 'Motor Speed (deg/s)' if mode_val == 'speed' else 'Position (deg)'
    else:
        label_output = 'Motor Speed (rad/s)' if mode_val == 'speed' else 'Position (rad)'
    ax1.plot(time_log, current_log, color='#1f77b4', linewidth=1.5, label=label_output)

    # Draw horizontal target lines spanning their intervals and acceptable error shading
    for i, (value, start_time) in enumerate(targets_val):
        end_time = targets_val[i+1][1] if i+1 < len(targets_val) else sim_duration
        ax1.hlines(y=value, xmin=start_time, xmax=end_time, colors='r', linestyles='--', alpha=0.8)
        # acceptable band shading
        xs = [start_time, end_time]
        ax1.fill_betweenx([value - acceptable_error_val, value + acceptable_error_val],
                            x1=start_time, x2=end_time, color='#00ff00', alpha=0.08)
        ax1.text((start_time + end_time) / 2, value + (1.5 * acceptable_error_val),
                  f'Target {i+1}: {value}', color='r', ha='center')

        # find first time index where all remaining points stay within the threshold
        start_idx = int(start_time / timestep)
        end_idx = int(end_time / timestep)
        # clip indices to available arrays
        start_idx = max(0, min(start_idx, len(current_log)))
        end_idx = max(0, min(end_idx, len(current_log)))
        if end_idx - start_idx >= 2:
            arr = np.array(current_log[start_idx:end_idx])
            within = np.abs(arr - value) <= acceptable_error_val
            ok_idx = None
            for j in range(len(within)):
                if np.all(within[j:]):
                    ok_idx = start_idx + j
                    break
            if ok_idx is not None and (end_idx - ok_idx) > 1:
                ok_time = time_log[ok_idx]
                ax1.axvline(x=ok_time, color='#ff00ff', linestyle=':', alpha=0.9)
                ymin, ymax = ax1.get_ylim()
                ax1.text(ok_time, ymin,
                          f'Ok at {ok_time - start_time:.3f}s', color='#ff00ff', va='bottom', ha='left', rotation=0)

    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel(label_output)
    ax1.grid(True)

    ax2.plot(time_log, control_log, color='#2ca02c', alpha=0.75, label='Control Voltage (V)')
    ax2.set_ylabel('Control Voltage (V)', rotation=270, labelpad=15, va='bottom')
    ax2.yaxis.set_label_position("right")
    ax2.yaxis.tick_right()
    ax2.grid(False)

    ax3.plot(time_log, motor_current_log, color='#ff7f0e', alpha=0.75, label='Motor Current (A)')
    ax3.set_ylabel('Motor Current (A)', rotation=270, labelpad=15, va='bottom')
    ax3.yaxis.set_label_position("right")
    ax3.yaxis.tick_right()
    ax3.grid(False)

    lines_1, labels_1 = ax1.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    lines_3, labels_3 = ax3.get_legend_handles_labels()
    ax1.legend(lines_1 + lines_2 + lines_3, labels_1 + labels_2 + labels_3, loc='upper right')

    canvas.draw()
    loading_label.config(text="")  # remove loading message

# ---------------- GUI setup ----------------
root = tk.Tk()
root.title("PIDlab 7")

def on_closing():
    """This function is called when the user clicks the 'X' button."""
    print("Close the terminal to exit this program.")
    root.destroy()

try:
    root.iconbitmap(resource_path("pidlab7.ico"))
except Exception:
    pass

# Modern font and style
FONT_MAIN = ("Verdana", 11)
FONT_TITLE = ("Verdana", 15, "bold")
FONT_LABEL = ("Verdana", 10)
BG_COLOR = "#e2f1ff"
ACCENT_COLOR = "#0055ff"
FRAME_PAD = 12

root.configure(bg=BG_COLOR)

# Main frames
frame_left = tk.Frame(root, padx=FRAME_PAD, pady=FRAME_PAD, bg=BG_COLOR)
frame_left.pack(side=tk.LEFT, fill=tk.Y)
frame_right = tk.Frame(root, bg=BG_COLOR)
frame_right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

# Title
title_label = tk.Label(frame_left, text="PIDlab 7", font=FONT_TITLE, fg=ACCENT_COLOR, bg=BG_COLOR)
title_label.pack(pady=(0, 10))

# Measurement Mode Dropdown
measurement_mode_var = tk.StringVar(value="SI")
measurement_mode_label = tk.Label(frame_left, text="Measurement Mode", font=FONT_LABEL, bg=BG_COLOR)
measurement_mode_label.pack(anchor='w', pady=(0,2))
measurement_mode_dropdown = ttk.Combobox(frame_left, textvariable=measurement_mode_var,
                                         values=["SI", "Imperial"], state="readonly", font=FONT_MAIN)
measurement_mode_dropdown.pack(pady=(0, 10), fill=tk.X)

# System Mode Dropdown
system_mode_var = tk.StringVar(value="Free PID Mode")
system_mode_label = tk.Label(frame_left, text="System Mode", font=FONT_LABEL, bg=BG_COLOR)
system_mode_label.pack(anchor='w', pady=(0,2))
system_mode_dropdown = ttk.Combobox(frame_left, textvariable=system_mode_var,
                                     values=["Free PID Mode", "Vertical Load-Bearing PIDF Mode"], state="readonly", font=FONT_MAIN)
system_mode_dropdown.pack(pady=(0, 10), fill=tk.X)

# show/hide ff_frame
def update_system_mode(*args):
    if system_mode_var.get() == "Vertical Load-Bearing PIDF Mode":
        ff_frame.pack(fill=tk.X, pady=(8,0))
    else:
        ff_frame.pack_forget()
system_mode_var.trace_add("write", update_system_mode)

# Motor type
motor_type_label = tk.Label(frame_left, text="Motor Type", font=FONT_LABEL, bg=BG_COLOR)
motor_type_label.pack(anchor='w', pady=(0,2))
motor_type_var = tk.StringVar(value="tiny brush")
motor_dropdown = ttk.Combobox(frame_left, textvariable=motor_type_var,
                              values=["NEO", "Kraken", "tiny brush", "Custom..."], state="readonly", font=FONT_MAIN)
motor_dropdown.pack(pady=(0, 10), fill=tk.X)

# Custom motor fields (appear only when chosen)
custom_frame = tk.LabelFrame(frame_left, text="Custom Motor Parameters", font=FONT_LABEL, padx=8, pady=8, bg=BG_COLOR, fg=ACCENT_COLOR)
custom_frame.configure(labelanchor='n')
tk.Label(custom_frame, text="Inertia (kg·m²)", font=FONT_LABEL, bg=BG_COLOR).pack(anchor='w'); custom_inertia_entry = tk.Entry(custom_frame, font=FONT_MAIN, width=12); custom_inertia_entry.pack(fill=tk.X, pady=2)
tk.Label(custom_frame, text="Resistance (Ω)", font=FONT_LABEL, bg=BG_COLOR).pack(anchor='w'); custom_resistance_entry = tk.Entry(custom_frame, font=FONT_MAIN, width=12); custom_resistance_entry.pack(fill=tk.X, pady=2)
tk.Label(custom_frame, text="Torque const (Nm/A)", font=FONT_LABEL, bg=BG_COLOR).pack(anchor='w'); custom_torque_entry = tk.Entry(custom_frame, font=FONT_MAIN, width=12); custom_torque_entry.pack(fill=tk.X, pady=2)
tk.Label(custom_frame, text="Back-emf const (V·s/rad)", font=FONT_LABEL, bg=BG_COLOR).pack(anchor='w'); custom_backemf_entry = tk.Entry(custom_frame, font=FONT_MAIN, width=12); custom_backemf_entry.pack(fill=tk.X, pady=2)
tk.Label(custom_frame, text="Voltage limit (V)", font=FONT_LABEL, bg=BG_COLOR).pack(anchor='w'); custom_voltage_entry = tk.Entry(custom_frame, font=FONT_MAIN, width=12); custom_voltage_entry.pack(fill=tk.X, pady=2)
tk.Label(custom_frame, text="Mech viscous damping", font=FONT_LABEL, bg=BG_COLOR).pack(anchor='w'); custom_mech_damp_entry = tk.Entry(custom_frame, font=FONT_MAIN, width=12); custom_mech_damp_entry.pack(fill=tk.X, pady=2)
tk.Label(custom_frame, text="Current limit (A)", font=FONT_LABEL, bg=BG_COLOR).pack(anchor='w'); custom_current_entry = tk.Entry(custom_frame, font=FONT_MAIN, width=12); custom_current_entry.insert(0, "50.0"); custom_current_entry.pack(fill=tk.X, pady=2)

def update_custom_fields(*args):
    if motor_type_var.get() == "Custom...":
        custom_frame.pack(fill=tk.X, pady=(8,0))
    else:
        custom_frame.pack_forget()
motor_type_var.trace_add("write", update_custom_fields)

# Gearbox gear ratio field
gearbox_label = tk.Label(frame_left, text="Gearbox gear ratio (motor turns : output turns)", font=FONT_LABEL, bg=BG_COLOR)
gearbox_label.pack(anchor='w', pady=(0,2))
gearbox_entry = tk.Entry(frame_left, font=FONT_MAIN)
gearbox_entry.insert(0, "1")
gearbox_entry.pack(pady=(0, 10), fill=tk.X)

# PID entries
pid_frame = tk.LabelFrame(frame_left, text="PID Parameters", font=FONT_LABEL, padx=8, pady=8, bg=BG_COLOR, fg=ACCENT_COLOR)
pid_frame.configure(labelanchor='n')
tk.Label(pid_frame, text="kp", font=FONT_LABEL, bg=BG_COLOR).pack(anchor='w'); kp_entry = tk.Entry(pid_frame, font=FONT_MAIN); kp_entry.insert(0, "0.0015"); kp_entry.pack(fill=tk.X, pady=2)
tk.Label(pid_frame, text="ki", font=FONT_LABEL, bg=BG_COLOR).pack(anchor='w'); ki_entry = tk.Entry(pid_frame, font=FONT_MAIN); ki_entry.insert(0, "0.08"); ki_entry.pack(fill=tk.X, pady=2)
tk.Label(pid_frame, text="kd", font=FONT_LABEL, bg=BG_COLOR).pack(anchor='w'); kd_entry = tk.Entry(pid_frame, font=FONT_MAIN); kd_entry.insert(0, "0.000005"); kd_entry.pack(fill=tk.X, pady=2)
pid_frame.pack(fill=tk.X, pady=(8,0))

# Mode (speed/position)
mode_label = tk.Label(frame_left, text="Control Mode", font=FONT_LABEL, bg=BG_COLOR)
mode_label.pack(anchor='w', pady=(0,2))
mode_var = tk.StringVar(value="speed")
mode_dropdown = ttk.Combobox(frame_left, textvariable=mode_var, values=["speed", "position"], state="readonly", font=FONT_MAIN)
mode_dropdown.pack(pady=(0, 10), fill=tk.X)

# Feedforward & load params (only for PIDF mode)
ff_frame = tk.LabelFrame(frame_left, text="Feedforward & Load Params", font=FONT_LABEL, padx=8, pady=8, bg=BG_COLOR, fg=ACCENT_COLOR)
ff_frame.configure(labelanchor='n')
tk.Label(ff_frame, text="Velocity Feedforward (kF)", font=FONT_LABEL, bg=BG_COLOR).pack(anchor='w'); ff_entry = tk.Entry(ff_frame, font=FONT_MAIN); ff_entry.insert(0, "0.0"); ff_entry.pack(fill=tk.X, pady=2)
mass_label = tk.Label(ff_frame, text="Mass (kg)", font=FONT_LABEL, bg=BG_COLOR)
mass_label.pack(anchor='w')
mass_entry = tk.Entry(ff_frame, font=FONT_MAIN)
mass_entry.insert(0, "1.0")
mass_entry.pack(fill=tk.X, pady=2)
radius_label = tk.Label(ff_frame, text="Radius (m)", font=FONT_LABEL, bg=BG_COLOR)
radius_label.pack(anchor='w')
radius_entry = tk.Entry(ff_frame, font=FONT_MAIN)
radius_entry.insert(0, "1.0")
radius_entry.pack(fill=tk.X, pady=2)
radius_feet_entry = tk.Entry(ff_frame, font=FONT_MAIN, width=6)
radius_inches_entry = tk.Entry(ff_frame, font=FONT_MAIN, width=6)
radius_feet_entry.insert(0, "0")
radius_inches_entry.insert(0, "0")

# Show/hide imperial fields and update labels
def update_measurement_mode(*args):
    mode = measurement_mode_var.get()
    if mode == "Imperial":
        mass_label.config(text="Mass (lb)")
        radius_label.config(text="Radius (ft + in)")
        radius_entry.pack_forget()
        radius_feet_entry.pack(fill=tk.X, pady=2)
        radius_inches_entry.pack(fill=tk.X, pady=2)
    else:
        mass_label.config(text="Mass (kg)")
        radius_label.config(text="Radius (m)")
        radius_feet_entry.pack_forget()
        radius_inches_entry.pack_forget()
        radius_entry.pack(fill=tk.X, pady=2)
    run_simulation()
measurement_mode_var.trace_add("write", update_measurement_mode)

# Motor inversion checkbox
invert_var = tk.BooleanVar()
invert_checkbutton = tk.Checkbutton(frame_left, text="Invert Motor Output", variable=invert_var, command=run_simulation, font=FONT_LABEL, bg=BG_COLOR)
invert_checkbutton.pack(anchor='w', pady=(8,0))

# Targets & acceptable error
targets_frame = tk.LabelFrame(frame_left, text="Targets & Acceptable Error", font=FONT_LABEL, padx=8, pady=8, bg=BG_COLOR, fg=ACCENT_COLOR)
targets_frame.configure(labelanchor='n')
tk.Label(targets_frame, text="Targets (value, time)", font=FONT_LABEL, bg=BG_COLOR).pack(anchor='w')
target_entries = []

def add_target():
    t_frame = tk.Frame(targets_frame, bg=BG_COLOR)
    t_frame.pack(fill=tk.X, pady=2)
    idx = len(target_entries) + 1
    tk.Label(t_frame, text=f"Target {idx}", font=FONT_LABEL, bg=BG_COLOR).pack(side=tk.LEFT)
    val_entry = tk.Entry(t_frame, font=FONT_MAIN, width=8); val_entry.pack(side=tk.LEFT, padx=4)
    time_entry = tk.Entry(t_frame, font=FONT_MAIN, width=6); time_entry.pack(side=tk.LEFT, padx=4)
    val_entry.insert(0, "0"); time_entry.insert(0, "0")
    val_entry.bind("<KeyRelease>", run_simulation); time_entry.bind("<KeyRelease>", run_simulation)
    target_entries.append((val_entry, time_entry))

def remove_target():
    if target_entries:
        e = target_entries.pop()
        e[0].master.destroy()

btn_frame = tk.Frame(targets_frame, bg=BG_COLOR)
btn_frame.pack(fill=tk.X, pady=(4,0))
tk.Button(btn_frame, text="Add Target", command=add_target, font=FONT_LABEL, bg="#009737", fg="white", relief=tk.FLAT).pack(side=tk.LEFT, padx=2)
tk.Button(btn_frame, text="Remove Target", command=remove_target, font=FONT_LABEL, bg="#d9534f", fg="white", relief=tk.FLAT).pack(side=tk.LEFT, padx=2)

tk.Label(targets_frame, text="Acceptable Error (+/-)", font=FONT_LABEL, bg=BG_COLOR).pack(anchor='w', pady=(6,0))
acceptable_error_entry = tk.Entry(targets_frame, font=FONT_MAIN); acceptable_error_entry.insert(0, "1"); acceptable_error_entry.pack(fill=tk.X, pady=2)

targets_frame.pack(fill=tk.X, pady=(8,0))

# Loading indicator
loading_label = tk.Label(frame_left, text="", fg=ACCENT_COLOR, font=FONT_LABEL, bg=BG_COLOR)
loading_label.pack(pady=(12,0))

# Matplotlib figure
plt.style.use("seaborn-v0_8-bright")
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(7, 6), dpi=100, sharex=True)
fig.subplots_adjust(hspace=0.2)
canvas = FigureCanvasTkAgg(fig, master=frame_right)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=FRAME_PAD, pady=FRAME_PAD)

# Bind relevant widgets to run_simulation (live update)
widgets_to_bind = [kp_entry, ki_entry, kd_entry, acceptable_error_entry, motor_dropdown, mode_dropdown,
                   custom_inertia_entry, custom_resistance_entry, custom_torque_entry, custom_backemf_entry, custom_voltage_entry,
                   custom_current_entry, system_mode_dropdown, ff_entry, mass_entry, radius_entry, gearbox_entry, measurement_mode_dropdown,
                   radius_feet_entry, radius_inches_entry]
for w in widgets_to_bind:
    w.bind("<KeyRelease>", run_simulation)
system_mode_dropdown.bind("<<ComboboxSelected>>", run_simulation)
motor_dropdown.bind("<<ComboboxSelected>>", run_simulation)
mode_dropdown.bind("<<ComboboxSelected>>", run_simulation)
measurement_mode_dropdown.bind("<<ComboboxSelected>>", run_simulation)
root.protocol("WM_DELETE_WINDOW", on_closing)

# initial targets
for _ in range(3):
    add_target()

run_simulation()
root.mainloop()