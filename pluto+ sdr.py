#!/usr/bin/env python3
# ============================================================
# PlutoSDR MODCOD Real-time System (ONE FILE)
# ONE-SHOT MODE (CYCLIC TX di Pluto):
# - Start -> TX sekali (Pluto looping via tx_cyclic_buffer=True)
# - RX stop saat LEN+CRC valid
#
# UPGRADE untuk 500+ byte:
# - RX buffer size AUTO mengikuti panjang 1 frame TX (biar gak kepotong)
# - Saran: Samples/Symbol (SPS) turunin ke 10-20 untuk payload besar
#
# DISPLAY Received Files:
# - Listbox + Save/Open/Clear + store_received_file()
#
# Frame:
#   PREAMBLE (64b) + LEN(u32 bytes) + CRC32(u32) + PAYLOAD(FEC)
#
# Install:
#   pip install numpy matplotlib pyadi-iio
# Optional (WAV):
#   pip install scipy
#
# Loopback test:
#   TX1 -> RX1 via SMA + attenuator 20-30 dB
# ============================================================

import os, sys, time, threading, tempfile, platform, subprocess
import numpy as np
import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog
from collections import deque

# ---- Windows DLL helper (IIO Oscilloscope bin path) ----
def _ensure_libiio_on_windows():
    if not sys.platform.startswith("win"):
        return
    candidates = [
        r"C:\Program Files\IIO Oscilloscope\bin",
        r"C:\Program Files\Analog Devices\IIO Oscilloscope\bin",
        r"C:\Program Files (x86)\IIO Oscilloscope\bin",
        r"C:\Program Files (x86)\Analog Devices\IIO Oscilloscope\bin",
    ]
    for p in candidates:
        dll = os.path.join(p, "libiio.dll")
        if os.path.isfile(dll):
            try:
                os.add_dll_directory(p)
            except Exception:
                pass
            os.environ["PATH"] = p + ";" + os.environ.get("PATH", "")
            return

_ensure_libiio_on_windows()

try:
    import adi
except ImportError:
    print("Install pyadi-iio: pip install pyadi-iio")
    raise

try:
    from scipy.io import wavfile
    SCIPY_OK = True
except Exception:
    SCIPY_OK = False


# ============================================================
# FEC (SIMPLE)
# ============================================================
class ConvolutionalEncoder:
    def __init__(self, rate=0.5):
        self.rate = rate

    def encode(self, bits):
        encoded = []
        bits = list(map(int, bits))
        if abs(self.rate - 0.5) < 1e-9:  # 1/2
            for bit in bits:
                encoded.extend([bit, bit])
        elif abs(self.rate - 0.33) < 1e-2:  # 1/3
            for bit in bits:
                encoded.extend([bit, bit, bit])
        elif abs(self.rate - 0.67) < 1e-2:  # 2/3
            for i in range(0, len(bits), 2):
                if i + 1 < len(bits):
                    encoded.extend([bits[i], bits[i + 1], bits[i] ^ bits[i + 1]])
                else:
                    encoded.extend([bits[i], 0, bits[i]])
        elif abs(self.rate - 0.75) < 1e-9:  # 3/4
            for i in range(0, len(bits), 3):
                if i + 2 < len(bits):
                    encoded.extend([bits[i], bits[i + 1], bits[i + 2], bits[i] ^ bits[i + 1]])
                else:
                    remaining = bits[i:]
                    encoded.extend(list(remaining) + [0] * (4 - len(remaining)))
        return np.array(encoded, dtype=np.int8)


class ViterbiDecoder:
    def __init__(self, rate=0.5):
        self.rate = rate

    def decode(self, received_bits):
        rb = list(map(int, received_bits))
        if len(rb) < 2:
            return np.array([], dtype=np.int8)

        decoded = []
        if abs(self.rate - 0.5) < 1e-9:  # 1/2
            for i in range(0, len(rb) - 1, 2):
                decoded.append(1 if (rb[i] + rb[i + 1]) >= 1 else 0)
        elif abs(self.rate - 0.33) < 1e-2:  # 1/3
            for i in range(0, len(rb) - 2, 3):
                decoded.append(1 if (rb[i] + rb[i + 1] + rb[i + 2]) >= 2 else 0)
        elif abs(self.rate - 0.67) < 1e-2:  # 2/3
            for i in range(0, len(rb) - 2, 3):
                decoded.extend([rb[i], rb[i + 1]])
        elif abs(self.rate - 0.75) < 1e-9:  # 3/4
            for i in range(0, len(rb) - 3, 4):
                decoded.extend([rb[i], rb[i + 1], rb[i + 2]])
        return np.array(decoded, dtype=np.int8)


# ============================================================
# GUI + ONE-SHOT TX/RX (CYCLIC TX di Pluto)
# ============================================================
class PlutoSDRMODCOD:
    def __init__(self, root):
        self.root = root
        self.root.title("PlutoSDR MODCOD Real-time System (ONE-SHOT / CYCLIC TX)")
        self.root.geometry("1600x950")
        self.root.configure(bg="#2b2b2b")

        # Core
        self.encoder = ConvolutionalEncoder()
        self.decoder = ViterbiDecoder()
        self.sdr = None
        self.is_running = False

        # Counters
        self.rx_count = 0
        self.decode_count = 0
        self.tx_count = 0

        # MODCOD
        self.modulation = "BPSK"
        self.code_rate = 0.5

        # Real-time monitoring
        self.snr_history = deque(maxlen=100)

        # File transmission + display
        self.file_data = None
        self.file_name = None
        self.data_type = "text"
        self.received_files = []

        self.create_widgets()
        self.setup_realtime_graphs()

        self.log("ONE-SHOT (CYCLIC): Start -> TX sekali (Pluto loop) -> RX stop saat LEN+CRC valid", "info")
        self.log("Untuk 500+ byte: turunin Samples/Symbol (misal 10) biar frame gak kepanjangan.", "warning")

    # ----------------- Helpers: CRC / bit ops -----------------
    def _fec_factor(self):
        if abs(self.code_rate - 0.5) < 1e-9:
            return 2.0
        if abs(self.code_rate - 0.33) < 1e-2:
            return 3.0
        if abs(self.code_rate - 0.67) < 1e-2:
            return 1.5
        if abs(self.code_rate - 0.75) < 1e-9:
            return 4.0 / 3.0
        return 2.0

    def _bytes_to_bits(self, data_bytes: bytes):
        return np.array([((b >> i) & 1) for b in data_bytes for i in range(7, -1, -1)], dtype=np.int8)

    def _bits_to_bytes(self, bits: np.ndarray):
        out = bytearray()
        n = len(bits) - (len(bits) % 8)
        for i in range(0, n, 8):
            byte = 0
            for j in range(8):
                byte = (byte << 1) | int(bits[i + j])
            out.append(byte)
        return bytes(out)

    def _u32_to_bits(self, x: int):
        return np.array([(x >> (31 - i)) & 1 for i in range(32)], dtype=np.int8)

    def _bits_to_u32(self, b32: np.ndarray):
        x = 0
        for i in range(32):
            x = (x << 1) | int(b32[i])
        return x

    def _crc32_u32(self, data: bytes):
        import zlib
        return zlib.crc32(data) & 0xFFFFFFFF

    # ----------------- UI utils -----------------
    def _ui(self, fn, *args, **kwargs):
        try:
            self.root.after(0, lambda: fn(*args, **kwargs))
        except Exception:
            pass

    def log(self, message, type="info"):
        def _do():
            self.console.configure(state="normal")
            colors = {"info": "#00ff00", "success": "#4CAF50", "error": "#f44336", "warning": "#ff9800"}
            self.console.insert("end", f"[{time.strftime('%H:%M:%S')}] {message}\n", type)
            self.console.tag_config(type, foreground=colors.get(type, "#00ff00"))
            self.console.see("end")
            self.console.configure(state="disabled")
        self._ui(_do)

    def add_message(self, text):
        def _do():
            self.messages_list.configure(state="normal")
            self.messages_list.insert("1.0", f"[{time.strftime('%H:%M:%S')}] {text}\n")
            self.messages_list.configure(state="disabled")
        self._ui(_do)

    # ✅ FIX: update_stats ada di file ini (biar gak AttributeError)
    def update_stats(self, snr=0.0, rxreads=0, rxbuf=0, frame=0):
        def _do():
            self.total_rx_label.config(text=str(self.rx_count))
            self.decoded_label.config(text=str(self.decode_count))
            rate = (self.decode_count / self.rx_count * 100.0) if self.rx_count > 0 else 0.0
            self.success_rate_label.config(text=f"{rate:.1f}%")
            self.snr_label.config(text=f"{snr:.1f}")
            self.txmode_label.config(text="CYCLIC")
            self.rxreads_label.config(text=str(rxreads))
            self.rxbuf_label.config(text=str(rxbuf))
            self.frame_label.config(text=str(frame))
        self._ui(_do)

    # ----------------- UI build -----------------
    def create_widgets(self):
        header = tk.Frame(self.root, bg="#3d3d3d", height=70)
        header.pack(fill="x", padx=10, pady=10)

        tk.Label(header, text="📡 PlutoSDR MODCOD (ONE-SHOT / CYCLIC)", font=("Arial", 20, "bold"),
                 bg="#3d3d3d", fg="white").pack(pady=5)
        tk.Label(header, text="TX sekali (Pluto loop) -> RX stop saat LEN+CRC match",
                 font=("Arial", 11), bg="#3d3d3d", fg="#aaaaaa").pack()

        main_container = tk.Frame(self.root, bg="#2b2b2b")
        main_container.pack(fill="both", expand=True, padx=10, pady=5)

        # Left panel with scroll
        left_container = tk.Frame(main_container, bg="#3d3d3d", width=380)
        left_container.pack(side="left", fill="both", padx=5, pady=5)

        left_canvas = tk.Canvas(left_container, bg="#3d3d3d", width=380, highlightthickness=0)
        left_scrollbar = tk.Scrollbar(left_container, orient="vertical", command=left_canvas.yview)
        left_panel = tk.Frame(left_canvas, bg="#3d3d3d")

        left_panel.bind("<Configure>", lambda e: left_canvas.configure(scrollregion=left_canvas.bbox("all")))
        left_canvas.create_window((0, 0), window=left_panel, anchor="nw")
        left_canvas.configure(yscrollcommand=left_scrollbar.set)

        left_canvas.pack(side="left", fill="both", expand=True)
        left_scrollbar.pack(side="right", fill="y")

        def _on_mousewheel(event):
            left_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        left_canvas.bind_all("<MouseWheel>", _on_mousewheel)

        tk.Label(left_panel, text="⚙️ Configuration", font=("Arial", 14, "bold"),
                 bg="#3d3d3d", fg="white").pack(pady=10)

        # MODCOD section
        modcod_frame = tk.Frame(left_panel, bg="#4d4d4d")
        modcod_frame.pack(pady=10, padx=20, fill="x")

        tk.Label(modcod_frame, text="🎯 MODCOD Settings", bg="#4d4d4d", fg="white",
                 font=("Arial", 11, "bold")).pack(pady=5)

        mod_frame = tk.Frame(modcod_frame, bg="#4d4d4d")
        mod_frame.pack(pady=5, fill="x", padx=10)
        tk.Label(mod_frame, text="Modulation:", bg="#4d4d4d", fg="white",
                 font=("Arial", 9, "bold")).pack(anchor="w")
        self.mod_var = tk.StringVar(value="BPSK")
        mod_combo = ttk.Combobox(mod_frame, textvariable=self.mod_var, values=["BPSK"], state="readonly", width=30)
        mod_combo.pack(pady=3)
        mod_combo.bind("<<ComboboxSelected>>", self.on_modcod_change)

        rate_frame = tk.Frame(modcod_frame, bg="#4d4d4d")
        rate_frame.pack(pady=5, fill="x", padx=10)
        tk.Label(rate_frame, text="Code Rate:", bg="#4d4d4d", fg="white",
                 font=("Arial", 9, "bold")).pack(anchor="w")
        self.rate_var = tk.StringVar(value="1/2")
        rate_combo = ttk.Combobox(rate_frame, textvariable=self.rate_var,
                                  values=["1/2", "1/3", "2/3", "3/4"], state="readonly", width=30)
        rate_combo.pack(pady=3)
        rate_combo.bind("<<ComboboxSelected>>", self.on_modcod_change)

        self.modcod_info = tk.Label(modcod_frame, text="MODCOD: BPSK 1/2", bg="#5d5d5d",
                                    fg="#4CAF50", font=("Arial", 10, "bold"), pady=5)
        self.modcod_info.pack(pady=5, fill="x", padx=10)

        # Data Type section
        datatype_frame = tk.Frame(left_panel, bg="#4d4d4d")
        datatype_frame.pack(pady=10, padx=20, fill="x")

        tk.Label(datatype_frame, text="📁 Data Type", bg="#4d4d4d", fg="white",
                 font=("Arial", 11, "bold")).pack(pady=5)

        type_frame = tk.Frame(datatype_frame, bg="#4d4d4d")
        type_frame.pack(pady=5, fill="x", padx=10)

        self.datatype_var = tk.StringVar(value="text")
        tk.Radiobutton(type_frame, text="Text Message", variable=self.datatype_var, value="text",
                       bg="#4d4d4d", fg="white", selectcolor="#5d5d5d", font=("Arial", 9),
                       command=self.on_datatype_change).pack(anchor="w", pady=2)
        tk.Radiobutton(type_frame, text="File (Binary)", variable=self.datatype_var, value="file",
                       bg="#4d4d4d", fg="white", selectcolor="#5d5d5d", font=("Arial", 9),
                       command=self.on_datatype_change).pack(anchor="w", pady=2)
        tk.Radiobutton(type_frame, text="WAV Audio", variable=self.datatype_var, value="wav",
                       bg="#4d4d4d", fg="white", selectcolor="#5d5d5d", font=("Arial", 9),
                       command=self.on_datatype_change).pack(anchor="w", pady=2)

        self.file_btn = tk.Button(datatype_frame, text="📂 Select File", command=self.select_file,
                                  bg="#5d5d5d", fg="white", font=("Arial", 9, "bold"), state="disabled")
        self.file_btn.pack(pady=5, padx=10, fill="x")

        self.file_label = tk.Label(datatype_frame, text="No file selected", bg="#4d4d4d", fg="#aaaaaa",
                                   font=("Arial", 8), wraplength=320)
        self.file_label.pack(pady=2)

        # Pluto settings
        self.create_input(left_panel, "PlutoSDR URI:", "ip:192.168.2.14", "uri")

        # Message box (text)
        self.msg_frame = tk.Frame(left_panel, bg="#3d3d3d")
        self.msg_frame.pack(pady=5, padx=20, fill="both")
        tk.Label(self.msg_frame, text="Message:", bg="#3d3d3d", fg="white",
                 font=("Arial", 10, "bold")).pack(anchor="w")
        self.message_text = scrolledtext.ScrolledText(self.msg_frame, height=3, width=33,
                                                      bg="#4d4d4d", fg="white", font=("Courier", 10))
        self.message_text.pack(pady=5)
        self.message_text.insert("1.0", "Test > 500 bytes: " + ("A" * 520))

        self.create_input(left_panel, "TX Gain (dB):", "-30", "tx_gain")
        self.create_input(left_panel, "RX Gain (dB):", "40", "rx_gain")
        self.create_input(left_panel, "Samples/Symbol:", "10", "samples")

        # ONE-SHOT controls
        oneshot_frame = tk.Frame(left_panel, bg="#4d4d4d")
        oneshot_frame.pack(pady=10, padx=20, fill="x")
        tk.Label(oneshot_frame, text="🚀 ONE-SHOT Control", bg="#4d4d4d", fg="white",
                 font=("Arial", 11, "bold")).pack(pady=5)

        tmp2 = tk.Frame(oneshot_frame, bg="#4d4d4d")
        tmp2.pack(pady=5, fill="x", padx=10)
        tk.Label(tmp2, text="RX Max Reads (fail-safe):", bg="#4d4d4d", fg="white",
                 font=("Arial", 9, "bold")).pack(anchor="w")
        self.maxreads_entry = tk.Entry(tmp2, width=33, bg="#5d5d5d", fg="white", font=("Arial", 9))
        self.maxreads_entry.pack(pady=3)
        self.maxreads_entry.insert(0, "4000")

        # Buttons
        btn_frame = tk.Frame(left_panel, bg="#3d3d3d")
        btn_frame.pack(pady=15)

        self.start_btn = tk.Button(btn_frame, text="▶️ Start (CYCLIC TX)", command=self.start_transmission,
                                   bg="#4CAF50", fg="white", font=("Arial", 12, "bold"), width=16, height=2)
        self.start_btn.pack(side="left", padx=5)

        self.stop_btn = tk.Button(btn_frame, text="⏹️ Stop", command=self.stop_transmission,
                                  bg="#f44336", fg="white", font=("Arial", 12, "bold"), width=10, height=2, state="disabled")
        self.stop_btn.pack(side="left", padx=5)

        # Status
        stat_frame = tk.Frame(left_panel, bg="#4d4d4d")
        stat_frame.pack(pady=10, padx=20, fill="x")
        tk.Label(stat_frame, text="Status:", bg="#4d4d4d", fg="white",
                 font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", padx=5, pady=3)
        self.status_label = tk.Label(stat_frame, text="Ready", bg="#4d4d4d", fg="#4CAF50",
                                     font=("Arial", 10, "bold"))
        self.status_label.grid(row=0, column=1, sticky="w", padx=5, pady=3)

        # Middle panel graphs
        mid_panel = tk.Frame(main_container, bg="#3d3d3d", width=550)
        mid_panel.pack(side="left", fill="both", expand=True, padx=5, pady=5)

        tk.Label(mid_panel, text="📈 Real-time Signal Analysis", font=("Arial", 14, "bold"),
                 bg="#3d3d3d", fg="white").pack(pady=10)

        import matplotlib
        matplotlib.use("TkAgg")
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

        self.fig = Figure(figsize=(7, 10), facecolor="#3d3d3d")

        self.ax_const = self.fig.add_subplot(311)
        self.ax_const.set_facecolor("#2d2d2d")
        self.ax_const.set_title("Constellation Diagram", color="white", fontsize=11, fontweight="bold")
        self.ax_const.grid(True, alpha=0.3, color="gray")

        self.ax_time = self.fig.add_subplot(312)
        self.ax_time.set_facecolor("#2d2d2d")
        self.ax_time.set_title("RX Time Domain", color="white", fontsize=11, fontweight="bold")
        self.ax_time.grid(True, alpha=0.3, color="gray")

        self.ax_metrics = self.fig.add_subplot(313)
        self.ax_metrics.set_facecolor("#2d2d2d")
        self.ax_metrics.set_title("SNR Trend", color="white", fontsize=11, fontweight="bold")
        self.ax_metrics.grid(True, alpha=0.3, color="gray")

        self.fig.tight_layout(pad=2.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=mid_panel)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)

        # Right panel
        right_panel = tk.Frame(main_container, bg="#3d3d3d", width=450)
        right_panel.pack(side="right", fill="both", padx=5, pady=5)

        tk.Label(right_panel, text="📊 Real-time Statistics", font=("Arial", 14, "bold"),
                 bg="#3d3d3d", fg="white").pack(pady=10)

        stats_frame = tk.Frame(right_panel, bg="#4d4d4d")
        stats_frame.pack(pady=10, padx=20, fill="x")

        self.create_stat(stats_frame, "Total RX:", 0, 0, "total_rx")
        self.create_stat(stats_frame, "Decoded:", 0, 1, "decoded")
        self.create_stat(stats_frame, "Success %:", 1, 0, "success_rate")
        self.create_stat(stats_frame, "SNR (dB):", 1, 1, "snr")
        self.create_stat(stats_frame, "TX Mode:", 2, 0, "txmode")
        self.create_stat(stats_frame, "RX Reads:", 2, 1, "rxreads")
        self.create_stat(stats_frame, "RX Buf:", 3, 0, "rxbuf")
        self.create_stat(stats_frame, "Frame:", 3, 1, "frame")

        tk.Label(right_panel, text="💻 Console", font=("Arial", 14, "bold"),
                 bg="#3d3d3d", fg="white").pack(pady=10)

        self.console = scrolledtext.ScrolledText(right_panel, height=10, bg="#1e1e1e", fg="#00ff00",
                                                 font=("Courier", 8), state="disabled")
        self.console.pack(pady=5, padx=20, fill="both", expand=False)

        # ===== Received Files DISPLAY =====
        tk.Label(right_panel, text="📥 Received Files", font=("Arial", 12, "bold"),
                 bg="#3d3d3d", fg="white").pack(pady=8)

        received_frame = tk.Frame(right_panel, bg="#4d4d4d")
        received_frame.pack(pady=5, padx=20, fill="x")

        files_container = tk.Frame(received_frame, bg="#4d4d4d")
        files_container.pack(pady=5, padx=5, fill="x")

        self.files_listbox = tk.Listbox(files_container, height=4, bg="#2d2d2d", fg="white", font=("Courier", 8))
        files_scrollbar = tk.Scrollbar(files_container, orient="vertical", command=self.files_listbox.yview)
        self.files_listbox.config(yscrollcommand=files_scrollbar.set)
        self.files_listbox.pack(side="left", fill="both", expand=True)
        files_scrollbar.pack(side="right", fill="y")

        file_btn_frame = tk.Frame(received_frame, bg="#4d4d4d")
        file_btn_frame.pack(pady=5, fill="x")

        tk.Button(file_btn_frame, text="💾 Save", command=self.save_received_file,
                  bg="#5d5d5d", fg="white", width=8).pack(side="left", padx=2)
        tk.Button(file_btn_frame, text="📂 Open", command=self.open_received_file,
                  bg="#5d5d5d", fg="white", width=8).pack(side="left", padx=2)
        tk.Button(file_btn_frame, text="🗑️ Clear", command=self.clear_received_files,
                  bg="#5d5d5d", fg="white", width=8).pack(side="left", padx=2)

        tk.Label(right_panel, text="💬 Decoded Messages", font=("Arial", 12, "bold"),
                 bg="#3d3d3d", fg="white").pack(pady=10)

        self.messages_list = scrolledtext.ScrolledText(right_panel, height=10, bg="#2d2d2d", fg="white",
                                                       font=("Courier", 8), state="disabled")
        self.messages_list.pack(pady=5, padx=20, fill="both", expand=True)

    def setup_realtime_graphs(self):
        pass

    def create_input(self, parent, label, default, var_name):
        frame = tk.Frame(parent, bg="#3d3d3d")
        frame.pack(pady=5, padx=20, fill="x")
        tk.Label(frame, text=label, bg="#3d3d3d", fg="white", font=("Arial", 9, "bold")).pack(anchor="w")
        entry = tk.Entry(frame, width=33, bg="#4d4d4d", fg="white", font=("Arial", 9))
        entry.pack(pady=3)
        entry.insert(0, default)
        setattr(self, f"{var_name}_entry", entry)

    def create_stat(self, parent, label, row, col, var_name):
        frame = tk.Frame(parent, bg="#5d5d5d")
        frame.grid(row=row, column=col, padx=5, pady=5, sticky="ew")
        tk.Label(frame, text=label, bg="#5d5d5d", fg="white", font=("Arial", 9, "bold")).pack()
        value_label = tk.Label(frame, text="0", bg="#5d5d5d", fg="#4CAF50", font=("Arial", 13, "bold"))
        value_label.pack()
        setattr(self, f"{var_name}_label", value_label)
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_columnconfigure(1, weight=1)

    # ----------------- Handlers -----------------
    def on_modcod_change(self, event=None):
        rate = self.rate_var.get()
        rate_map = {"1/2": 0.5, "1/3": 0.33, "2/3": 0.67, "3/4": 0.75}
        self.code_rate = rate_map[rate]
        self.modcod_info.config(text=f"MODCOD: BPSK {rate}")
        self.log(f"MODCOD: BPSK {rate}", "info")

    def on_datatype_change(self):
        dtype = self.datatype_var.get()
        self.data_type = dtype
        if dtype == "text":
            self.file_btn.config(state="disabled")
            self.msg_frame.pack(pady=5, padx=20, fill="both")
            self.log("Data type: Text Message", "info")
        else:
            self.file_btn.config(state="normal")
            self.msg_frame.pack_forget()
            self.file_data = None
            self.file_name = None
            self.file_label.config(text="No file selected", fg="#aaaaaa")
            if dtype == "wav" and not SCIPY_OK:
                self.log("SciPy belum ada. Install: pip install scipy (buat WAV)", "warning")
            self.log(f"Data type: {dtype.upper()}", "info")

    def select_file(self):
        dtype = self.datatype_var.get()
        if dtype == "file":
            file_path = filedialog.askopenfilename(title="Select File", filetypes=[("All Files", "*.*")])
        else:
            file_path = filedialog.askopenfilename(title="Select WAV File", filetypes=[("WAV Files", "*.wav"), ("All Files", "*.*")])
        if not file_path:
            return
        try:
            self.file_name = os.path.basename(file_path)
            if dtype == "wav":
                if not SCIPY_OK:
                    self.log("SciPy belum terinstall. pip install scipy", "error")
                    return
                sample_rate, audio_data = wavfile.read(file_path)
                if audio_data.dtype != np.int16:
                    audio_data = audio_data.astype(np.int16)
                self.file_data = audio_data.tobytes()
                self.file_label.config(text=f"WAV: {self.file_name}\nSize: {len(self.file_data)} bytes", fg="#4CAF50")
            else:
                with open(file_path, "rb") as f:
                    self.file_data = f.read()
                self.file_label.config(text=f"File: {self.file_name}\nSize: {len(self.file_data)} bytes", fg="#4CAF50")
            self.log(f"Loaded: {self.file_name} ({len(self.file_data)} bytes)", "success")
        except Exception as e:
            self.log(f"Error loading file: {e}", "error")
            self.file_data = None
            self.file_name = None
            self.file_label.config(text="Error loading file", fg="#f44336")

    # ----------------- Graphs (ringan) -----------------
    def update_graphs(self, rx_data):
        sps = max(2, int(self.samples_entry.get()))
        max_sym = min(len(rx_data) // sps, 800)
        if max_sym < 20:
            return
        idxs = (np.arange(max_sym) * sps + (sps // 2)).astype(int)
        idxs = idxs[idxs < len(rx_data)]
        sym = rx_data[idxs].astype(np.complex64)
        if len(sym) < 20:
            return

        i_data = np.real(sym)
        q_data = np.imag(sym)
        i_data = i_data / (np.std(i_data) + 1e-10)
        q_data = q_data / (np.std(q_data) + 1e-10)

        self.ax_const.clear()
        self.ax_const.scatter(i_data, q_data, c="#00ff00", alpha=0.6, s=16)
        self.ax_const.set_facecolor("#2d2d2d")
        self.ax_const.set_title("BPSK Constellation", color="white", fontsize=11, fontweight="bold")
        self.ax_const.grid(True, alpha=0.3, color="gray")
        self.ax_const.set_xlim(-3, 3)
        self.ax_const.set_ylim(-3, 3)

        self.ax_time.clear()
        self.ax_time.plot(np.real(rx_data[:2000]), color="#00ff00", linewidth=0.6)
        self.ax_time.set_facecolor("#2d2d2d")
        self.ax_time.set_title("RX Baseband (I)", color="white", fontsize=11, fontweight="bold")
        self.ax_time.grid(True, alpha=0.3, color="gray")

        self.ax_metrics.clear()
        if len(self.snr_history) > 0:
            x = np.arange(len(self.snr_history))
            self.ax_metrics.plot(x, list(self.snr_history), color="#4CAF50", linewidth=2, label="SNR (dB)")
            self.ax_metrics.legend(loc="upper right", fontsize=8)
        self.ax_metrics.set_facecolor("#2d2d2d")
        self.ax_metrics.set_title("SNR Trend", color="white", fontsize=11, fontweight="bold")
        self.ax_metrics.grid(True, alpha=0.3, color="gray")

        self._ui(self.canvas.draw)

    # ----------------- Framing -----------------
    def create_signal(self, payload_bytes: bytes, sps: int):
        preamble = np.array([1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1] * 4, dtype=np.int8)  # 64b
        length_u32 = len(payload_bytes)
        crc_u32 = self._crc32_u32(payload_bytes)

        bits_payload = self._bytes_to_bits(payload_bytes)

        self.encoder = ConvolutionalEncoder(self.code_rate)
        enc_payload = self.encoder.encode(bits_payload).astype(np.int8)

        header = np.concatenate([preamble, self._u32_to_bits(length_u32), self._u32_to_bits(crc_u32)]).astype(np.int8)
        full_bits = np.concatenate([header, enc_payload]).astype(np.int8)

        symbols = (2 * full_bits - 1).astype(np.float32)
        tx = np.repeat(symbols, sps).astype(np.float32)
        tx_c = (tx + 0j).astype(np.complex64)
        tx_c = (tx_c * (2**13)).astype(np.complex64)

        self.log(f"Frame: bytes={length_u32} | enc bits={len(enc_payload)} | total bits={len(full_bits)} | sps={sps} | samples={len(tx_c)}", "info")
        return tx_c

    def decode_signal(self, rx_signal: np.ndarray, sps: int):
        demod = np.real(rx_signal)
        n_sym = (len(demod) // sps) - 1
        if n_sym <= 200:
            return None

        sym = np.array([np.mean(demod[i * sps:(i + 1) * sps]) for i in range(n_sym)], dtype=np.float32)
        bits = (sym > 0).astype(np.int8)

        preamble = np.array([1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1] * 4, dtype=np.int8)
        Lp = len(preamble)

        scan_max = min(len(bits) - (Lp + 64 + 50), 30000)
        if scan_max <= 0:
            return None

        best_match, best_pos = 0, -1
        for i in range(scan_max):
            m = int(np.sum(bits[i:i + Lp] == preamble))
            if m > best_match:
                best_match, best_pos = m, i
                if best_match >= Lp - 2:
                    break

        if best_pos < 0 or best_match < Lp - 6:
            return None

        idx = best_pos + Lp
        if idx + 64 > len(bits):
            return None

        length_u32 = self._bits_to_u32(bits[idx:idx + 32]); idx += 32
        crc_u32 = self._bits_to_u32(bits[idx:idx + 32]); idx += 32

        if length_u32 <= 0 or length_u32 > 50_000_000:
            return None

        payload_bits_len = int(length_u32 * 8)
        enc_len = int(np.ceil(payload_bits_len * self._fec_factor()))
        if idx + enc_len > len(bits):
            return None

        enc_payload = bits[idx:idx + enc_len]

        self.decoder = ViterbiDecoder(self.code_rate)
        dec_bits = self.decoder.decode(enc_payload).astype(np.int8)
        if len(dec_bits) < payload_bits_len:
            return None
        dec_bits = dec_bits[:payload_bits_len]
        decoded_bytes = self._bits_to_bytes(dec_bits)

        if len(decoded_bytes) != length_u32:
            return None
        if self._crc32_u32(decoded_bytes) != crc_u32:
            return None
        return decoded_bytes

    # ----------------- Start/Stop -----------------
    def start_transmission(self):
        if self.is_running:
            return
        self.is_running = True
        self.rx_count = 0
        self.decode_count = 0
        self.tx_count = 0
        self.snr_history.clear()

        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.status_label.config(text="Running", fg="#4CAF50")

        self.log(f"Start ONE-SHOT CYCLIC: BPSK {self.rate_var.get()}", "info")
        threading.Thread(target=self.run_oneshot, daemon=True).start()

    def stop_transmission(self):
        self.is_running = False
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.status_label.config(text="Stopped", fg="#ff9800")
        self.log("Stopped (destroy TX/RX buffers).", "warning")

        if self.sdr:
            try:
                self.sdr.tx_destroy_buffer()  # wajib buat cyclic stop
            except Exception:
                pass
            try:
                self.sdr.rx_destroy_buffer()
            except Exception:
                pass

    # ----------------- ONE-SHOT runner (CYCLIC + AUTO RX BUFFER) -----------------
    def run_oneshot(self):
        try:
            uri = self.uri_entry.get().strip()
            sps = max(2, int(self.samples_entry.get()))
            tx_gain = int(self.tx_gain_entry.get())
            rx_gain = int(self.rx_gain_entry.get())
            max_reads = max(100, int(self.maxreads_entry.get()))

            # payload
            dtype = self.data_type
            if dtype == "text":
                payload = self.message_text.get("1.0", "end-1c").encode("utf-8")
                tx_name = "text.txt"
                tx_type = "text"
            elif dtype in ["file", "wav"]:
                if self.file_data is None:
                    self.log("No file selected!", "error")
                    self._ui(self.stop_transmission)
                    return
                payload = self.file_data
                tx_name = self.file_name or ("audio.wav" if dtype == "wav" else "file.bin")
                tx_type = dtype
            else:
                self.log("Invalid data type!", "error")
                self._ui(self.stop_transmission)
                return

            # Init SDR
            self.log("Init PlutoSDR...", "info")
            self.sdr = adi.Pluto(uri)
            self.sdr.tx_lo = 2400000000
            self.sdr.rx_lo = 2400000000
            self.sdr.sample_rate = 2000000
            self.sdr.tx_hardwaregain_chan0 = tx_gain
            self.sdr.rx_hardwaregain_chan0 = rx_gain
            self.sdr.gain_control_mode_chan0 = "manual"

            # TX cyclic ON (loop di Pluto)
            self.sdr.tx_cyclic_buffer = True

            # Build frame
            tx_frame = self.create_signal(payload, sps)
            frame_len = int(len(tx_frame))

            # AUTO set RX buffer >= 1 frame (plus margin), cap 2M
            need = int(frame_len * 1.25)
            need = max(1024 * 256, min(1024 * 1024 * 2, need))
            self.sdr.rx_buffer_size = need

            self.log(f"✓ TX:{tx_gain}dB RX:{rx_gain}dB | TX cyclic=ON | RXbuf={need} | frame={frame_len}", "success")
            self.update_stats(snr=0.0, rxreads=self.rx_count, rxbuf=need, frame=frame_len)

            # RX warmup
            for _ in range(2):
                _ = self.sdr.rx()

            # TX sekali (Pluto repeat terus)
            self.log("TX start: cyclic buffer ON (Pluto repeat waveform)", "info")
            self.sdr.tx(tx_frame)
            self.tx_count = 1

            time.sleep(0.15)

            # RX loop sampai ketemu LEN+CRC valid
            self.log("Listening RX until LEN+CRC valid...", "info")
            decoded_ok = None
            last_snr = 0.0

            for n in range(max_reads):
                if not self.is_running:
                    break

                rx = self.sdr.rx()
                self.rx_count += 1

                pwr = float(np.mean(np.abs(rx) ** 2))
                snr_linear = pwr / (1e-10 + float(np.var(np.imag(rx))))
                last_snr = float(10 * np.log10(snr_linear + 1e-10))
                self.snr_history.append(last_snr)

                if n % 2 == 0:
                    self.update_graphs(rx)

                decoded = self.decode_signal(rx, sps)
                if decoded is not None:
                    decoded_ok = decoded
                    break

                if n % 25 == 0:
                    self.update_stats(snr=last_snr, rxreads=self.rx_count, rxbuf=need, frame=frame_len)

            if decoded_ok is None:
                self.log("❌ Decode gagal (LEN+CRC gak ketemu). Turunin SPS (10), cek attenuator/gain/kabel/uri.", "error")
                self._ui(self.stop_transmission)
                return

            self.decode_count += 1
            self.update_stats(snr=last_snr, rxreads=self.rx_count, rxbuf=need, frame=frame_len)

            if dtype == "text":
                txt = decoded_ok.decode("utf-8", errors="ignore")
                self.log(f"✅ RX OK: {txt[:140]}", "success")
                self.add_message(txt)
            else:
                self.log(f"✅ RX OK: {tx_name} ({len(decoded_ok)} bytes)", "success")
                self.add_message(f"RX OK: {tx_name} ({len(decoded_ok)} bytes)")
                self.store_received_file(decoded_ok, tx_name, tx_type)

            self._ui(self.stop_transmission)

        except Exception as e:
            import traceback
            self.log(f"Error: {e}", "error")
            self.log(traceback.format_exc(), "error")
            self._ui(self.stop_transmission)

    # ----------------- Received file handling (DISPLAY) -----------------
    def store_received_file(self, data, filename, filetype):
        ts = time.strftime("%H:%M:%S")
        self.received_files.append({
            "data": data,
            "filename": filename,
            "type": filetype,
            "size": len(data),
            "timestamp": ts
        })
        self._ui(self.files_listbox.insert, "end", f"[{ts}] {filename} ({len(data)} B)")

    def save_received_file(self):
        sel = self.files_listbox.curselection()
        if not sel:
            self.log("No file selected", "warning")
            return
        f = self.received_files[sel[0]]
        path = filedialog.asksaveasfilename(initialfile=f["filename"], defaultextension="")
        if not path:
            return
        try:
            with open(path, "wb") as fp:
                fp.write(f["data"])
            self.log(f"Saved: {os.path.basename(path)}", "success")
        except Exception as e:
            self.log(f"Save error: {e}", "error")

    def open_received_file(self):
        sel = self.files_listbox.curselection()
        if not sel:
            self.log("No file selected", "warning")
            return
        f = self.received_files[sel[0]]
        try:
            # kasih suffix sesuai ext biar kebuka bener
            _, ext = os.path.splitext(f["filename"])
            suffix = ext if ext else ".bin"
            tmp = tempfile.NamedTemporaryFile(delete=False, suffix=suffix)
            tmp.write(f["data"])
            tmp.close()

            if platform.system() == "Windows":
                os.startfile(tmp.name)
            elif platform.system() == "Darwin":
                subprocess.run(["open", tmp.name])
            else:
                subprocess.run(["xdg-open", tmp.name])

            self.log(f"Opened: {f['filename']}", "success")
        except Exception as e:
            self.log(f"Open error: {e}", "error")

    def clear_received_files(self):
        self.received_files.clear()
        self.files_listbox.delete(0, "end")
        self.log("Received files cleared", "info")


if __name__ == "__main__":
    root = tk.Tk()
    app = PlutoSDRMODCOD(root)
    root.mainloop()
