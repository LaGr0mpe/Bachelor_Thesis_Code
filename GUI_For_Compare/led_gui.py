import socket
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
except ImportError:
    serial = None


UART_PORT = "/dev/ttyACM0"
UART_BAUD = 115200

UDP_IP = "192.168.1.25"
UDP_PORT = 1234


class LedControlApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Nucleo LED Control (UART + UDP)")
        self.resizable(False, False)

        self.uart_port_var = tk.StringVar(value=UART_PORT)
        self.uart_baud_var = tk.IntVar(value=UART_BAUD)

        self.udp_ip_var = tk.StringVar(value=UDP_IP)
        self.udp_port_var = tk.IntVar(value=UDP_PORT)

        self._build_ui()

    def _build_ui(self):
        pad = {"padx": 10, "pady": 6}

        # UART frame
        uart_frame = ttk.LabelFrame(self, text="UART")
        uart_frame.grid(row=0, column=0, sticky="ew", **pad)

        ttk.Label(uart_frame, text="Port:").grid(row=0, column=0, sticky="w")
        ttk.Entry(uart_frame, textvariable=self.uart_port_var, width=18).grid(row=0, column=1, sticky="w")

        ttk.Label(uart_frame, text="Baud:").grid(row=0, column=2, sticky="w", padx=(10, 0))
        ttk.Entry(uart_frame, textvariable=self.uart_baud_var, width=8).grid(row=0, column=3, sticky="w")

        ttk.Button(uart_frame, text="UART ON", command=lambda: self.send_uart(b"1")).grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(uart_frame, text="UART OFF", command=lambda: self.send_uart(b"0")).grid(row=1, column=2, columnspan=2, sticky="ew", pady=(8, 0))

        # UDP frame
        udp_frame = ttk.LabelFrame(self, text="Ethernet (UDP)")
        udp_frame.grid(row=1, column=0, sticky="ew", **pad)

        ttk.Label(udp_frame, text="IP:").grid(row=0, column=0, sticky="w")
        ttk.Entry(udp_frame, textvariable=self.udp_ip_var, width=18).grid(row=0, column=1, sticky="w")

        ttk.Label(udp_frame, text="Port:").grid(row=0, column=2, sticky="w", padx=(10, 0))
        ttk.Entry(udp_frame, textvariable=self.udp_port_var, width=8).grid(row=0, column=3, sticky="w")

        ttk.Button(udp_frame, text="Ethernet ON", command=lambda: self.send_udp(b"1")).grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(udp_frame, text="Ethernet OFF", command=lambda: self.send_udp(b"0")).grid(row=1, column=2, columnspan=2, sticky="ew", pady=(8, 0))

        # Status
        self.status_var = tk.StringVar(value="Ready.")
        status = ttk.Label(self, textvariable=self.status_var)
        status.grid(row=2, column=0, sticky="w", padx=10, pady=(0, 10))

    def set_status(self, msg: str):
        self.status_var.set(msg)

    def send_uart(self, payload: bytes):
        if serial is None:
            messagebox.showerror("pyserial missing", "Install pyserial: pip install pyserial")
            return

        port = self.uart_port_var.get().strip()
        baud = int(self.uart_baud_var.get())

        try:
            # Открываем порт кратковременно, шлём 1 байт и закрываем.
            with serial.Serial(port=port, baudrate=baud, timeout=0.2, write_timeout=0.2) as ser:
                ser.write(payload)
                ser.flush()
            self.set_status(f"UART sent: {payload!r} -> {port}@{baud}")
        except Exception as e:
            messagebox.showerror("UART error", str(e))
            self.set_status("UART error.")

    def send_udp(self, payload: bytes):
        ip = self.udp_ip_var.get().strip()
        port = int(self.udp_port_var.get())

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.sendto(payload, (ip, port))
            self.set_status(f"UDP sent: {payload!r} -> {ip}:{port}")
        except Exception as e:
            messagebox.showerror("UDP error", str(e))
            self.set_status("UDP error.")


if __name__ == "__main__":
    app = LedControlApp()
    app.mainloop()
