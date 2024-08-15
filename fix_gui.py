import tkinter as tk
from tkinter import messagebox, simpledialog
from pymavlink import mavutil
from concurrent.futures import ThreadPoolExecutor
import serial.tools.list_ports
from pymavlink_helper import PyMavlinkHelper
import time
from typing import Tuple, List, Dict


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    available_ports = [port.device for port in ports]
    return available_ports


def select_ports():
    available_ports = list_serial_ports()

    if not available_ports:
        messagebox.showerror("Error", "No available serial ports found.")
        return None

    indexed_ports = {
        str(index): port for index, port in enumerate(available_ports, start=1)
    }
    port_list = "\n".join([f"{index}: {port}" for index, port in indexed_ports.items()])

    drone_count = simpledialog.askinteger(
        "Drone Count",
        "How many drones do you want to control?",
        minvalue=1,
        maxvalue=10,
    )
    if drone_count is None:
        messagebox.showerror("Error", "No drone count provided.")
        return None

    selected_ports = []
    for i in range(drone_count):
        port_index = simpledialog.askstring(
            "Select Serial Port",
            f"Select port for Drone {i+1} by index:\nAvailable ports:\n{port_list}",
        )
        if port_index in indexed_ports:
            selected_ports.append(indexed_ports[port_index])
            available_ports.remove(indexed_ports[port_index])
            indexed_ports.pop(port_index)
            port_list = "\n".join(
                [f"{index}: {port}" for index, port in indexed_ports.items()]
            )
        else:
            messagebox.showerror("Error", f"Invalid port selected for Drone {i+1}.")
            return None

    return selected_ports


class DroneControlApp:
    def __init__(self, root, drone_helper):
        self.root = root
        self.root.title("Drone Control Panel")
        self.drone_helper = drone_helper
        self.executor = ThreadPoolExecutor(max_workers=10)

        self.drone_frames = []
        self.takeoff_entries = []
        self.move_entries = []
        self.arm_status_labels = []

        for i in range(self.drone_helper.drone_count):
            frame = tk.Frame(root, padx=10, pady=10, borderwidth=2, relief="ridge")
            frame.grid(row=0, column=i)

            tk.Label(frame, text=f"Drone {i+1}").pack()

            tk.Button(frame, text="Arm", command=lambda i=i: self.arm_drone(i)).pack(
                pady=5
            )
            tk.Button(
                frame, text="Disarm", command=lambda i=i: self.disarm_drone(i)
            ).pack(pady=5)

            tk.Label(frame, text="Takeoff Altitude (m)").pack()
            takeoff_entry = tk.Entry(frame)
            takeoff_entry.pack(pady=5)
            self.takeoff_entries.append(takeoff_entry)
            tk.Button(
                frame, text="Takeoff", command=lambda i=i: self.takeoff_drone(i)
            ).pack(pady=5)

            tk.Label(frame, text="Move to (x,y,z)").pack()
            move_entry = tk.Entry(frame)
            move_entry.pack(pady=5)
            self.move_entries.append(move_entry)
            tk.Button(frame, text="Move", command=lambda i=i: self.move_drone(i)).pack(
                pady=5
            )

            tk.Button(frame, text="Land", command=lambda i=i: self.land_drone(i)).pack(
                pady=5
            )

            arm_status_label = tk.Label(frame, text="Arm Status: Unknown")
            arm_status_label.pack(pady=5)
            self.arm_status_labels.append(arm_status_label)

            self.drone_frames.append(frame)

        tk.Button(root, text="Check Arm Status", command=self.check_arm_status).grid(
            row=1, column=0, columnspan=self.drone_helper.drone_count, pady=10
        )

        tk.Label(root, text="Swarm Move Offset (x,y,z)").grid(
            row=2, column=0, columnspan=self.drone_helper.drone_count
        )
        self.swarm_move_entry = tk.Entry(root)
        self.swarm_move_entry.grid(
            row=3, column=0, columnspan=self.drone_helper.drone_count, pady=5
        )
        tk.Button(root, text="Swarm Move", command=self.swarm_move_drones).grid(
            row=4, column=0, columnspan=self.drone_helper.drone_count, pady=10
        )

    def arm_drone(self, drone_index):
        try:
            self.executor.submit(self.drone_helper._arm_drone, drone_index)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to arm drone {drone_index+1}: {e}")

    def disarm_drone(self, drone_index):
        try:
            self.executor.submit(
                self.drone_helper._force_disarm, self.drone_helper.vehicles[drone_index]
            )
        except Exception as e:
            messagebox.showerror(
                "Error", f"Failed to disarm drone {drone_index+1}: {e}"
            )

    def takeoff_drone(self, drone_index):
        altitude = self.takeoff_entries[drone_index].get()
        try:
            altitude = float(altitude)
            self.executor.submit(
                self.drone_helper._takeoff,
                self.drone_helper.vehicles[drone_index],
                altitude,
            )
        except ValueError:
            messagebox.showerror("Error", "Invalid altitude value")
        except Exception as e:
            messagebox.showerror(
                "Error", f"Failed to takeoff drone {drone_index+1}: {e}"
            )

    def move_drone(self, drone_index):
        move_str = self.move_entries[drone_index].get()
        try:
            coords = tuple(map(float, move_str.split(",")))
            if len(coords) != 3:
                raise ValueError("Must provide exactly three coordinates (x, y, z)")

            self.executor.submit(
                self.drone_helper._move,
                self.drone_helper.vehicles[drone_index],
                coords,
            )

            # Monitor coordinates in the main thread
            def monitor_coords():
                while True:
                    current_coords = self.drone_helper.get_current_state()[drone_index]
                    print(
                        f"Drone {drone_index+1} current coordinates: {current_coords}"
                    )
                    time.sleep(1)

            self.executor.submit(monitor_coords)

        except ValueError:
            messagebox.showerror(
                "Error", "Invalid coordinates. Please provide in the format x,y,z"
            )
        except Exception as e:
            messagebox.showerror("Error", f"Failed to move drone {drone_index+1}: {e}")

    def swarm_move_drones(self):
        move_str = self.swarm_move_entry.get()
        try:
            coords = tuple(map(float, move_str.split(",")))
            if len(coords) != 3:
                raise ValueError("Must provide exactly three coordinates (x, y, z)")
            self.executor.submit(self.drone_helper.swarm_move, coords)
        except ValueError:
            messagebox.showerror(
                "Error", "Invalid coordinates. Please provide in the format x,y,z"
            )
        except Exception as e:
            messagebox.showerror("Error", f"Failed to execute swarm move: {e}")

    def land_drone(self, drone_index):
        try:
            self.executor.submit(self.drone_helper._land_drone, drone_index)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to land drone {drone_index+1}: {e}")

    def check_arm_status(self):
        try:
            arm_status = self.drone_helper._check_is_armed()
            for i, status in enumerate(arm_status):
                status_text = "Armed" if status else "Disarmed"
                self.arm_status_labels[i].config(text=f"Arm Status: {status_text}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to check arm status: {e}")


if __name__ == "__main__":
    root = tk.Tk()

    selected_ports = select_ports()
    if selected_ports:
        drone_helper = PyMavlinkHelper(
            drone_count=len(selected_ports),
            connection_strings=selected_ports,
            baud_rate=57600,
        )
        drone_helper.initialize_environment()
        app = DroneControlApp(root, drone_helper)

        root.mainloop()
    else:
        messagebox.showerror("Error", "No ports selected.")
