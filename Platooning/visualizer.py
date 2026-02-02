import tkinter as tk
import socket
import struct
import time

# --- CONFIGURATION ---
UDP_IP = "0.0.0.0"
UDP_PORT = 4999

# --- VIEW SETTINGS ---
WINDOW_WIDTH = 1600       
WINDOW_HEIGHT = 600       
METERS_TO_PIXELS = 4.0    
TRUCK_WIDTH = 50          
TRUCK_HEIGHT = 20         
TIMEOUT = 2.0             

# --- NETWORK SETUP ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

trucks = {}

def parse_packet(data):
    try:
        # Expected C++ Struct: int id, double pos, double speed, bool brake, bool decoupled, long timestamp
        if len(data) >= 32:
            unpacked = struct.unpack('i 4x d d ? ? 2x l', data[0:32])
            return unpacked[0], unpacked[1], unpacked[2], unpacked[3], unpacked[4], unpacked[5]
    except Exception as e:
        print(f"Unpack error: {e}")
    return None

class PlatoonVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("DPS Truck Platoon (Distance Travelled)")
        
        self.width = WINDOW_WIDTH
        self.height = WINDOW_HEIGHT
        self.canvas = tk.Canvas(root, width=self.width, height=self.height, bg="#222222")
        self.canvas.pack()
        
        self.cy = self.height / 2
        self.update_plot()

    def draw_truck(self, x, y, color, label_text, is_brake):
        # Body
        self.canvas.create_rectangle(x - TRUCK_WIDTH/2, y - TRUCK_HEIGHT/2,
                                     x + TRUCK_WIDTH/2, y + TRUCK_HEIGHT/2,
                                     fill=color, outline="White", width=2)
        
        # Wheels
        wheel_y = y + TRUCK_HEIGHT/2
        self.canvas.create_oval(x - 20, wheel_y - 5, x - 10, wheel_y + 5, fill="#444")
        self.canvas.create_oval(x + 10, wheel_y - 5, x + 20, wheel_y + 5, fill="#444")

        # Brake Lights
        if is_brake:
            self.canvas.create_oval(x + 20, y - 5, x + 28, y + 5, fill="red", outline="red")

        # Label
        self.canvas.create_text(x, y - 35, text=label_text, fill="Red", font=("Arial", 10, "bold"))

    def draw_moving_road(self, offset):
        marker_spacing = 50 
        start = int(-(offset % marker_spacing))
        for i in range(start, self.width, marker_spacing):
            self.canvas.create_line(i, self.cy + 25, i + 20, self.cy + 25, fill="white", width=2)
            self.canvas.create_line(i, self.cy - 25, i + 20, self.cy - 25, fill="white", width=2)

    def update_plot(self):
        # 1. Receive Data
        try:
            while True:
                data, addr = sock.recvfrom(1024)
                parsed = parse_packet(data)
                if parsed:
                    t_id, pos, spd, brake, decoupled, ts = parsed
                    
                    # Logic to track Distance Travelled
                    if t_id not in trucks:
                        # First time seeing this truck? Save start_pos.
                        trucks[t_id] = {
                            "start_pos": pos, 
                            "pos": pos, 
                            "spd": spd, 
                            "brake": brake, 
                            "decoupled": decoupled, 
                            "last_seen": time.time()
                        }
                    else:
                        # Update existing, KEEP start_pos
                        trucks[t_id].update({
                            "pos": pos, 
                            "spd": spd, 
                            "brake": brake, 
                            "decoupled": decoupled, 
                            "last_seen": time.time()
                        })
        except BlockingIOError:
            pass

        # 2. Clear Screen
        self.canvas.delete("all")

        # 3. Filter Old Trucks & Find Leader
        current_time = time.time()
        active_trucks = []
        max_pos = -999999.0

        for t_id in list(trucks.keys()):
            if current_time - trucks[t_id]["last_seen"] > TIMEOUT:
                del trucks[t_id] 
            else:
                pos = trucks[t_id]["pos"]
                active_trucks.append((t_id, pos, trucks[t_id]))
                if pos > max_pos:
                    max_pos = pos

        # 4. Draw Scene
        if active_trucks:
            # Camera follows leader (Leader is fixed at 85% screen width)
            camera_offset = max_pos - (self.width * 0.85 / METERS_TO_PIXELS)
            
            active_trucks.sort(key=lambda x: x[1]) 

            self.draw_moving_road(camera_offset * METERS_TO_PIXELS)

            for i, (t_id, pos, t) in enumerate(active_trucks):
                pixel_x = (pos - camera_offset) * METERS_TO_PIXELS
                
                color = "#00CC00" 
                if t["decoupled"]: color = "orange"
                if t["brake"]: color = "red"
                if t_id == 0: color = "#0088FF"

                # --- CALCULATE DISTANCE TRAVELLED ---
                dist_travelled = t["pos"] - t["start_pos"]
                
                # Label with Distance
                label = f"T{t_id}\n{t['spd']*3.6:.0f} km/h\nRun: {dist_travelled:.1f} m"
                
                self.draw_truck(pixel_x, self.cy, color, label, t["brake"])

                # Draw Gap Lines
                if i < len(active_trucks) - 1:
                    next_truck_pos = active_trucks[i+1][1]
                    dist = next_truck_pos - pos
                    
                    next_pixel_x = (next_truck_pos - camera_offset) * METERS_TO_PIXELS
                    
                    mid_y = self.cy + 30
                    self.canvas.create_line(pixel_x + 25, mid_y, next_pixel_x - 25, mid_y, 
                                            fill="yellow", arrow=tk.BOTH)
                    mid_x = (pixel_x + next_pixel_x) / 2
                    self.canvas.create_text(mid_x, mid_y + 10, text=f"Gap: {dist:.1f}m", fill="yellow", font=("Arial", 9))

        else:
            self.canvas.create_text(self.width/2, self.height/2, text="Waiting for Platoon...", fill="white")

        self.root.after(50, self.update_plot)

if __name__ == "__main__":
    root = tk.Tk()
    app = PlatoonVisualizer(root)
    root.mainloop()