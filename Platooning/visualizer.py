import tkinter as tk
import socket
import struct
import time

# --- MINI-CONFIGURATION ---
UDP_IP = "0.0.0.0"
UDP_PORT = 4999
METERS_TO_PIXELS = 6.0    # Scale: 6 pixels per meter
TRUCK_WIDTH = 50          # Visual length of the truck
TRUCK_HEIGHT = 20         # Visual height
TIMEOUT = 2.0             

# --- NETWORK SETUP ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

trucks = {}

def parse_packet(data):
    try:
        if len(data) >= 32:
            unpacked = struct.unpack('i 4x d d ? ? 2x l', data[0:32])
            return unpacked[0], unpacked[1], unpacked[2], unpacked[3], unpacked[4], unpacked[5]
    except Exception as e:
        print(f"Unpack error: {e}")
    return None

class PlatoonVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("DPS Driving Simulator")
        
        # Window size
        self.width = 800
        self.height = 250
        self.canvas = tk.Canvas(root, width=self.width, height=self.height, bg="#1a1a1a")
        self.canvas.pack()
        
        # Road Background
        self.road_y_top = 100
        self.road_y_bot = 150
        self.cy = 125  # Center of road

        self.update()

    def draw_moving_road(self, leader_pos_meters):
        """ Draws dashed lines that scroll backward to simulate speed """
        self.canvas.create_rectangle(0, self.road_y_top, self.width, self.road_y_bot, fill="#333", outline="")
        
        dash_len = 20
        gap_len = 20
        pattern_len = dash_len + gap_len
        
        # Shift LEFT as position increases
        shift = (leader_pos_meters * METERS_TO_PIXELS) % pattern_len
        
        start_x = -shift 
        while start_x < self.width:
            self.canvas.create_line(start_x, self.cy, start_x + dash_len, self.cy, 
                                    fill="white", width=2, tags="road_lines")
            start_x += pattern_len

    def draw_truck(self, x, y, color, label):
        """ Draws the custom truck shape """
        w = TRUCK_WIDTH
        h = TRUCK_HEIGHT
        
        trailer_w = w * 0.65
        cab_w = w * 0.25
        gap = w * 0.05
        
        left_edge = x - (w / 2)
        right_edge = x + (w / 2)
        
        # 1. Trailer
        self.canvas.create_rectangle(
            left_edge, y - h/2, 
            left_edge + trailer_w, y + h/2 - 4, 
            fill=color, outline="white", width=1, tags="truck_data"
        )
        
        # 2. Cab
        cab_x1 = left_edge + trailer_w + gap
        cab_x2 = right_edge
        self.canvas.create_polygon(
            cab_x1, y + h/2 - 4, cab_x1, y - h/2 + 4, 
            cab_x1 + 5, y - h/2, cab_x2, y - h/2, 
            cab_x2, y + h/2 - 4,
            fill=color, outline="white", width=1, tags="truck_data"
        )
        
        # 3. Window
        self.canvas.create_rectangle(cab_x1 + 8, y - h/2 + 2, cab_x2 - 2, y - h/4, fill="#87CEEB", outline="", tags="truck_data")
        
        # 4. Wheels
        wheel_r = 4
        wheel_y = y + h/2 - 2
        for wx in [left_edge + 5, left_edge + trailer_w - 10, cab_x2 - 12]:
            self.canvas.create_oval(wx, wheel_y - wheel_r, wx + 2*wheel_r, wheel_y + wheel_r, fill="black", outline="gray", tags="truck_data")

        # 5. Label (Moved up to y - 35 to fit 2 lines)
        self.canvas.create_text(x, y - 35, text=label, fill="white", font=("Arial", 8, "bold"), justify="center", tags="truck_data")

    def update(self):
        # 1. Receive Data
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                parsed = parse_packet(data)
                if parsed:
                    t_id, pos, spd, brake, decoupled, ts = parsed
                    trucks[t_id] = {
                        'pos': pos, 'spd': spd, 'brake': brake, 
                        'decoupled': decoupled, 'last_seen': time.time()
                    }
            except BlockingIOError:
                break

        # 2. Clear Screen
        self.canvas.delete("truck_data")
        self.canvas.delete("road_lines") 
        
        now = time.time()
        active_trucks = []
        for t_id, t in list(trucks.items()):
            if now - t['last_seen'] < TIMEOUT:
                active_trucks.append((t_id, t['pos']))
        
        active_trucks.sort(key=lambda x: x[1], reverse=True)

        # 3. Draw Scene
        if active_trucks:
            leader_pos = active_trucks[0][1]
            
            # Animate Road
            self.draw_moving_road(leader_pos)
            
            # Camera Focus: 75% to the right
            screen_focus_x = 600  
            camera_offset = leader_pos 

            for i, (t_id, pos) in enumerate(active_trucks):
                t = trucks[t_id]
                rel_pos = pos - camera_offset
                pixel_x = screen_focus_x + (rel_pos * METERS_TO_PIXELS)

                # Color Coding
                color = "#00BFFF" 
                if t['decoupled']: color = "#FFA500"
                if t['brake']: color = "#FF4500"
                if t_id == 0: color = "#32CD32"

                # --- UPDATED LABEL ---
                # Added '\nDist: ...m' to show distance
                label = f"T{t_id}: {t['spd']*3.6:.0f} km/h\nDist: {t['pos']:.1f} m"
                
                self.draw_truck(pixel_x, self.cy, color, label)

                # Distance Lines
                if i > 0:
                    prev_pos = active_trucks[i-1][1]
                    dist = prev_pos - pos
                    prev_pixel_x = screen_focus_x + ((prev_pos - camera_offset) * METERS_TO_PIXELS)
                    
                    start_line = pixel_x + (TRUCK_WIDTH/2)
                    end_line = prev_pixel_x - (TRUCK_WIDTH/2)
                    
                    if end_line > start_line:
                        self.canvas.create_line(start_line, self.cy + 10, end_line, self.cy + 10, 
                                                fill="yellow", dash=(2, 2), tags="truck_data")
                        self.canvas.create_text((start_line + end_line)/2, self.cy + 20, 
                                                text=f"{dist:.1f}m", fill="yellow", font=("Arial", 7), tags="truck_data")
        else:
            self.draw_moving_road(0)
            self.canvas.create_text(400, 50, text="Waiting for Platoon...", fill="white", tags="truck_data")

        self.root.after(40, self.update) 

if __name__ == "__main__":
    root = tk.Tk()
    root.attributes('-topmost', True) 
    app = PlatoonVisualizer(root)
    root.mainloop()