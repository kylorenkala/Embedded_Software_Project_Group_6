import tkinter as tk
import socket
import struct
import time

# --- CONFIGURATION ---
UDP_IP = "0.0.0.0"
UDP_PORT = 4999
METERS_TO_PIXELS = 12.0   
TRUCK_WIDTH = 100         
TRUCK_HEIGHT = 40
TIMEOUT = 2.0             

# --- NETWORK SETUP ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

trucks = {}

def parse_packet(data):
    try:
        # Old format: i 4x d d ? 3x l
        # New format: i 4x d d ? ? 2x l  (Two bools: Brake, Decoupled)
        
        # Check for the new larger packet size (approx 33-40 bytes depending on packing)
        if len(data) >= 32:
             # Try unpacking with 2 bools
             try:
                 # i=4, d=8, d=8, ?=1, ?=1, pad=2, l=8 => Total 32 bytes
                 # We simply add another '?' for the decoupled flag
                 unpacked = struct.unpack('i 4x d d ? ? 2x l', data[0:32])
                 return unpacked[0], unpacked[1], unpacked[2], unpacked[3], unpacked[5] # Return ID, Pos, Spd, Brake, Time
             except:
                 # Fallback to old format if something goes wrong
                 return struct.unpack('i 4x d d ? 3x l', data[0:32])
    except:
        return None
    return None

class PlatoonVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("🚛 Distributed Platoon Simulation")
        
        # --- 1. SET YOUR CUSTOM SIZE HERE ---
        # Change these numbers to whatever you want!
        # Examples: 800x600, 1280x720, 1000x500
        WINDOW_WIDTH = 1900
        WINDOW_HEIGHT = 600
        
        self.root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        
        # --- 2. DISABLE FULLSCREEN ---
        # Comment this line out by adding a # at the start
        # self.root.attributes("-fullscreen", True) 

        self.root.bind("<Escape>", self.close_fullscreen)

        # --- 3. TELL THE LOGIC YOUR NEW SIZE ---
        # We manually set these so the drawing code knows where the center is
        self.screen_w = WINDOW_WIDTH
        self.screen_h = WINDOW_HEIGHT

        # ... (Rest of the code remains the same) ...
        self.canvas = tk.Canvas(root, bg="#222222", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.smooth_distances = {} 
        self.update_simulation()

    def close_fullscreen(self, event=None):
        self.root.attributes("-fullscreen", False)
        self.root.quit()

    def draw_obstacle(self, x, y):
        w = 30
        h = 120
        self.canvas.create_rectangle(x, y - h/2, x + w, y + h/2, fill="#AA4444", outline="white", width=3)
        for i in range(0, 120, 20):
            sy = (y - h/2) + i
            self.canvas.create_line(x, sy, x + w, sy + 15, fill="black", width=4)
        self.canvas.create_text(x + 15, y - h/2 - 25, text="OBSTACLE", fill="red", font=("Arial", 14, "bold"))

    def draw_truck(self, x, y, color, truck_id, speed, braking, position):
        trailer_w = TRUCK_WIDTH * 0.7
        cabin_w = TRUCK_WIDTH * 0.25
        self.canvas.create_rectangle(x - TRUCK_WIDTH, y - TRUCK_HEIGHT/2, x - TRUCK_WIDTH + trailer_w, y + TRUCK_HEIGHT/2, fill=color, outline="white", width=3)
        cabin_x = x - TRUCK_WIDTH + trailer_w + 4
        self.canvas.create_rectangle(cabin_x, y - TRUCK_HEIGHT/2 + 8, cabin_x + cabin_w, y + TRUCK_HEIGHT/2, fill=color, outline="white", width=3)
        
        wheel_r = 8
        wheel_y = y + TRUCK_HEIGHT/2
        self.canvas.create_oval(x - TRUCK_WIDTH + 15, wheel_y - wheel_r, x - TRUCK_WIDTH + 15 + wheel_r*2, wheel_y + wheel_r, fill="black")
        self.canvas.create_oval(x - TRUCK_WIDTH + 55, wheel_y - wheel_r, x - TRUCK_WIDTH + 55 + wheel_r*2, wheel_y + wheel_r, fill="black")
        self.canvas.create_oval(cabin_x + 10, wheel_y - wheel_r, cabin_x + 10 + wheel_r*2, wheel_y + wheel_r, fill="black")

        if braking:
            self.canvas.create_oval(x - TRUCK_WIDTH - 8, y - 15, x - TRUCK_WIDTH + 8, y + 15, fill="#FF0000", outline="#FF5555", width=2)

        spd_kmh = int(speed * 3.6)
        self.canvas.create_text(x - TRUCK_WIDTH, y - 45, text=f"ID:{truck_id} | {spd_kmh} km/h", fill="white", anchor="w", font=("Arial", 12, "bold"))
        self.canvas.create_text(x - TRUCK_WIDTH, y - 70, text=f"Dist: {abs(position):.1f} m", fill="#00FFFF", anchor="w", font=("Arial", 11))

    def update_simulation(self):
        try:
            while True:
                data, addr = sock.recvfrom(1024)
                parsed = parse_packet(data)
                if parsed:
                    t_id, pos, spd, brake, ts = parsed
                    trucks[t_id] = {'pos': pos, 'spd': spd, 'brake': brake, 'ts': time.time()}
        except BlockingIOError: pass

        self.canvas.delete("all")
        cy = self.screen_h / 2
        now = time.time()
        
        leader_id = 0
        if 0 not in trucks and trucks: leader_id = max(trucks.keys())
        
        truck_visuals = {}
        for t_id, t in trucks.items():
            dt = now - t['ts']
            if dt > 0.5: dt = 0.0 
            predicted_pos = t['pos'] + (t['spd'] * dt)
            truck_visuals[t_id] = predicted_pos

        camera_focus_pos = 0
        if leader_id in trucks and leader_id in truck_visuals:
            camera_focus_pos = truck_visuals[leader_id]

        scroll_offset = (camera_focus_pos * METERS_TO_PIXELS) % 150 
        for x in range(0, self.screen_w + 150, 150):
            draw_x = x - scroll_offset
            self.canvas.create_rectangle(draw_x, cy - 4, draw_x+60, cy + 4, fill="white", outline="")
            
        self.canvas.create_line(0, cy - 80, self.screen_w, cy - 80, fill="gray", width=5)
        self.canvas.create_line(0, cy + 80, self.screen_w, cy + 80, fill="gray", width=5)

        if not trucks:
            self.canvas.create_text(self.screen_w/2, cy - 100, text="Waiting for Platoon...", fill="white", font=("Arial", 24))
        else:
            screen_focus_x = self.screen_w * 0.8  
            
            if 0 in trucks and trucks[0]['brake']:
                obstacle_dist = 20.0 
                obs_pixel_x = screen_focus_x + (obstacle_dist * METERS_TO_PIXELS)
                self.draw_obstacle(obs_pixel_x, cy)

            # --- SORT BY POSITION (Descending) ---
            # This ensures we connect lines based on who is actually physically ahead
            active_trucks = []
            for tid, t in trucks.items():
                if now - t['ts'] < TIMEOUT:
                    active_trucks.append((tid, truck_visuals[tid]))
            
            # Sort: Largest Position (Front) -> Smallest Position (Back)
            active_trucks.sort(key=lambda x: x[1], reverse=True)

            for i, (t_id, visual_pos) in enumerate(active_trucks):
                t = trucks[t_id]
                relative_pos = visual_pos - camera_focus_pos
                pixel_x = screen_focus_x + (relative_pos * METERS_TO_PIXELS)

                if -200 < pixel_x < self.screen_w + 200:
                    color = "#44AAFF"
                    if t['brake']: color = "#FF2222"
                    if t_id == 0: color = "#FFAA00"

                    self.draw_truck(pixel_x, cy, color, t_id, t['spd'], t['brake'], visual_pos)

                    # Connect to the truck physically in front (index i-1)
                    if i > 0: 
                        pred_id = active_trucks[i-1][0]
                        pred_visual_pos = active_trucks[i-1][1]
                            
                        # Smoothing
                        raw_dist = pred_visual_pos - visual_pos
                        if t_id not in self.smooth_distances:
                            self.smooth_distances[t_id] = raw_dist
                        else:
                            self.smooth_distances[t_id] = (self.smooth_distances[t_id] * 0.9) + (raw_dist * 0.1)
                        final_dist = self.smooth_distances[t_id]

                        # Draw Line
                        pred_rel = pred_visual_pos - camera_focus_pos
                        pred_px = screen_focus_x + (pred_rel * METERS_TO_PIXELS)
                        lx1 = pixel_x
                        lx2 = pred_px - TRUCK_WIDTH
                        
                        if lx2 > lx1:
                            self.canvas.create_line(lx1, cy, lx2, cy, fill="#FFFF00", width=3, dash=(8, 4))
                            self.canvas.create_text((lx1+lx2)/2, cy + 40, text=f"{final_dist:.1f}m", fill="#FFFF00", font=("Arial", 12))

        self.root.after(16, self.update_simulation)

if __name__ == "__main__":
    root = tk.Tk()
    app = PlatoonVisualizer(root)
    root.mainloop()