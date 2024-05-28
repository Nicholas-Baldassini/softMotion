import tkinter as tk
import random

class CircleDrawer:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, width=800, height=600, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Center
        self.canvas_width = 800
        self.canvas_height = 600
        self.center_x = self.canvas_width // 2
        self.center_y = self.canvas_height // 2

        # Scale
        self.scale = 25

        self.coord_label = tk.Label(root, text="x: 0, y: 0", bg="white")
        self.coord_label.place(x=10, y=10)

        self.save_button = tk.Button(root, text="Save to file", command=self.saveFile)
        self.save_button.pack(side=tk.TOP, anchor=tk.NE) 

        # Draw center
        self.draw_center_rectangle()
        self.circles = []

        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<Button-3>", self.on_right_click)  # Bind right-click event
        self.canvas.bind("<Motion>", self.update_coordinates)  # Bind mouse movement to update coordinates

        # Variables to keep track of the current circle being dragged
        self.current_circle = None
        self.start_x = 0
        self.start_y = 0

    def transform_coords(self, x, y):
        # Transform coordinates
        new_x = (x - self.center_x) / self.scale
        new_y = (self.center_y - y) / self.scale
        return new_x, new_y

    def inverse_transform_coords(self, x, y):
        # Transform cucoordinates back
        new_x = x * self.scale + self.center_x
        new_y = self.center_y - y * self.scale
        return new_x, new_y

    def saveFile(self):
        filename = "URDFS/obstacles/circle_locations.txt"
        with open(filename, "w+") as f:
            for i in self.circles:
                x, y = i["cx"], i["cy"]
                radius = i["radius"] / self.scale
                f.write(f'{x} {y} {radius}\n')

    def draw_center_rectangle(self):
        rect_width = 0.5  #scaled
        rect_height = 2.0  # sclaed
        x0, y0 = self.inverse_transform_coords(-rect_width / 2, rect_height / 2 + 4)
        x1, y1 = self.inverse_transform_coords(rect_width / 2, -rect_height / 2 + 4)
        self.canvas.create_rectangle(x0, y0, x1, y1, fill="red", outline="")

    def on_click(self, event):
        x, y = event.x, event.y
        transformed_x, transformed_y = self.transform_coords(x, y)
        for circle in self.circles:
            cx, cy, radius, item_id, dot_id = circle['cx'], circle['cy'], circle['radius'], circle['id'], circle['dot_id']
            canvas_cx, canvas_cy = self.inverse_transform_coords(cx, cy)
            if (canvas_cx - x)**2 + (canvas_cy - y)**2 <= radius**2:
                self.current_circle = circle
                self.start_x, self.start_y = x, y
                return
        
        # If no circle clicked
        radius = 20
        color = random.choice(["green"])
        item_id = self.canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill=color, outline="")
        dot_id = self.canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill="black", outline="")
        self.circles.append({'cx': transformed_x, 'cy': transformed_y, 'radius': radius, 'id': item_id, 'dot_id': dot_id})
        self.current_circle = None

    def on_drag(self, event):
        if self.current_circle:
            x, y = event.x, event.y
            dx = x - self.start_x
            dy = y - self.start_y
            new_radius = self.current_circle['radius'] + int((dx**2 + dy**2)**0.5)

            cx, cy = self.inverse_transform_coords(self.current_circle['cx'], self.current_circle['cy'])
            self.canvas.coords(self.current_circle['id'],
                               cx - new_radius,
                               cy - new_radius,
                               cx + new_radius,
                               cy + new_radius)

            self.current_circle['radius'] = new_radius
            self.start_x, self.start_y = x, y

    def on_release(self, event):
        self.current_circle = None

    def on_right_click(self, event):
        x, y = event.x, event.y
        for circle in self.circles:
            cx, cy, radius, item_id, dot_id = circle['cx'], circle['cy'], circle['radius'], circle['id'], circle['dot_id']
            canvas_cx, canvas_cy = self.inverse_transform_coords(cx, cy)
            if (canvas_cx - x)**2 + (canvas_cy - y)**2 <= radius**2:
                self.canvas.delete(item_id)
                self.canvas.delete(dot_id)
                self.circles.remove(circle)
                return

    def update_coordinates(self, event):
        x, y = event.x, event.y
        transformed_x, transformed_y = self.transform_coords(x, y)
        self.coord_label.config(text=f"x: {transformed_x:.2f}, y: {transformed_y:.2f}")


root = tk.Tk()
root.title("Create taskspace")
app = CircleDrawer(root)
root.mainloop()
