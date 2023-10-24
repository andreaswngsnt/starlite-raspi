import tkinter as tk
import cv2
from PIL import Image, ImageTk
import webbrowser
from geopy.geocoders import Nominatim

class CameraApp:
    def __init__(self, window, window_title):
        self.window = window
        self.window.title(window_title)

        self.video_source = 0  # Use the default camera (usually the built-in webcam)
        self.vid = cv2.VideoCapture(self.video_source)

        self.canvas = tk.Canvas(window, width=self.vid.get(3), height=self.vid.get(4))
        self.canvas.grid(row=0, column=0, columnspan=2)

        # Create a frame for camera buttons
        camera_button_frame = tk.Frame(window)
        camera_button_frame.grid(row=1, column=0, padx=20, pady=10)

        self.btn_capture = tk.Button(camera_button_frame, text="Start Camera", width=15, command=self.start_camera)
        self.btn_capture.pack(pady=5)

        self.btn_stop = tk.Button(camera_button_frame, text="Stop Camera", width=15, command=self.stop_camera)
        self.btn_stop.pack(pady=5)

        # Create a frame for the "Location" button below the camera buttons
        location_button_frame = tk.Frame(window)
        location_button_frame.grid(row=2, column=0, padx=20, pady=10)

        self.btn_location = tk.Button(location_button_frame, text="Location", width=15)  # Removed the command
        self.btn_location.grid(row=0, column=0, pady=5)

        # Create a frame for the location input and enter button
        location_frame = tk.Frame(window)
        location_frame.grid(row=3, column=0, padx=20, pady=10)

        # Add a label to indicate the purpose of the textbox
        destination_label = tk.Label(location_frame, text="Destination:", width=15)
        destination_label.grid(row=0, column=0, padx=5)

        self.location_entry = tk.Entry(location_frame, width=20)
        self.location_entry.grid(row=0, column=1, padx=5)

        self.btn_enter = tk.Button(location_frame, text="Enter", width=15, command=self.show_location_on_map)
        self.btn_enter.grid(row=0, column=2, padx=5)

        self.is_capturing = False
        self.update()

        self.geolocator = Nominatim(user_agent="camera_app")

    def start_camera(self):
        self.is_capturing = True
        self.btn_capture["state"] = "disabled"
        self.btn_stop["state"] = "active"
        self.vid = cv2.VideoCapture(self.video_source)

    def stop_camera(self):
        self.is_capturing = False
        self.btn_capture["state"] = "active"
        self.btn_stop["state"] = "disabled"
        self.vid.release()
        self.canvas.delete("all")  # Remove the camera frame from the canvas

    def show_location_on_map(self):
        address = self.location_entry.get()
        if address:
            location = self.geolocator.geocode(address)
            if location:
                lat, lon = location.latitude, location.longitude
                map_url = f"https://www.google.com/maps?q={lat},{lon}&t=k"  # Satellite map URL
                webbrowser.open(map_url, new=2)

    def update(self):
        if self.is_capturing:
            ret, frame = self.vid.read()
            if ret:
                self.photo = ImageTk.PhotoImage(image=Image.fromarray(frame))
                self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
        self.window.after(10, self.update)

if __name__ == "__main__":
    root = tk.Tk()
    app = CameraApp(root, "Camera Application")
    root.mainloop()
