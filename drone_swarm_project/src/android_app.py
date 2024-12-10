from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.dropdown import DropDown
from kivy.uix.button import Button
from kivy_garden.mapview import MapView, MapMarker
from kivy.uix.popup import Popup
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.scrollview import ScrollView
from plyer import gps, accelerometer
from kivy.clock import Clock
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.uix.gridlayout import GridLayout
from kivy.uix.scrollview import ScrollView
from kivy.uix.relativelayout import RelativeLayout

# Import Config Values from config.py
from config import (
    TAKEOFF_ALTITUDE,
    HOVER_TIME,
    USER_LATITUDE,
    USER_LONGITUDE,
    OFFSET_DISTANCE,
    ORBIT_AROUND_USER,
    REVOLVE_OFFSET_DISTANCE,
    ANGLE_OFFSET,
    REVOLVE_SPEED,
    ROTATE_TRIANGLE_FORMATION,
    SWAP_POSITIONS,
    SWAP_POSITION_SPEED,
    DRONE_CONNECTIONS,
)


class DroneControlApp(App):
    def build(self):
        return MainLayout()


class MainLayout(FloatLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Initialize Config Values
        self.takeoff_altitude = TAKEOFF_ALTITUDE
        self.hover_time = HOVER_TIME
        self.user_latitude = USER_LATITUDE
        self.user_longitude = USER_LONGITUDE
        self.offset_distance = OFFSET_DISTANCE
        self.orbit_around_user = ORBIT_AROUND_USER
        self.revolve_offset_distance = REVOLVE_OFFSET_DISTANCE
        self.angle_offset = ANGLE_OFFSET
        self.revolve_speed = REVOLVE_SPEED
        self.rotate_triangle_formation = ROTATE_TRIANGLE_FORMATION
        self.swap_positions = SWAP_POSITIONS
        self.swap_position_speed = SWAP_POSITION_SPEED
        self.drone_connections = list(DRONE_CONNECTIONS.values())

        # Map View
        self.map = MapView(zoom=19, lat=0, lon=0)
        self.add_widget(self.map)

        # User Marker
        self.user_marker = MapMarker(lat=0, lon=0, source="user_marker.png")
        self.map.add_marker(self.user_marker)

        # Initialize Drone Markers
        self.drone_markers = {}
        self.simulated_drones = {
            "Drone1": {"latitude": -35.3631673, "longitude": 149.1652317},
            "Drone2": {"latitude": -35.3631773, "longitude": 149.1652417},
            "Drone3": {"latitude": -35.3631723, "longitude": 149.1652267},
        }

        # Add Drone Markers
        for drone_id, location in self.simulated_drones.items():
            self.add_drone_marker(drone_id, location["latitude"], location["longitude"])

        try:
            gps.configure(on_location=self.update_location, on_status=self.gps_status)
            gps.start()
        except NotImplementedError:
            print("GPS not available. Using simulated coordinates.")
            self.simulated_location = {'latitude': -35.3631723, 'longitude': 149.1652367}
            self.update_location(self.simulated_location)


        # Initialize Accelerometer Safely
        try:
            accelerometer.enable()
            self.accelerometer_enabled = True
            Clock.schedule_interval(
                self.read_accelerometer, 1 / 10
            )  # Updates every 0.1 seconds
        except NotImplementedError:
            print("Accelerometer not supported on this platform.")
            self.accelerometer_enabled = False

        # Add Settings Button (Top-Left)
        settings_button = Button(
            text="Settings",
            size_hint=(None, None),
            size=(100, 50),
            pos_hint={"x": 0.02, "y": 0.9},
        )  # Top-left position
        settings_button.bind(on_press=self.open_settings)
        self.add_widget(settings_button)

        # Create Button Layout
        bottom_layout = BoxLayout(
            orientation="vertical",
            size_hint=(0.2, 0.3),
            pos_hint={"x": 0.02, "y": 0.02},
        )

        # Mode Button (Top)
        self.dropdown = DropDown()
        self.mode_button = Button(text="Normal", size_hint=(1, 0.33))
        self.mode_button.bind(on_release=self.dropdown.open)
        modes = ["Normal", "Swap Positions", "Orbit", "Rotate Triangle"]
        for mode in modes:
            btn = Button(text=mode, size_hint_y=None, height=44)
            btn.bind(on_release=lambda btn: self.select_mode(btn.text))
            self.dropdown.add_widget(btn)
        bottom_layout.add_widget(self.mode_button)

        # Takeoff/Land Button (Middle)
        self.takeoff_land_button = Button(text="Take Off", size_hint=(1, 0.33))
        self.takeoff_land_button.bind(on_press=self.toggle_takeoff_land)
        bottom_layout.add_widget(self.takeoff_land_button)

        # Connect Button (Bottom)
        self.connect_button = Button(text="Connect", size_hint=(1, 0.33))
        self.connect_button.bind(on_press=self.connect_to_drones)
        bottom_layout.add_widget(self.connect_button)

        self.add_widget(bottom_layout)

    def read_accelerometer(self, dt):
        """
        Reads accelerometer data only if enabled and prints values.
        """
        if not self.accelerometer_enabled:
            return  # Skip if accelerometer isn't supported

        try:
            accel_data = accelerometer.acceleration
            if accel_data != (None, None, None):
                print(
                    f"Accelerometer Data: X={accel_data[0]:.2f}, Y={accel_data[1]:.2f}, Z={accel_data[2]:.2f}"
                )
        except Exception as e:
            print(f"Error reading accelerometer: {e}")
            Clock.unschedule(self.read_accelerometer)  # Stop further readings

    def add_drone_marker(self, drone_id, lat, lon):
        """
        Adds a marker for a drone on the map.
        """
        marker = MapMarker(
            lat=lat, lon=lon, source="drone_marker.png"
        )  # Replace with your drone icon
        self.drone_markers[drone_id] = marker
        self.map.add_marker(marker)
        print(f"Added marker for {drone_id} at Lat {lat}, Lon {lon}")

    def update_drone_location(self, drone_id, lat, lon):
        """
        Updates the location of a drone marker.
        """
        if drone_id in self.drone_markers:
            marker = self.drone_markers[drone_id]
            marker.lat = lat
            marker.lon = lon
            print(f"Updated {drone_id} to Lat {lat}, Lon {lon}")
        else:
            print(f"Drone ID {drone_id} not found. Adding a new marker.")
            self.add_drone_marker(drone_id, lat, lon)

    def toggle_takeoff_land(self, instance):
        if instance.text == "Take Off":
            instance.text = "Land"
            print("Command: Take Off")
        else:
            instance.text = "Take Off"
            print("Command: Land")

    def select_mode(self, mode):
        # Update the button text to the selected mode
        self.mode_button.text = mode
        print(f"Mode selected: {mode}")
        # Close the dropdown after selection
        self.dropdown.dismiss()


    def open_settings(self, instance):
        """
        Displays the Settings Popup with a scrollable grid layout and the Close button at the top-right.
        """
        # Create the Main Content Layout
        content = FloatLayout()

        # Scrollable content for configuration fields
        scroll_view = ScrollView(size_hint=(1, 0.8), pos_hint={'x': 0, 'y': 0.1})

        # Create a GridLayout for configuration fields
        settings_grid = GridLayout(cols=2, size_hint_y=None, padding=10, spacing=10)
        settings_grid.bind(minimum_height=settings_grid.setter("height"))

        # Ensure drone_connections is a list and validate its length
        while len(self.drone_connections) < 3:
            self.drone_connections.append("")

        # Configuration Fields
        config_fields = {
            "Takeoff Altitude": str(self.takeoff_altitude),
            "Hover Time": str(self.hover_time),
            "User Latitude": str(self.user_latitude),
            "User Longitude": str(self.user_longitude),
            "Offset Distance": str(self.offset_distance),
            "Revolve Speed": str(self.revolve_speed),
            "Drone 1 Connection": self.drone_connections[0],
            "Drone 2 Connection": self.drone_connections[1],
            "Drone 3 Connection": self.drone_connections[2],
        }

        # Store input widgets for later access
        self.config_inputs = {}

        for label, value in config_fields.items():
            settings_label = Label(text=f"{label}:", size_hint=(0.4, None), height=40)
            settings_input = TextInput(text=value, multiline=False, size_hint=(0.6, None), height=40)
            self.config_inputs[label] = settings_input
            settings_grid.add_widget(settings_label)
            settings_grid.add_widget(settings_input)

        # Add the settings grid to the scroll view
        scroll_view.add_widget(settings_grid)

        # Add ScrollView to the Content
        content.add_widget(scroll_view)

        # Close Button in Top-Right Using RelativeLayout
        relative_layout = RelativeLayout(size_hint=(1, 1))
        close_button = Button(text="Close", size_hint=(None, None), size=(50, 35), pos_hint={'right': 1.15, 'top': 1.27})
        close_button.bind(on_press=lambda x: popup.dismiss())
        relative_layout.add_widget(close_button)
        content.add_widget(relative_layout)

        # Save Button at the Bottom
        save_button = Button(text="Save", size_hint=(1, 0.1), pos_hint={'x': 0, 'y': 0})
        save_button.bind(on_press=self.save_settings)
        content.add_widget(save_button)

        # Create and Open the Popup
        popup = Popup(title="Drone Settings", content=content, size_hint=(0.8, 0.8), auto_dismiss=False)
        popup.open()

    def save_settings(self, instance):
        """
        Save updated settings from the popup inputs.
        """
        try:
            self.takeoff_altitude = float(self.config_inputs["Takeoff Altitude"].text)
            self.hover_time = float(self.config_inputs["Hover Time"].text)
            self.user_latitude = float(self.config_inputs["User Latitude"].text)
            self.user_longitude = float(self.config_inputs["User Longitude"].text)
            self.offset_distance = float(self.config_inputs["Offset Distance"].text)
            self.revolve_speed = float(self.config_inputs["Revolve Speed"].text)

            # Update all three drone connections
            self.drone_connections[0] = self.config_inputs["Drone 1 Connection"].text
            self.drone_connections[1] = self.config_inputs["Drone 2 Connection"].text
            self.drone_connections[2] = self.config_inputs["Drone 3 Connection"].text

            print("Settings Saved Successfully!")
        except ValueError:
            print("Error: Invalid Input in Settings!")

    def update_location(self, location):
        """
        Updates the map with the user's current location and centers the map on it.
        """
        lat = location['latitude']
        lon = location['longitude']

        # Center the Map on the User's Location
        self.map.center_on(lat, lon)
        self.user_marker.lat = lat
        self.user_marker.lon = lon
        print(f"User Location Updated: Lat {lat}, Lon {lon}")


    def gps_status(self, status):
        """
        Print GPS status updates.
        """
        print(f"GPS Status: {status}")

    def update_simulated_location(self):
        """
        Updates the map with simulated coordinates.
        """
        lat = self.simulated_location["latitude"]
        lon = self.simulated_location["longitude"]
        self.map.center_on(lat, lon)
        self.user_marker.lat = lat
        self.user_marker.lon = lon
        print(f"Simulated Location: Lat {lat}, Lon {lon}")

        # Simulate drone location updates
        self.update_drone_location(
            "Drone1", -35.3631673, 149.1652317
        )  # Slightly northwest of user
        self.update_drone_location(
            "Drone2", -35.3631773, 149.1652417
        )  # Slightly southeast of user
        self.update_drone_location(
            "Drone3", -35.3631723, 149.1652267
        )  # Slightly southwest of user

    def connect_to_drones(self, instance):
        """
        Handles the Connect button press.
        """
        print("Attempting to connect to drones...")
        # Placeholder for drone connection logic


if __name__ == "__main__":
    DroneControlApp().run()
