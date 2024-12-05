from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.dropdown import DropDown
from kivy.uix.button import Button
from kivy_garden.mapview import MapView, MapMarker  # Added MapMarker for the location dot
from kivy.uix.popup import Popup
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.label import Label
from kivy.uix.floatlayout import FloatLayout
from plyer import gps



class DroneControlApp(App):
    def build(self):
        return MainLayout()


class MainLayout(FloatLayout):  # FloatLayout allows overlapping elements
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Map view
        self.map = MapView(zoom=15, lat=0, lon=0)  # Default location
        self.add_widget(self.map)

        # User marker
        self.user_marker = MapMarker(lat=0, lon=0, source="user_marker.png")  # Add a custom icon or dot
        self.map.add_marker(self.user_marker)

        try:
            # Configure and start GPS
            gps.configure(on_location=self.update_location, on_status=self.gps_status)
            gps.start()
        except NotImplementedError:
            print("GPS not available. Using simulated coordinates.")
            # Simulated coordinates for testing
            self.simulated_location = {'latitude': -35.3631723 , 'longitude': 149.1652367}  # New York City coordinates
            self.update_simulated_location()

        # Settings button overlay
        settings_button = Button(text="☰",
                                 size_hint=(None, None),
                                 size=(50, 50),
                                 pos_hint={'x': 0, 'y': 0.9})  # Top-left position
        settings_button.bind(on_press=self.open_settings)
        self.add_widget(settings_button)

        # Bottom controls
        bottom_layout = BoxLayout(size_hint=(1, 0.2), pos_hint={'x': 0, 'y': 0})

        # Takeoff/Land Button
        self.takeoff_land_button = Button(text="Take Off")
        self.takeoff_land_button.bind(on_press=self.toggle_takeoff_land)
        bottom_layout.add_widget(self.takeoff_land_button)

        # Dropdown for modes
        self.dropdown = DropDown()
        self.mode_button = Button(text="Normal", size_hint=(None, None), size=(550, 50))  # Default to "Normal"
        self.mode_button.bind(on_release=self.dropdown.open)

        # Add modes to the dropdown
        modes = ["Normal", "Swap Positions", "Orbit", "Rotate Triangle"]
        for mode in modes:
            btn = Button(text=mode, size_hint_y=None, height=44)
            btn.bind(on_release=lambda btn: self.select_mode(btn.text))
            self.dropdown.add_widget(btn)

        bottom_layout.add_widget(self.mode_button)

        self.add_widget(bottom_layout)

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
        # Create a popup for settings
        content = FloatLayout()

        # Close button at the top right
        close_button = Button(text="Close",
                              size_hint=(None, None),
                              size=(100, 40),
                              pos_hint={'x': 0.85, 'y': 0.85})
        content.add_widget(close_button)

        # Settings label
        label = Label(text="Settings Menu",
                      size_hint=(None, None),
                      size=(200, 50),
                      pos_hint={'x': 0.3, 'y': 0.6})
        content.add_widget(label)

        # Popup configuration
        popup = Popup(title="",
                      content=content,
                      size_hint=(0.8, 0.8),
                      auto_dismiss=False)

        close_button.bind(on_press=popup.dismiss)
        popup.open()

    def update_location(self, location):
        """
        Update the map with the user's current location.
        """
        lat = location['latitude']
        lon = location['longitude']
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
        lat = self.simulated_location['latitude']
        lon = self.simulated_location['longitude']
        self.map.center_on(lat, lon)
        self.user_marker.lat = lat
        self.user_marker.lon = lon
        print(f"Simulated Location: Lat {lat}, Lon {lon}")


if __name__ == '__main__':
    DroneControlApp().run()
