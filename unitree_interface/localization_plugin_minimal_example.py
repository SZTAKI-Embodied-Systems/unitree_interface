# This code is a minimal working example of how to integrate a localization plugin
import time
from unitree_interface.localization import load_plugin

# load localization plugin configuration
plugin_config_dict = {
    "localization_plugin": "motioncapture",
    "localization_timeout_sec": 0.1
}

localization_plugin = load_plugin(plugin_config_dict["localization_plugin"])
localization_plugin = localization_plugin(plugin_config_dict)
localization_timeout_sec = plugin_config_dict["localization_timeout_sec"]

def on_low_state_callback(): # ---> called when new low state is received from the robot
    localization_output = localization_plugin.get_state()
    if localization_output is None:
        return
    now = time.time()
    localization_time = localization_plugin.get_last_update_time()
    if now - localization_time > localization_timeout_sec:
        print(f"[WARNING] Localization plugin timeout: {now - localization_time} s")
        return