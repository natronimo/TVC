import time
import numpy as np
from flightgear_python.fg_if import FDMConnection
from flightgear_python.fg_if import GuiConnection
import tvc_6dof as sim

def fdm_callback(fdm_data, event_pipe):
    if event_pipe.child_poll():
        child, = event_pipe.child_recv()
        fdm_data['lat_rad'] = child[0]
        fdm_data['lon_rad'] = child[1]
        fdm_data['alt_m'] = child[2]
        fdm_data['phi_rad'] = child[3]
        fdm_data['theta_rad'] = child[4]
        fdm_data['psi_rad'] = child[5]
    return fdm_data

def gui_callback(gui_data, event_pipe):
    lat = gui_data['lat_rad']
    lon = gui_data['lon_rad']
    alt = gui_data['alt_m']
    child_data = (lat, lon, alt)
    event_pipe.child_send(child_data)

if __name__ == '__main__':

    fdm_conn = FDMConnection(fdm_version=24)
    fdm_event_pipe = fdm_conn.connect_rx('localhost', 5501, fdm_callback)
    fdm_conn.connect_tx('localhost', 5502)
    fdm_conn.start()

    gui_conn = GuiConnection(gui_version=8)
    gui_event_pipe = gui_conn.connect_rx('localhost', 5504, gui_callback)
    gui_conn.start()

    pipe_data = gui_event_pipe.parent_recv()
    lat_i, lon_i, alt_i = pipe_data

    R = 6378137
    z_offset = 4.2

    for t in range(np.size(sim.x_history, 1)):

        lat = sim.x_history[1, t]/R + lat_i
        lon = sim.x_history[0, t]/(R*np.cos(lat_i)) + lon_i
        alt = sim.x_history[2, t] + alt_i + z_offset
        phi = sim.x_history[7, t]
        theta = sim.x_history[6, t]
        psi = sim.x_history[8, t]

        fdm_event_pipe.parent_send(([lat, lon, alt, phi, theta, psi],))

        time.sleep(sim.time_step)

    fdm_conn.stop()
    gui_conn.stop()
