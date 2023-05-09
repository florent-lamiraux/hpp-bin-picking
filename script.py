# Example script to run in tiago deburring demo.

from hpp.corbaserver import Client, loadServerPlugin
from hpp.corbaserver.bin_picking import Client as bpClient

loadServerPlugin("corbaserver", "bin_picking.so")

s = bpClient()
s.bin_picking.createGripper('pandas/panda2_gripper', q0,
    graph.edges['pandas/panda2_gripper > part/lateral_top | f'], [[]])
