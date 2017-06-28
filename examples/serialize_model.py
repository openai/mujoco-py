"""
# Serialization/Deserialization of Models

Sometimes its useful to send a mujoco model over the network, or save it
to a file with all assets embedded.
"""
import mujoco_py

# The binary MJB format is preferable, since it includes assets like
# textures and meshes.
model = mujoco_py.load_model_from_path("xmls/claw.xml")
mjb_bytestring = model.get_mjb()
model_from_binary = mujoco_py.load_model_from_mjb(mjb_bytestring)
assert model.nbody == model_from_binary.nbody

# XML is preferable to MJB when readability and backward compatibility are
# important.
xml_string = model.get_xml()
model_from_xml = mujoco_py.load_model_from_xml(xml_string)
assert model.nbody == model_from_xml.nbody
