# Minimal component to trigger the IDF component manager.
#
# The component manager only resolves dependencies from components that have
# BOTH a CMakeLists.txt and an idf_component.yml in the same directory.
# Since ArduPilot uses idf.cmake directly (not project.cmake), none of the
# standard IDF components serve as a trigger. This empty component exists
# solely to provide that trigger so the component manager downloads esp-dsp
# and any future managed dependencies automatically.

idf_component_register()
