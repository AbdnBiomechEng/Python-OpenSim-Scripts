# This script takes a .mot file containing time-series model 'coordinates' (which is the format of OpenSim's IK results)
# creates an .sto file containing time-series 'states', then combines the model with the states so that model attributes
# can be extracted - e.g. body orientations.

# Notes - you'll see errors about missing .vtp files - these are just geometry files which we don't need right now

from functions import *

# Files that need to be in folder:
    # functions.py
    # analyse settings template .xml file
    # model file (the same one which was used for the IK)
    # .mot coordinates file (IK results)

""" SETTINGS """

analyze_settings_template_file = "Analysis_Settings.xml"
model_file = "das3.osim"
coord_file = "OMC_IK_results.mot"

results_path = r"C:\Users\r03mm22\Documents\Protocol_Testing\ScriptForBradley"   # Folder where you want the analyze tool to write the states file
start_time = 0  # Time interval you want to analyze (time stamp, in whatever units are in the .mot file)
end_time = 20


""" MAIN """

# Create states file from coordinates file
create_states_file_from_coordinates_file(analyze_settings_template_file, model_file, coord_file,
                                         results_path, start_time, end_time)

# Get body orientations from states file and save to csv
get_body_quats_from_states("OMC_StatesReporter_states.sto", model_file, results_path)

# Read in the new csv file
thorax_OMC, humerus_OMC, radius_OMC = read_in_quats(start_time, end_time, file_name=results_path + r"\Body_Oris.csv", trim_bool=True)

# Get humero-thoracic joint angles from the thorax and humerus orientations
eul_1, eul_2, eul_3 = get_JA_euls_from_quats(thorax_OMC, humerus_OMC, eul_seq="YZY")


