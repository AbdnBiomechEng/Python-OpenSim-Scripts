import opensim as osim
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R

# This function uses the opensim analyze tool to create a states file from a coordinates file
def create_states_file_from_coordinates_file(analyze_settings_template_file, model_file, coord_file,
                                             results_path, start_time, end_time):

    # Instantiate a Analyze Tool
    analyze_tool = osim.AnalyzeTool(analyze_settings_template_file)
    analyze_tool.setModelFilename(model_file)
    analyze_tool.setResultsDir(results_path)
    analyze_tool.setCoordinatesFileName(coord_file)
    analyze_tool.setInitialTime(start_time)
    analyze_tool.setFinalTime(end_time)
    analyze_tool.setName("OMC")
    analyze_tool.run()

# This function takes a model body and a state and gets the body's orientation in the ground frame
def get_body_quat(state, body):
    Rot = body.getTransformInGround(state).R()
    quat = Rot.convertRotationToQuaternion()
    output_quat = np.array([quat.get(0), quat.get(1), quat.get(2), quat.get(3)])
    return output_quat


# This function pairs a model with a states file to extract body orientations and saves them in a csv file
def get_body_quats_from_states(states_file, model_file, results_path):

    # Create an opensim time series table from the states file
    states_table = osim.TimeSeriesTable(states_file)

    # Create the model and the bodies
    model = osim.Model(model_file)
    thorax = model.getBodySet().get('thorax')
    humerus = model.getBodySet().get('humerus_r')
    radius = model.getBodySet().get('radius_r')

    # Unlock any locked coordinates in model
    for coord in ['TH_x','TH_y','TH_z','TH_x_trans','TH_y_trans','TH_z_trans',
                  'SC_x','SC_y','SC_z','AC_x','AC_y','AC_z','GH_y','GH_z','GH_yy','EL_x','PS_y']:
        model.getCoordinateSet().get(coord).set_locked(False)

    # Get the states info from the states file
    print("Getting states info from states file...")
    stateTrajectory = osim.StatesTrajectory.createFromStatesTable(model, states_table)
    n_rows = stateTrajectory.getSize()

    # Initiate the system so that the model can actively realise positions based on states
    model.initSystem()

    # Get the orientation of each body of interest
    thorax_quats = np.zeros((n_rows, 4))
    humerus_quats = np.zeros((n_rows, 4))
    radius_quats = np.zeros((n_rows, 4))
    for row in range(n_rows):
        state = stateTrajectory.get(row)
        model.realizePosition(state)
        thorax_quats[row] = get_body_quat(state, thorax)
        humerus_quats[row] = get_body_quat(state, humerus)
        radius_quats[row] = get_body_quat(state, radius)

    # Write all body quats to a csv file
    thorax_quats_df = pd.DataFrame({"Thorax_Q0": thorax_quats[:,0],"Thorax_Q1": thorax_quats[:,1], "Thorax_Q2": thorax_quats[:,2], "Thorax_Q3": thorax_quats[:,3]})
    humerus_quats_df = pd.DataFrame({"Humerus_Q0": humerus_quats[:,0],"Humerus_Q1": humerus_quats[:,1], "Humerus_Q2": humerus_quats[:,2],"Humerus_Q3": humerus_quats[:,3]})
    radius_quats_df = pd.DataFrame({"Radius_Q0": radius_quats[:,0],"Radius_Q1": radius_quats[:,1], "Radius_Q2": radius_quats[:,2],"Radius_Q3": radius_quats[:,3]})
    time_df = pd.DataFrame({"Time": np.asarray(states_table.getIndependentColumn())[:]})

    all_quats_df = pd.concat([time_df, thorax_quats_df, humerus_quats_df, radius_quats_df], axis=1)

    print("Writing orientations file to csv...")

    all_quats_df.to_csv(results_path + r"\Body_Oris.csv", mode='w', encoding='utf-8', na_rep='nan')


# This function reads in the quaternions from the newly created csv file
def read_in_quats(start_time, end_time, file_name, trim_bool):
    with open(file_name, 'r') as file:
        df = pd.read_csv(file, header=0)
    # Trim dataframe
    if trim_bool == True:
        df = df.loc[(df['Time'] >= start_time) & (df['Time'] <= end_time)]
    # Extract separate dfs for each body
    thorax_quats = df.filter(["Thorax_Q0", "Thorax_Q1", "Thorax_Q2", "Thorax_Q3"], axis=1)
    humerus_quats = df.filter(["Humerus_Q0", "Humerus_Q1", "Humerus_Q2", "Humerus_Q3"], axis=1)
    radius_quats = df.filter(["Radius_Q0", "Radius_Q1", "Radius_Q2", "Radius_Q3"], axis=1)
    thorax_quats_np = thorax_quats.to_numpy()
    humerus_quats_np = humerus_quats.to_numpy()
    radius_quats_np = radius_quats.to_numpy()

    return thorax_quats_np, humerus_quats_np, radius_quats_np


# Define a function for quaternion multiplication
def quat_mul(Q0, Q1):
    """
    Multiplies two quaternions.
    Input
    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31)
    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32)
    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)
    """
    w0 = Q0[0]
    x0 = Q0[1]
    y0 = Q0[2]
    z0 = Q0[3]
    w1 = Q1[0]
    x1 = Q1[1]
    y1 = Q1[2]
    z1 = Q1[3]
    # Computer the product of the two quaternions, term by term
    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([Q0Q1_w, Q0Q1_x, Q0Q1_y, Q0Q1_z])
    return final_quaternion


# Calculate quaternion conjugate
def quat_conj(Q0):
    w0 = Q0[0]
    x0 = Q0[1]
    y0 = Q0[2]
    z0 = Q0[3]
    output_quaternion = np.array([w0, -x0, -y0, -z0])
    return output_quaternion


# This function takes quaternion body orientations and outputs the joint euler angles
def get_JA_euls_from_quats(body1_quats, body2_quats, eul_seq):

    n_rows = len(body1_quats)
    eul_1_arr = np.zeros((n_rows))
    eul_2_arr = np.zeros((n_rows))
    eul_3_arr = np.zeros((n_rows))
    for row in range(n_rows):
        joint_Rot = quat_mul(quat_conj(body1_quats[row]), body2_quats[row])  # Calculate joint Rot quat
        joint_scipyR = R.from_quat([joint_Rot[1], joint_Rot[2], joint_Rot[3], joint_Rot[0]])    # In scalar last format
        joint_eul = joint_scipyR.as_euler(eul_seq, degrees=True)    # Get euler angles
        eul_1_arr[row], eul_2_arr[row], eul_3_arr[row] = joint_eul[0], joint_eul[1], joint_eul[2]

    return eul_1_arr, eul_2_arr, eul_3_arr

