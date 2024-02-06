This code is an example of how to extract more information about a model when all you have is a .mot file as a result of an inverse kinematics solve.
It reads in a .mot file which contains OpenSim model 'coordinates', and creates a .sto file which contains 'states'. 
The model is then matched with the states and the orientation of each model body can be extracted.
The orientation (in quaternions) of each body is then saved in a .csv file for further use. 
Also included is an example of how to calculate humero-thoracic joint angles from the resultant thorax and humerus body orientations. 
