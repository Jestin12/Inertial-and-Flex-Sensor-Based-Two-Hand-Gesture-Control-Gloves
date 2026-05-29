# Inertial-and-Flex-Sensor-Based-Two-Hand-Gesture-Control-gloves-Thesis
Data acquisition and ML pipelines for gesture control gloves

This github repository contains code for three separate systems:
- Data collation and transmission code for the intertial and flex sensor gloves -> Left_COM_Sync & Right_COM_Sync
- The data receiver and CSV compilation code for the receiving host PC -> WIFI_Pipelined_Receiver_w_Error_Detection.py
- The data visualiser, for graphically observing data from all data channels -> glove_data_visualiser_v4.ipynb
- The ML training and testing pipeline for static dual hand gestures -> glove_ml_explorer.ipynb
- The ML training and testing pipeline for finger pose -> FingerPoseClassification.ipynb
- The ML training and testing pipeline for dynamic gestures (uses deep learning models, could be used for static) -> 1D CNN - Dynamic Gesture

To run the gloves, 
1. Edit the WIFI settings in Net.cpp in both Left_COM_Sync & Right_COM_Sync to match the hotspot of the host computer.
2. Edit the WIFI_Pipelined_Receiver_w_Error_Detection.py to have an output directory of your choosing.
3. Upload the Left_COM_Sync.ino & Right_COM_Sync.ino files to the ESP32 S3 Zero mcus of the left and right gloves.
4. Power on each glove and keep stationary until the flashing green LED turns off
5. Run the WIFI_Pipelined_Receiver_w_Error_Detection.py, terminal will prompt when the gloves are connected to the TCP server ports
6. Press Enter to begin recording a sample when the code prompts.
7. Once the recording is complete a .csv file with the code will be save to the output directory chosen and the code will exit

Run the WIFI_Pipelined_Receiver_w_Error_Detection.py script again to keep recording readings. Repeating steps 5 - 7.
