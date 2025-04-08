** Note to supervisors ** 

For progress review, see latest files:  
- (ARDUINO) Tensegrity_test_highLevel_neighb_REC ==> with selfishness function to regulate lower-level behaviour & global check function (allow or disable selfishness) based on internal goal
****************************************************************************************************************************************
- (PYTHON) Hardware_camera_setup_ARDUINO(error) ==> send coordinates/error to Arduino script "Camera_test_printing", reads Serial Arduino monitor and save data into Excel file
- (ARDUINO) Camera_test_printing ==> receives coordinates/error encap to target from python & prints them/it in Serial monitor
****************************************************************************************************************************************
- (PYTHON) Hardware_Pyth-Ard_EXPERIMENTS ==> sends error to Arduino ==> Arduino prints ALL DATA from robot ==> all data read by python & saved in Excel file 
- (ADRUINO) Tensegrity_test_highLevel_CAMERA_TESTS ==> getting & logging error (endcap to target) from Arduino Script "Hardware_Pyth-Ard_EXPERIMENTS"

