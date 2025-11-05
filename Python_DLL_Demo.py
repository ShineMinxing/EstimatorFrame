"""
% ===========================================================================
% Filename: Python_DLL_Demo.py
% Description:
%
% A demo for demonstrating how to call estimator port in DLL form from Python
% using ctypes on Windows.
%
% Estimator\EstimatorPortN.dll is required at runtime (the .lib is not used by Python).
%
% Mainly use .vscode\tasks.json and CMake targets to build Estimator\EstimatorPortN.dll,
% then run this script with Python 3 (requires NumPy).
%
% Initial version: Minxing Sun
% Unit: UCAS, Institute of Optics And Electronics, Lab 1, Class of 2020
% Email: 401435318@qq.com
% Date: November 5, 2025
%
% Updates:
% Unit:
% Email:
% Date:
% ===========================================================================
"""


import ctypes
from ctypes import *
import os
import sys
import time
import numpy as np

# Constants
INPUT_PATH = "ObservationData/DoubleReflectorTrace/Trace1000.txt"
OUTPUT_PATH = "EstimationResult"
DLL_PATH = "Estimator/EstimatorPortN.dll"
DATA_ROWS = 1000
READ_DATA_COLUMNS = 3
WRITE_DATA_COLUMNS = 5
Observation_Dimension = 2
State_Dimension = 4

# Load DLL
try:
    estimator_dll = cdll.LoadLibrary(DLL_PATH)
except Exception as e:
    print(f"Unable to load EstimatorPortN.dll from {DLL_PATH}")
    sys.exit(1)

# Define EstimatorPortN struct
class EstimatorPortN(Structure):
    pass

# Function pointer types
StateTransitionEquationType = CFUNCTYPE(None, POINTER(c_double), c_double, POINTER(c_double), POINTER(EstimatorPortN))
ObservationEquationType = CFUNCTYPE(None, POINTER(c_double), POINTER(c_double), POINTER(EstimatorPortN))
PredictionEquationType = CFUNCTYPE(None, POINTER(c_double), c_double, POINTER(c_double), POINTER(EstimatorPortN))
EstimatorPortFuncType = CFUNCTYPE(None, POINTER(c_double), POINTER(c_double), POINTER(EstimatorPortN))
EstimatorPortTerminationType = CFUNCTYPE(None, POINTER(EstimatorPortN))

# Define _fields_ for EstimatorPortN
EstimatorPortN._fields_ = [
    ('PortName', c_char_p),
    ('PortIntroduction', c_char_p),
    ('Nx', c_int),
    ('Nz', c_int),
    ('PredictStep', c_int),
    ('Intervel', c_double),
    ('PredictTime', c_double),
    ('EstimatedState', POINTER(c_double)),
    ('PredictedState', POINTER(c_double)),
    ('CurrentObservation', POINTER(c_double)),
    ('PredictedObservation', POINTER(c_double)),
    ('Matrix_F', POINTER(c_double)),
    ('Matrix_G', POINTER(c_double)),
    ('Matrix_B', POINTER(c_double)),
    ('Matrix_H', POINTER(c_double)),
    ('Matrix_P', POINTER(c_double)),
    ('Matrix_Q', POINTER(c_double)),
    ('Matrix_R', POINTER(c_double)),
    ("Int_Par", POINTER(c_int)),
    ("Double_Par", POINTER(c_double)),
    ('StateTransitionEquation', StateTransitionEquationType),
    ('ObservationEquation', ObservationEquationType),
    ('PredictionEquation', PredictionEquationType),
    ('EstimatorPort', EstimatorPortFuncType),
    ('EstimatorPortTermination', EstimatorPortTerminationType),
]

# Define initialization
estimator_dll.StateSpaceModel1_Initialization.argtypes = [POINTER(EstimatorPortN)]
estimator_dll.StateSpaceModel1_Initialization.restype = None
# Define estiamtor port
estimator_dll.StateSpaceModel1_EstimatorPort.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(EstimatorPortN)]
estimator_dll.StateSpaceModel1_EstimatorPort.restype = None
# Define termination
estimator_dll.StateSpaceModel1_EstimatorPortTermination.argtypes = [POINTER(EstimatorPortN)]
estimator_dll.StateSpaceModel1_EstimatorPortTermination.restype = None

# Create an instance of EstimatorPortN
state_space_model1 = EstimatorPortN()

# Initialize the estimator
estimator_dll.StateSpaceModel1_Initialization(byref(state_space_model1))

# Read data from file
try:
    ReadFileData = np.loadtxt(INPUT_PATH)
except Exception as e:
    print(f"Unable to open file: {INPUT_PATH}")
    sys.exit(1)

if ReadFileData.shape != (DATA_ROWS, READ_DATA_COLUMNS):
    print(f"Data shape mismatch. Expected {(DATA_ROWS, READ_DATA_COLUMNS)}, got {ReadFileData.shape}")
    sys.exit(1)

EstimatorObservation = ReadFileData[:, 1:3]  # columns 1 and 2
TimeData = ReadFileData[:, 0]  # column 0

# Initialize arrays
EstimatedState = np.zeros((DATA_ROWS, State_Dimension), dtype=np.float64)
WriteFileData = np.zeros((DATA_ROWS, WRITE_DATA_COLUMNS), dtype=np.float64)

# Processing loop
for i in range(DATA_ROWS):
    obs_ptr = EstimatorObservation[i].ctypes.data_as(POINTER(c_double))
    state_ptr = EstimatedState[i].ctypes.data_as(POINTER(c_double))
    estimator_dll.StateSpaceModel1_EstimatorPort(obs_ptr, state_ptr, byref(state_space_model1))
    # Write time and estimated state to WriteFileData
    WriteFileData[i, 0] = TimeData[i]
    WriteFileData[i, 1:] = EstimatedState[i]

# Terminate the estimator
estimator_dll.StateSpaceModel1_EstimatorPortTermination(byref(state_space_model1))

# Write data to output file
if not os.path.exists(OUTPUT_PATH):
    os.makedirs(OUTPUT_PATH)

timestamp = time.strftime("%Y%m%d%H%M%S", time.localtime())
output_file_path = os.path.join(OUTPUT_PATH, f"EstimationResult_{timestamp}.txt")

try:
    with open(output_file_path, 'w') as f:
        for row in WriteFileData:
            f.write(' '.join(f"{val:.6f}" for val in row) + '\n')
    print(f"Estimation data has been written to {output_file_path}")
except Exception as e:
    print(f"Unable to write to file: {output_file_path}")
    sys.exit(1)

print("Program terminated...")
