"""
% ===========================================================================
% Filename: Python_SO_Demo.py
% Description:
%
% A demo for demonstrating how to call estimator port in shared object (.so) form
% from Python using ctypes on Linux.
%
% Estimator/EstimatorPortN.so is required at runtime.
%
% Mainly use .vscode/tasks.json and CMake targets to build Estimator/EstimatorPortN.so,
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


import os
import sys
import time
from pathlib import Path
import numpy as np
from ctypes import *

# ---------------------------
# Paths (robust to cwd)
# ---------------------------
ROOT = Path(__file__).resolve().parent
INPUT_PATH  = ROOT / "ObservationData" / "DoubleReflectorTrace" / "Trace1000.txt"
OUTPUT_DIR  = ROOT / "EstimationResult"
SO_PATH     = ROOT / "Output" / "EstimatorPortN.so"

# ---------------------------
# Constants (must match model)
# ---------------------------
DATA_ROWS = 1000
READ_DATA_COLUMNS = 3
WRITE_DATA_COLUMNS = 5
Observation_Dimension = 2
State_Dimension = 4

# ---------------------------
# Load shared library (.so)
# ---------------------------
if not SO_PATH.exists():
    print(f"[Error] Shared library not found: {SO_PATH}")
    sys.exit(1)

try:
    estimator_so = CDLL(str(SO_PATH))
except Exception as e:
    print(f"[Error] Unable to load EstimatorPortN.so from {SO_PATH}\n{e}")
    sys.exit(1)

# ---------------------------
# Define EstimatorPortN struct (must match header)
# ---------------------------
class EstimatorPortN(Structure):
    pass

StateTransitionEquationType = CFUNCTYPE(None, POINTER(c_double), c_double, POINTER(c_double), POINTER(EstimatorPortN))
ObservationEquationType     = CFUNCTYPE(None, POINTER(c_double), POINTER(c_double), POINTER(EstimatorPortN))
PredictionEquationType      = CFUNCTYPE(None, POINTER(c_double), c_double, POINTER(c_double), POINTER(EstimatorPortN))
EstimatorPortFuncType       = CFUNCTYPE(None, POINTER(c_double), POINTER(c_double), POINTER(EstimatorPortN))
EstimatorPortTerminationType= CFUNCTYPE(None, POINTER(EstimatorPortN))

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
    ('ObservationEquation',     ObservationEquationType),
    ('PredictionEquation',      PredictionEquationType),
    ('EstimatorPort',           EstimatorPortFuncType),
    ('EstimatorPortTermination',EstimatorPortTerminationType),
]

# ---------------------------
# Bind C functions
# ---------------------------
# void StateSpaceModel1_Initialization(struct EstimatorPortN *);
estimator_so.StateSpaceModel1_Initialization.argtypes = [POINTER(EstimatorPortN)]
estimator_so.StateSpaceModel1_Initialization.restype  = None

# void StateSpaceModel1_EstimatorPort(double *obs, double *state, struct EstimatorPortN *);
estimator_so.StateSpaceModel1_EstimatorPort.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(EstimatorPortN)]
estimator_so.StateSpaceModel1_EstimatorPort.restype  = None

# void StateSpaceModel1_EstimatorPortTermination(struct EstimatorPortN *);
estimator_so.StateSpaceModel1_EstimatorPortTermination.argtypes = [POINTER(EstimatorPortN)]
estimator_so.StateSpaceModel1_EstimatorPortTermination.restype  = None

def main():
    # ---------------------------
    # Read data
    # ---------------------------
    if not INPUT_PATH.exists():
        print(f"[Error] Input file not found: {INPUT_PATH}")
        sys.exit(1)

    try:
        ReadFileData = np.loadtxt(INPUT_PATH, dtype=np.float64)
    except Exception as e:
        print(f"[Error] Unable to read file: {INPUT_PATH}\n{e}")
        sys.exit(1)

    if ReadFileData.shape != (DATA_ROWS, READ_DATA_COLUMNS):
        print(f"[Error] Data shape mismatch. Expected {(DATA_ROWS, READ_DATA_COLUMNS)}, got {ReadFileData.shape}")
        sys.exit(1)

    # observation = columns 1..2 ; time = column 0
    EstimatorObservation = np.ascontiguousarray(ReadFileData[:, 1:3], dtype=np.float64)
    TimeData             = np.ascontiguousarray(ReadFileData[:, 0],   dtype=np.float64)

    # ---------------------------
    # Prepare outputs
    # ---------------------------
    EstimatedState = np.zeros((DATA_ROWS, State_Dimension), dtype=np.float64)
    WriteFileData  = np.zeros((DATA_ROWS, WRITE_DATA_COLUMNS), dtype=np.float64)

    # ---------------------------
    # Create and initialize estimator
    # ---------------------------
    ep = EstimatorPortN()
    estimator_so.StateSpaceModel1_Initialization(byref(ep))

    # ---------------------------
    # Process rows
    # ---------------------------
    for i in range(DATA_ROWS):
        obs_ptr   = EstimatorObservation[i].ctypes.data_as(POINTER(c_double))
        state_ptr = EstimatedState[i].ctypes.data_as(POINTER(c_double))
        estimator_so.StateSpaceModel1_EstimatorPort(obs_ptr, state_ptr, byref(ep))

        WriteFileData[i, 0]  = TimeData[i]
        WriteFileData[i, 1:] = EstimatedState[i]

    # ---------------------------
    # Termination
    # ---------------------------
    estimator_so.StateSpaceModel1_EstimatorPortTermination(byref(ep))

    # ---------------------------
    # Write output
    # ---------------------------
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d%H%M%S", time.localtime())
    out_path  = OUTPUT_DIR / f"EstimationResult_{timestamp}.txt"

    try:
        with open(out_path, 'w', encoding='utf-8') as f:
            for row in WriteFileData:
                f.write(' '.join(f"{val:.6f}" for val in row) + '\n')
        print(f"[OK] Estimation data has been written to {out_path}")
    except Exception as e:
        print(f"[Error] Unable to write file: {out_path}\n{e}")
        sys.exit(1)

    print("[Done] Program terminated.")

if __name__ == "__main__":
    main()
