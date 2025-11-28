// If you want to test another Estimator, substitue EstimatorXXXX_ to your estimator

#include "EstimatorPortN.h"

EstimatorPortN StateSpaceModel_Demo_;

void StateSpaceModel_Demo_StateTransitionFunction(double *In_State, double *Out_State, struct EstimatorPortN *estimator) {
    Out_State[0] = In_State[0] + estimator->Intervel * In_State[1];
    Out_State[1] = In_State[1];
    Out_State[2] = In_State[2] + estimator->Intervel * In_State[3];
    Out_State[3] = In_State[3];
}

void StateSpaceModel_Demo_ObservationFunction(double *In_State, double *Out_Observation, EstimatorPortN *estimator) {
    Out_Observation[0] = In_State[0] + estimator->Intervel * In_State[1];
    Out_Observation[1] = In_State[2] + estimator->Intervel * In_State[3];
}

void StateSpaceModel_Demo_PredictionFunction(double *In_State, double *Out_PredictedState, EstimatorPortN *estimator) {
    Out_PredictedState[0] = In_State[0] + estimator->PredictTime * In_State[1];
    Out_PredictedState[1] = In_State[1];
    Out_PredictedState[2] = In_State[2] + estimator->PredictTime * In_State[3];
    Out_PredictedState[3] = In_State[3];
}

EXPORT void StateSpaceModel_Demo_EstimatorPort(double *In_Observation, double In_Observation_Timestamp, struct EstimatorPortN *estimator) {
	for (int i = 0; i < estimator->Nz; i++)
	{
		estimator->CurrentObservation[i] = In_Observation[i];
	}
    estimator->ObservationTimestamp = In_Observation_Timestamp;
    estimator->CurrentTimestamp = In_Observation_Timestamp;
    Estimator1001_Estimation(estimator);
    estimator->StateUpdateTimestamp = estimator->CurrentTimestamp;
}

EXPORT void StateSpaceModel_Demo_EstimatorPortTermination(struct EstimatorPortN *estimator) {
    Estimator1001_Termination();

    free(estimator->PortName);
    estimator->PortName = NULL;
    free(estimator->PortIntroduction);
    estimator->PortIntroduction = NULL;
    free(estimator->EstimatedState);
    estimator->EstimatedState = NULL;
    free(estimator->PredictedState);
    estimator->PredictedState = NULL;
    free(estimator->CurrentObservation);
    estimator->CurrentObservation = NULL;
    free(estimator->PredictedObservation);
    estimator->PredictedObservation = NULL;
    free(estimator->Matrix_F);
    estimator->Matrix_F = NULL;
    free(estimator->Matrix_G);
    estimator->Matrix_G = NULL;
    free(estimator->Matrix_B);
    estimator->Matrix_B = NULL;
    free(estimator->Matrix_H);
    estimator->Matrix_H = NULL;
    free(estimator->Matrix_P);
    estimator->Matrix_P = NULL;
    free(estimator->Matrix_Q);
    estimator->Matrix_Q = NULL;
    free(estimator->Matrix_R);
    estimator->Matrix_R = NULL;
    free(estimator->Int_Par);
    estimator->Int_Par = NULL;
    free(estimator->Double_Par);
    estimator->Double_Par = NULL;
    printf("EstimatorPort terminated.\n");
}


EXPORT void StateSpaceModel_Demo_Initialization(EstimatorPortN *estimator) 
{
    char *PortNameTemp = "Tested Estimator v0.00";
    char *PortIntroductionTemp = "For Reference";

    #define StateSpaceModel_Demo_NX 4
    #define StateSpaceModel_Demo_NZ 2
    #define StateSpaceModel_Demo_PredictStep 2
    #define StateSpaceModel_Demo_Interval 0.005
    #define StateSpaceModel_Demo_PredictTime 0.01
 
    double X0[StateSpaceModel_Demo_NX] = {\
    1,0,1,0\
    };
    double F[StateSpaceModel_Demo_NX*StateSpaceModel_Demo_NX] = {\
    1,StateSpaceModel_Demo_Interval,0,0,\
    0,1,0,0,\
    0,0,1,StateSpaceModel_Demo_Interval,\
    0,0,0,1\
    };
    double G[StateSpaceModel_Demo_NX*StateSpaceModel_Demo_NX] = {\
    1,0,0,0,\
    0,1,0,0,\
    0,0,1,0,\
    0,0,0,1\
    };
    double B[StateSpaceModel_Demo_NX] = {\
    0,0,0,0\
    };
    double H[StateSpaceModel_Demo_NZ*StateSpaceModel_Demo_NX] = {\
    1,0,0,0,\
    0,0,1,0,\
    };
    double P[StateSpaceModel_Demo_NX*StateSpaceModel_Demo_NX] = {\
    1,0,0,0,\
    0,1,0,0,\
    0,0,1,0,\
    0,0,0,1\
    };
    double Q[StateSpaceModel_Demo_NX*StateSpaceModel_Demo_NX] = {\
    1,0,0,0,\
    0,1,0,0,\
    0,0,1,0,\
    0,0,0,1\
    };
    double R[StateSpaceModel_Demo_NZ*StateSpaceModel_Demo_NZ] = {\
    1,0,\
    0,1\
    };
    double Int_Par[1] = {1};
    double Double_Par[1] = {1};

    estimator->Nx = StateSpaceModel_Demo_NX;
    estimator->Nz = StateSpaceModel_Demo_NZ;
    estimator->PredictStep = StateSpaceModel_Demo_PredictStep;
    estimator->Intervel = StateSpaceModel_Demo_Interval;
    estimator->PredictTime = StateSpaceModel_Demo_PredictTime;

    estimator->PortName = (char *)malloc(100);
    estimator->PortIntroduction = (char *)malloc(1000);
    estimator->EstimatedState = (double *)malloc(estimator->Nx * sizeof(double));
    estimator->PredictedState = (double *)malloc(estimator->Nx * sizeof(double));
    estimator->CurrentObservation = (double *)malloc(estimator->Nz * sizeof(double));
    estimator->PredictedObservation = (double *)malloc(estimator->Nz * sizeof(double));
    estimator->Matrix_F = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_G = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_B = (double *)malloc(estimator->Nx * sizeof(double));
    estimator->Matrix_H = (double *)malloc(estimator->Nz * estimator->Nx * sizeof(double));
    estimator->Matrix_P = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_Q = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_R = (double *)malloc(estimator->Nz * estimator->Nz * sizeof(double));
    estimator->Int_Par = (int *)malloc(1 * sizeof(int));
    estimator->Double_Par = (double *)malloc(1 * sizeof(double));
    if (estimator->PortName == NULL || estimator->PortIntroduction == NULL ||\
        estimator->EstimatedState == NULL || estimator->PredictedState == NULL ||\
        estimator->CurrentObservation == NULL || estimator->PredictedObservation == NULL ||\
        estimator->Matrix_F == NULL || estimator->Matrix_G == NULL ||\
        estimator->Matrix_B == NULL || estimator->Matrix_H == NULL ||\
        estimator->Matrix_P == NULL || estimator->Matrix_Q == NULL ||\
        estimator->Matrix_R == NULL || estimator->Int_Par == NULL ||\
        estimator->Double_Par == NULL) {
        perror("Memory allocation failed");
        exit(EXIT_FAILURE);
    }

    strcpy(estimator->PortName, PortNameTemp);
    strcpy(estimator->PortIntroduction, PortIntroductionTemp);
    memcpy(estimator->EstimatedState, X0, estimator->Nx * sizeof(double));
    memcpy(estimator->PredictedState, X0, estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_F, F, estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_G, G, estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_B, B, estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_H, H, estimator->Nz * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_P, P, estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_Q, Q, estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_R, R, estimator->Nz * estimator->Nz * sizeof(double));
    memcpy(estimator->Int_Par, Int_Par, 1 * sizeof(int));
    memcpy(estimator->Double_Par, Double_Par, 1 * sizeof(double));

    // Initiate pointer
    estimator->StateTransitionEquation = StateSpaceModel_Demo_StateTransitionFunction;
    estimator->ObservationEquation = StateSpaceModel_Demo_ObservationFunction;
    estimator->PredictionEquation = StateSpaceModel_Demo_PredictionFunction;

    printf("%s is initialized\n", estimator->PortName);

    Estimator1001_Init(estimator);
}