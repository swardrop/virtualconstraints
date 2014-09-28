/* Include files */

#include "RobotSim_sfun.h"
#include "RobotSim_sfun_debug_macros.h"
#include "c2_RobotSim.h"
#include "c4_RobotSim.h"
#include "c5_RobotSim.h"
#include "c6_RobotSim.h"
#include "c7_RobotSim.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _RobotSimMachineNumber_;
real_T _sfTime_;

/* Function Declarations */

/* Function Definitions */
void RobotSim_initializer(void)
{
}

void RobotSim_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_RobotSim_method_dispatcher(SimStruct *simstructPtr, unsigned int
  chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==2) {
    c2_RobotSim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_RobotSim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_RobotSim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_RobotSim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==7) {
    c7_RobotSim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_RobotSim_process_check_sum_call( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1059532550U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2696901866U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(4269585384U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(385887293U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1709487349U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1180211943U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2697715833U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1594465925U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 2:
        {
          extern void sf_c2_RobotSim_get_check_sum(mxArray *plhs[]);
          sf_c2_RobotSim_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_RobotSim_get_check_sum(mxArray *plhs[]);
          sf_c4_RobotSim_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_RobotSim_get_check_sum(mxArray *plhs[]);
          sf_c5_RobotSim_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_RobotSim_get_check_sum(mxArray *plhs[]);
          sf_c6_RobotSim_get_check_sum(plhs);
          break;
        }

       case 7:
        {
          extern void sf_c7_RobotSim_get_check_sum(mxArray *plhs[]);
          sf_c7_RobotSim_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3564696471U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(678668628U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1090454852U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3896867807U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3021165162U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2203897525U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3292166474U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(901167127U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_RobotSim_autoinheritance_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(aiChksum, "wZRtCcaNavzdaP3qAm6SKD") == 0) {
          extern mxArray *sf_c2_RobotSim_get_autoinheritance_info(void);
          plhs[0] = sf_c2_RobotSim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "N9foIpSF15CZOMy7NCF37B") == 0) {
          extern mxArray *sf_c4_RobotSim_get_autoinheritance_info(void);
          plhs[0] = sf_c4_RobotSim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "2wPULTJ8RFWc4RyvNDjVx") == 0) {
          extern mxArray *sf_c5_RobotSim_get_autoinheritance_info(void);
          plhs[0] = sf_c5_RobotSim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "QRwLNlHsnWIdMbkwLly82E") == 0) {
          extern mxArray *sf_c6_RobotSim_get_autoinheritance_info(void);
          plhs[0] = sf_c6_RobotSim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 7:
      {
        if (strcmp(aiChksum, "FLdlTXFzb9FNGaYQN2FIpE") == 0) {
          extern mxArray *sf_c7_RobotSim_get_autoinheritance_info(void);
          plhs[0] = sf_c7_RobotSim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_RobotSim_get_eml_resolved_functions_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        extern const mxArray *sf_c2_RobotSim_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_RobotSim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray *sf_c4_RobotSim_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_RobotSim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray *sf_c5_RobotSim_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_RobotSim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray *sf_c6_RobotSim_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_RobotSim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 7:
      {
        extern const mxArray *sf_c7_RobotSim_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c7_RobotSim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_RobotSim_third_party_uses_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(tpChksum, "8QGgNycR9lvTJQC85TqplB") == 0) {
          extern mxArray *sf_c2_RobotSim_third_party_uses_info(void);
          plhs[0] = sf_c2_RobotSim_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "7rClGZudbhgTSss4A051ME") == 0) {
          extern mxArray *sf_c4_RobotSim_third_party_uses_info(void);
          plhs[0] = sf_c4_RobotSim_third_party_uses_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "fdbuUYnHIdbVJzafpRfLDG") == 0) {
          extern mxArray *sf_c5_RobotSim_third_party_uses_info(void);
          plhs[0] = sf_c5_RobotSim_third_party_uses_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "MLWSoyZZt0uoPhvhxFmO7D") == 0) {
          extern mxArray *sf_c6_RobotSim_third_party_uses_info(void);
          plhs[0] = sf_c6_RobotSim_third_party_uses_info();
          break;
        }
      }

     case 7:
      {
        if (strcmp(tpChksum, "YpH97dIRWeMXNPkWssbUE") == 0) {
          extern mxArray *sf_c7_RobotSim_third_party_uses_info(void);
          plhs[0] = sf_c7_RobotSim_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void RobotSim_debug_initialize(struct SfDebugInstanceStruct* debugInstance)
{
  _RobotSimMachineNumber_ = sf_debug_initialize_machine(debugInstance,"RobotSim",
    "sfun",0,5,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,_RobotSimMachineNumber_,0,
    0);
  sf_debug_set_machine_data_thresholds(debugInstance,_RobotSimMachineNumber_,0);
}

void RobotSim_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_RobotSim_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info("RobotSim",
      "RobotSim");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_RobotSim_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
