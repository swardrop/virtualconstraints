/* Include files */

#include "compassGaitSim_impacts2_sfun.h"
#include "c1_compassGaitSim_impacts2.h"
#include "c2_compassGaitSim_impacts2.h"
#include "c3_compassGaitSim_impacts2.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _compassGaitSim_impacts2MachineNumber_;
real_T _sfTime_;

/* Function Declarations */

/* Function Definitions */
void compassGaitSim_impacts2_initializer(void)
{
}

void compassGaitSim_impacts2_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_compassGaitSim_impacts2_method_dispatcher(SimStruct
  *simstructPtr, unsigned int chartFileNumber, const char* specsCksum, int_T
  method, void *data)
{
  if (chartFileNumber==1) {
    c1_compassGaitSim_impacts2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_compassGaitSim_impacts2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_compassGaitSim_impacts2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_compassGaitSim_impacts2_process_check_sum_call( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
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
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1617887463U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3032934937U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(551550307U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(548926000U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(747839966U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(280891425U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2534368131U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(370560622U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_compassGaitSim_impacts2_get_check_sum(mxArray *plhs[]);
          sf_c1_compassGaitSim_impacts2_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_compassGaitSim_impacts2_get_check_sum(mxArray *plhs[]);
          sf_c2_compassGaitSim_impacts2_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_compassGaitSim_impacts2_get_check_sum(mxArray *plhs[]);
          sf_c3_compassGaitSim_impacts2_get_check_sum(plhs);
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
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2515104692U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4015318512U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3931316417U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2775748259U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_compassGaitSim_impacts2_autoinheritance_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
     case 1:
      {
        if (strcmp(aiChksum, "nZ1H9GjcvK51ln5nzPheRF") == 0) {
          extern mxArray *sf_c1_compassGaitSim_impacts2_get_autoinheritance_info
            (void);
          plhs[0] = sf_c1_compassGaitSim_impacts2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "RGCFwXTHi5fR1ROZu4RdcB") == 0) {
          extern mxArray *sf_c2_compassGaitSim_impacts2_get_autoinheritance_info
            (void);
          plhs[0] = sf_c2_compassGaitSim_impacts2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "4sx5Wy6AzySUTUCuJeXcyD") == 0) {
          extern mxArray *sf_c3_compassGaitSim_impacts2_get_autoinheritance_info
            (void);
          plhs[0] = sf_c3_compassGaitSim_impacts2_get_autoinheritance_info();
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

unsigned int sf_compassGaitSim_impacts2_get_eml_resolved_functions_info( int
  nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
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
     case 1:
      {
        extern const mxArray
          *sf_c1_compassGaitSim_impacts2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_compassGaitSim_impacts2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray
          *sf_c2_compassGaitSim_impacts2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_compassGaitSim_impacts2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_compassGaitSim_impacts2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_compassGaitSim_impacts2_get_eml_resolved_functions_info();
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

void compassGaitSim_impacts2_debug_initialize(void)
{
  _compassGaitSim_impacts2MachineNumber_ = sf_debug_initialize_machine(
    "compassGaitSim_impacts2","sfun",0,3,0,0,0);
  sf_debug_set_machine_event_thresholds(_compassGaitSim_impacts2MachineNumber_,0,
    0);
  sf_debug_set_machine_data_thresholds(_compassGaitSim_impacts2MachineNumber_,0);
}

void compassGaitSim_impacts2_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_compassGaitSim_impacts2_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "compassGaitSim_impacts2", "compassGaitSim_impacts2");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_compassGaitSim_impacts2_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
