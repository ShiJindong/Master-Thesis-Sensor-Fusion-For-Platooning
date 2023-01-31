/*
******************************************************************************
**  CarMaker - Version 11.0
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
**
** Functions
** ---------
**
** Initialization
**
**	User_Init_First ()
**	User_PrintUsage ()
**	User_ScanCmdLine ()
**
**	User_AppLogFilter ()
**
**	User_Init ()
**	User_Register ()
**	User_DeclQuants ()
**
**	User_Param_Add ()
**	User_Param_Get ()
**
**
** Main TestRun Start/End:
**
**	User_TestRun_Start_atBegin ()
**	User_TestRun_Start_atEnd ()
**	User_TestRun_Start_StaticCond_Calc ()
**	User_TestRun_Start_Finalize ()
**	User_TestRun_RampUp ()
**
**	User_TestRun_End_First ()
**	User_TestRun_End ()
**
**
** Main Cycle:
**
**	User_In ()
**
**	User_DrivMan_Calc ()
** 	User_Traffic_Calc ()
**	User_VehicleControl_Calc ()std::cout << "Time: " << RSDSIF.Timestamp
**	User_ApoMsg_Send ()
**
**	User_ShutDown ()
**	User_End ()
**	User_Cleanup ()
**
**
******************************************************************************
*/

#include <Global.h>

#if defined(WIN32)
# include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(XENO)
# include <mio.h>
#endif
# include <ioconf.h>

#include <CarMaker.h>
#include <Car/Vehicle_Car.h>

#include <ADASRP.h>

#include <rbs.h>

#include "IOVec.h"
#include "User.h"

#include "SimNet.h"

/* @@PLUGIN-BEGIN-INCLUDE@@ - Automatically generated code - don't edit! */
/* @@PLUGIN-END@@ */


int UserCalcCalledByAppTestRunCalc = 0;


tUser User;

#define AddV2VCommunicateDelay 1


// Paramters about Frequency
int compute_frequency = 100;                    /* CPU Compute Frequency in Hz */

int imu_frequency = 100;                        /* IMU measurement frequency in Hz */
int wheel_rot_rate_frequency = 25;              /* Wheel Rotation Rate measurement frequency in Hz */
int wheel_steer_angle_frequency = 100;          /* Wheel Steering Angle measurement frequency in Hz */
int v2v_communicate_frequency = 50;             /* Vehicle-to-Vehicle communication frequency in Hz */

int imu_cycle_ms = 1000 / imu_frequency;                                    /* Measurement cycle of IMU in millisecond */
int wheel_rot_rate_cycle_ms = 1000 / wheel_rot_rate_frequency;              /* Measurement cycle of Wheel Rotation Rate Data in millisecond */
int wheel_steer_angle_cycle_ms = 1000 / wheel_steer_angle_frequency;        /* Measurement cycle of Wheel Steering Angle Data in millisecond */
int compute_cycle_ms = 1000 / compute_frequency;                            /* CPU Compute Cycle in millisecond */
int v2v_communicate_cycle_ms = 1000 / v2v_communicate_frequency;            /* Vehicle-to-Vehicle communication cycle in millisecond */
int v2v_communicate_delay_ms = 15;                                          /* Time delay of vehicle-to-vehicle communication in millisecond (generally 10 ~ 30 ms) */

int lidar_cycle_ms = 60;                                                    /* Measurement cycle of Lidar ScanPoint in millisecond */
int camera_cycle_ms = 60;                                                   /* Measurement cycle of Camera in millisecond*/

double compute_cycle_s = static_cast<double>(compute_cycle_ms) / 1000;                  /* CPU Compute Cycle in second */
double v2v_communicate_delay_s = static_cast<double>(v2v_communicate_delay_ms) / 1000;  /* Time delay of vehicle-to-vehicle communication in second */
double CM_cycle_s = 1e-3;                                                               /* CarMaker Cycle in second: by default 1 ms */

// Relative pose: between leadbus rear and followbus front
Eigen::Vector3d groundtruth_rel_pose = Eigen::Vector3d::Zero();                 /* (0): dx, (1): dy, (2): dphi */
Eigen::Vector3d camera_rel_pose = Eigen::Vector3d::Zero();
Eigen::Vector3d lidar_rel_pose = Eigen::Vector3d::Zero();
Eigen::Vector3d fusion_rel_pose = Eigen::Vector3d::Zero();

Eigen::Vector3d camera_rel_pose_estimate_error = Eigen::Vector3d::Zero();       /* (0): estimate error of dx, (1): estimate error of dy, (2): estimate error of dphi */
Eigen::Vector3d lidar_rel_pose_estimate_error = Eigen::Vector3d::Zero();
Eigen::Vector3d fusion_rel_pose_estimate_error = Eigen::Vector3d::Zero();

// Absolute Pose: w.r.t. the vehicle rear (origin of Fr1)
Eigen::Vector3d groundtruth_follow_abs_pose = Eigen::Vector3d::Zero();          /* (0): x, (1): y, (2): phi */
Eigen::Vector3d groundtruth_lead_abs_pose = Eigen::Vector3d::Zero();

Eigen::Vector3d estimated_follow_abs_pose = Eigen::Vector3d::Zero();

Eigen::Vector3d follow_abs_pose_estimate_error = Eigen::Vector3d::Zero();       /* (0): estimate error of x, (1): estimate error of y, (2): estimate error of phi */

// Velocity
double groundtruth_follow_longi_velocity = 0.0;         
double groundtruth_lead_longi_velocity = 0.0;  

double follow_longi_velocity_estimate_error = 0.0;
double lead_longi_velocity_estimate_error = 0.0;

// Yaw Rate
double groundtruth_follow_yaw_rate = 0.0;
double groundtruth_lead_yaw_rate = 0.0;

// Sideslip Angle
double groundtruth_follow_sideslip_angle = 0.0;
double groundtruth_lead_sideslip_angle = 0.0;

double follow_sideslip_angle_estimate_error = 0.0;
double lead_sideslip_angle_estimate_error = 0.0;

// Estimated Lead States (logitudinal velocity vx2, yaw rate wz2, longitudinal accelerator ax2, sideslip angle beta2) from UKF
Eigen::Vector4d groundtruth_lead_states = Eigen::Vector4d::Zero();              /* (0): vx2, (1): wz2, (2): ax2, (3): beta2 */
Eigen::Vector4d estimated_lead_states = Eigen::Vector4d::Zero();
Eigen::Vector4d lead_states_estimate_error = Eigen::Vector4d::Zero();

// Global Data Buffer (save all data from followbus and leadbus)
std::shared_ptr<DataBuff> data_buff_ptr = std::make_shared<DataBuff>();

// Data Buffer only for V2V-Communication Delay
std::deque<IMUData> delayed_lead_imu_data_buff;
std::deque<WheelRotRateData> delayed_lead_wheel_rot_rate_data_buff;
std::deque<WheelSteerAngleData> delayed_lead_wheel_steer_angle_data_buff;

// Lidar
int UseLidar = 0;                   /* DVA Signal for Shutting Down the Lidar.   1: Use Lidar, 0: Shut Down Lidar */
const int LidarL = 0;               /* Indicator for left lidar and right lidar */
const int LidarR = 1;
int is_lidar_rel_pose_valid = 0;    /* Indicator for validity of estimated relative pose by Lidar Tracker. 0: invalid, 1: valid */

// Camera
int UseCamera = 0;                  /* DVA Signal for Shutting Down the Camera.  1: Use Camera, 0: Shut Down Camera */
int num_detected_markers = 0;       /* Number of detected Aruco Markers by Camera Tracker */
int is_camera_rel_pose_valid = 0;   /* Indicator for validity of estimated relative pose by camera Tracker. 0: invalid, 1: valid */

// V2V-Communication
int UseV2V = 0;                     /* DVA Signal for Shutting Down the V2V-Communication.  1: Use V2V, 0: Shut Down V2V */

// The main function flow of Vehicle Positioning
VehiclePositioningFlow vehicle_positioning_flow;

// Standard Deviation of estimated states in rel_pose_estimator
Eigen::VectorXd states_stdd_positiv = Eigen::VectorXd::Zero(7);
Eigen::VectorXd states_stdd_negativ = Eigen::VectorXd::Zero(7); 



/* Data to be shared between the SimNet applications */
typedef struct tMyUserData2Share {
    double Time;                        /* Time when data was set */
    double IMUData[6];                  /* IMUData[0~2]: linear_acceleration,  IMUData[3~5]: angular_velocity */
    double WheelRotRateData[4];         /* WheelRotRateData[0~3]: wheel rotation rate (rad/s) of (FL, FR, RL, RR)-Tires */
    double WheelSteerAngleData[4];      /* WheelSteerAngleData[0~3]: wheel steering angle (rad) of (FL, FR, RL, RR)-Tires */
    double SideslipAngleActual;         /* SideslipAngleActual: Actual Sideslip Angle */
    double VelocityActual[2];           /* VelocityActual[0]: Actual Longitudinal Velocity, VelocityActual[1]: Actual Lateral Velocity */
    double YawRateActual;               /* YawRateActual: Actual Yaw Rate */
    double AbsPoseActual[3];            /* AbsPoseActual[0~2]: x, y, phi */

    char AppStr[16];        /* SimNet application string of data provider */
    int AppType;            /* SimNet application Type of data provider */
    int AppID;              /* SimNet application ID of data provider */
    char Role[16];          /* Role name of data provider in current SimNet scenario */
} tMyUserData2Share;

/* 
    Master Application   ->   AppType: 1, AppStr: SimNet00, AppID: 0
    Slave 1 Application  ->   AppType: 2, AppStr: SimNet01, AppID: 1
 */

/* Data struct for the SimNet data share */
struct {
    tMyUserData2Share DataTx;       /* Data buffer for current application */
    tMyUserData2Share DataRx[10];   /* Data buffer for other applications */
    int nApps;                      /* Number of applications */
    int CycleNoRel;                 /* Relative simulation cycles since TestRun start */
} MySimNetData;



/*
** User_Init_First ()
**
** First, low level initialization of the User module
**
** Call:
** - one times at start of program
** - no realtime conditions
**
*/

int
User_Init_First (void)
{
    memset (&User, 0, sizeof(User));

    return 0;
}



/*
** User_PrintUsage ()
**
** Print the user/application specific programm arguments
*/

void
User_PrintUsage (const char *Pgm)
{
    /* REMARK: 1 log statement for each usage line, no line breaks */
    LogUsage("\n");
    LogUsage("Usage: %s [options] [testrun]\n", Pgm);
    LogUsage("Options:\n");

#if defined(CM_HIL)
    {
	const tIOConfig *cf;
	const char *defio = IO_GetDefault();
	LogUsage(" -io %-12s Default I/O configuration (%s)\n", "default",
	    (defio!=NULL && strcmp(defio, "none")!=0) ? defio : "minimal I/O");
	for (cf=IO_GetConfigurations(); cf->Name!=NULL; cf++)
	    LogUsage(" -io %-12s %s\n", cf->Name, cf->Description);
    }
#endif
}



/*
** User_ScanCmdLine ()
**
** Scan application specific command line arguments
**
** Return:
** - argv: last unscanned argument
** - NULL: error or unknown argument
*/

char **
User_ScanCmdLine (int argc, char **argv)
{
    const char *Pgm = argv[0];

    /* I/O configuration to be used in case no configuration was
       specified on the command line. */
    IO_SelectDefault("default" /* or "demoapp", "demorbs,demofr" etc. */);

    while (*++argv) {
	if (strcmp(*argv, "-io") == 0 && argv[1] != NULL) {
	    if (IO_Select(*++argv) != 0)
		return NULL;
	} else if (strcmp(*argv, "-h") == 0 || strcmp(*argv, "-help") == 0) {
	    User_PrintUsage(Pgm);
	    SimCore_PrintUsage(Pgm); /* Possible exit(), depending on CM-platform! */
	    return  NULL;
	} else if ((*argv)[0] == '-') {
	    LogErrF(EC_General, "Unknown option '%s'", *argv);
	    return NULL;
	} else {
	    break;
	}
    }

    return argv;
}



/*
** User_Init ()
**
** Basic initialization of the module User.o
**
** Call:
** - once at program start
** - no realtime conditions
*/

int
User_Init (void)
{
    return 0;
}



int
User_Register (void)
{

    /* @@PLUGIN-BEGIN-REGISTER@@ - Automatically generated code - don't edit! */
    /* @@PLUGIN-END@@ */

    return 0;
}



/*
** User_DeclQuants ()
**
** Add user specific quantities to the dictionary
**
** Call:
** - once at program start
** - no realtime conditions
*/

void
User_DeclQuants (void)
{
    // Velocity
    DDefDouble(NULL, "groundtruth_follow_longi_velocity", "m/s", &groundtruth_follow_longi_velocity, DVA_IO_Out);
    DDefDouble(NULL, "estimated_follow_longi_velocity", "m/s", &vehicle_positioning_flow.estimated_follow_longi_velocity_, DVA_IO_Out);
    
    DDefDouble(NULL, "groundtruth_lead_longi_velocity", "m/s", &groundtruth_lead_longi_velocity, DVA_IO_Out);
    DDefDouble(NULL, "estimated_lead_longi_velocity", "m/s", &vehicle_positioning_flow.estimated_lead_longi_velocity_, DVA_IO_Out);

    DDefDouble(NULL, "follow_longi_velocity_estimate_error", "m/s", &follow_longi_velocity_estimate_error, DVA_IO_Out);
    DDefDouble(NULL, "lead_longi_velocity_estimate_error", "m/s", &lead_longi_velocity_estimate_error, DVA_IO_Out);

    // Sideslip Angle
    DDefDouble(NULL, "groundtruth_follow_sideslip_angle", "rad", &groundtruth_follow_sideslip_angle, DVA_IO_Out);
    DDefDouble(NULL, "estimated_follow_sideslip_angle", "rad", &vehicle_positioning_flow.estimated_follow_sideslip_angle_, DVA_IO_Out);

    DDefDouble(NULL, "groundtruth_lead_sideslip_angle", "rad", &groundtruth_lead_sideslip_angle, DVA_IO_Out);
    DDefDouble(NULL, "estimated_lead_sideslip_angle", "rad", &vehicle_positioning_flow.estimated_lead_sideslip_angle_, DVA_IO_Out);

    DDefDouble(NULL, "follow_sideslip_angle_estimate_error", "rad", &follow_sideslip_angle_estimate_error, DVA_IO_Out);
    DDefDouble(NULL, "lead_sideslip_angle_estimate_error", "rad", &lead_sideslip_angle_estimate_error, DVA_IO_Out);

    // Relative Pose RF
    DDefDouble(NULL, "camera_rel_pose_dx", "m", &camera_rel_pose(0), DVA_IO_Out);
    DDefDouble(NULL, "camera_rel_pose_dy", "m", &camera_rel_pose(1), DVA_IO_Out);
    DDefDouble(NULL, "camera_rel_pose_dphi", "rad", &camera_rel_pose(2), DVA_IO_Out);

    DDefDouble(NULL, "lidar_rel_pose_dx", "m", &lidar_rel_pose(0), DVA_IO_Out);
    DDefDouble(NULL, "lidar_rel_pose_dy", "m", &lidar_rel_pose(1), DVA_IO_Out);
    DDefDouble(NULL, "lidar_rel_pose_dphi", "rad", &lidar_rel_pose(2), DVA_IO_Out);

    DDefDouble(NULL, "fusion_rel_pose_dx",     "m", &fusion_rel_pose(0), DVA_IO_Out);
    DDefDouble(NULL, "fusion_rel_pose_dy", "m", &fusion_rel_pose(1), DVA_IO_Out);
    DDefDouble(NULL, "fusion_rel_pose_dphi", "rad", &fusion_rel_pose(2), DVA_IO_Out);

    DDefDouble(NULL, "groundtruth_rel_pose_dx", "m", &groundtruth_rel_pose(0), DVA_IO_Out);
    DDefDouble(NULL, "groundtruth_rel_pose_dy", "m", &groundtruth_rel_pose(1), DVA_IO_Out);
    DDefDouble(NULL, "groundtruth_rel_pose_dphi", "rad", &groundtruth_rel_pose(2), DVA_IO_Out);

    // Realtive Pose Estimation Error 
    DDefDouble(NULL, "camera_rel_pose_dx_estimate_error", "m", &camera_rel_pose_estimate_error(0), DVA_IO_Out);
    DDefDouble(NULL, "camera_rel_pose_dy_estimate_error", "m", &camera_rel_pose_estimate_error(1), DVA_IO_Out);
    DDefDouble(NULL, "camera_rel_pose_dphi_estimate_error", "rad", &camera_rel_pose_estimate_error(2), DVA_IO_Out);

    DDefDouble(NULL, "lidar_rel_pose_dx_estimate_error", "m", &lidar_rel_pose_estimate_error(0), DVA_IO_Out);
    DDefDouble(NULL, "lidar_rel_pose_dy_estimate_error", "m", &lidar_rel_pose_estimate_error(1), DVA_IO_Out);
    DDefDouble(NULL, "lidar_rel_pose_dphi_estimate_error", "rad", &lidar_rel_pose_estimate_error(2), DVA_IO_Out);

    DDefDouble(NULL, "fusion_rel_pose_dx_estimate_error", "m", &fusion_rel_pose_estimate_error(0), DVA_IO_Out);
    DDefDouble(NULL, "fusion_rel_pose_dy_estimate_error", "m", &fusion_rel_pose_estimate_error(1), DVA_IO_Out);
    DDefDouble(NULL, "fusion_rel_pose_dphi_estimate_error", "rad", &fusion_rel_pose_estimate_error(2), DVA_IO_Out);

    // Absolute Pose
    DDefDouble(NULL, "groundtruth_follow_abs_pose_x", "m", &groundtruth_follow_abs_pose(0), DVA_IO_Out);
    DDefDouble(NULL, "groundtruth_follow_abs_pose_y", "m", &groundtruth_follow_abs_pose(1), DVA_IO_Out);
    DDefDouble(NULL, "groundtruth_follow_abs_pose_phi", "rad", &groundtruth_follow_abs_pose(2), DVA_IO_Out);

    DDefDouble(NULL, "estimated_follow_abs_pose_x", "m", &estimated_follow_abs_pose(0), DVA_IO_Out);
    DDefDouble(NULL, "estimated_follow_abs_pose_y", "m", &estimated_follow_abs_pose(1), DVA_IO_Out);
    DDefDouble(NULL, "estimated_follow_abs_pose_phi", "rad", &estimated_follow_abs_pose(2), DVA_IO_Out);

    DDefDouble(NULL, "groundtruth_lead_abs_pose_x", "m", &groundtruth_lead_abs_pose(0), DVA_IO_Out);
    DDefDouble(NULL, "groundtruth_lead_abs_pose_y", "m", &groundtruth_lead_abs_pose(1), DVA_IO_Out);
    DDefDouble(NULL, "groundtruth_lead_abs_pose_phi", "rad", &groundtruth_lead_abs_pose(2), DVA_IO_Out);

    // Absolute Pose Estimation Error
    DDefDouble(NULL, "follow_abs_pose_x_estimate_error", "m", &follow_abs_pose_estimate_error(0), DVA_IO_Out);
    DDefDouble(NULL, "follow_abs_pose_y_estimate_error", "m", &follow_abs_pose_estimate_error(1), DVA_IO_Out);
    DDefDouble(NULL, "follow_abs_pose_phi_estimate_error", "rad", &follow_abs_pose_estimate_error(2), DVA_IO_Out);

    // Estimated Lead Bus States from Filter
    DDefDouble(NULL, "estimated_lead_states_vx2", "m/s", &estimated_lead_states(0), DVA_IO_Out);
    DDefDouble(NULL, "estimated_lead_states_wz2", "rad/s", &estimated_lead_states(1), DVA_IO_Out);
    DDefDouble(NULL, "estimated_lead_states_ax2", "m/s^2", &estimated_lead_states(2), DVA_IO_Out);
    DDefDouble(NULL, "estimated_lead_states_beta2", "rad", &estimated_lead_states(3), DVA_IO_Out);

    DDefDouble(NULL, "groundtruth_lead_states_vx2", "m/s", &groundtruth_lead_states(0), DVA_IO_Out);
    DDefDouble(NULL, "groundtruth_lead_states_wz2", "rad/s", &groundtruth_lead_states(1), DVA_IO_Out);
    DDefDouble(NULL, "groundtruth_lead_states_ax2", "m/s^2", &groundtruth_lead_states(2), DVA_IO_Out);
    DDefDouble(NULL, "groundtruth_lead_states_beta2", "rad", &groundtruth_lead_states(3), DVA_IO_Out);

    DDefDouble(NULL, "lead_states_estimate_error_vx2", "m/s", &lead_states_estimate_error(0), DVA_IO_Out);
    DDefDouble(NULL, "lead_states_estimate_error_wz2", "rad/s", &lead_states_estimate_error(1), DVA_IO_Out);
    DDefDouble(NULL, "lead_states_estimate_error_ax2", "m/s^2", &lead_states_estimate_error(2), DVA_IO_Out);
    DDefDouble(NULL, "lead_states_estimate_error_beta2", "rad", &lead_states_estimate_error(3), DVA_IO_Out);

    // Indicators
    DDefInt(NULL, "number_of_detected_markers", "", &num_detected_markers, DVA_IO_Out);
    DDefInt(NULL, "is_lidar_rel_pose_valid", "", &is_lidar_rel_pose_valid, DVA_IO_Out);
    DDefInt(NULL, "is_camera_rel_pose_valid", "", &is_camera_rel_pose_valid, DVA_IO_Out);

    // Direct Variable Access
    DDefInt(NULL, "Use_Lidar", "", &UseLidar, DVA_DM);
    DDefInt(NULL, "Use_Camera", "", &UseCamera, DVA_DM);
    DDefInt(NULL, "Use_V2V", "", &UseV2V, DVA_DM);

    // Standard Deviation of estimated states in rel_pose_estimator
    DDefDouble(NULL, "stdd_dx_positive", "m", &states_stdd_positiv(0), DVA_IO_Out);
    DDefDouble(NULL, "stdd_dy_positive", "m", &states_stdd_positiv(1), DVA_IO_Out);
    DDefDouble(NULL, "stdd_dphi_positive", "rad", &states_stdd_positiv(2), DVA_IO_Out);
    DDefDouble(NULL, "stdd_vx2_positive", "m/s", &states_stdd_positiv(3), DVA_IO_Out);
    DDefDouble(NULL, "stdd_wz2_positive", "rad/s", &states_stdd_positiv(4), DVA_IO_Out);
    DDefDouble(NULL, "stdd_ax2_positive", "m/s^2", &states_stdd_positiv(5), DVA_IO_Out);
    DDefDouble(NULL, "stdd_beta2_positive", "rad", &states_stdd_positiv(6), DVA_IO_Out);

    DDefDouble(NULL, "stdd_dx_negativ", "m", &states_stdd_negativ(0), DVA_IO_Out);
    DDefDouble(NULL, "stdd_dy_negativ", "m", &states_stdd_negativ(1), DVA_IO_Out);
    DDefDouble(NULL, "stdd_dphi_negativ", "rad", &states_stdd_negativ(2), DVA_IO_Out);
    DDefDouble(NULL, "stdd_vx2_negativ", "m/s", &states_stdd_negativ(3), DVA_IO_Out);
    DDefDouble(NULL, "stdd_wz2_negativ", "rad/s", &states_stdd_negativ(4), DVA_IO_Out);
    DDefDouble(NULL, "stdd_ax2_negativ", "m/s^2", &states_stdd_negativ(5), DVA_IO_Out);
    DDefDouble(NULL, "stdd_beta2_negativ", "rad", &states_stdd_negativ(6), DVA_IO_Out);

    // Velocity Estimator
    DDefDouble(NULL, "follow_wheel_speed_CG_FL", "m/s", &vehicle_positioning_flow.follow_wheel_speed_CG_(0), DVA_IO_Out);
    DDefDouble(NULL, "follow_wheel_speed_CG_FR", "m/s", &vehicle_positioning_flow.follow_wheel_speed_CG_(1), DVA_IO_Out);
    DDefDouble(NULL, "follow_wheel_speed_CG_RL", "m/s", &vehicle_positioning_flow.follow_wheel_speed_CG_(2), DVA_IO_Out);
    DDefDouble(NULL, "follow_wheel_speed_CG_RR", "m/s", &vehicle_positioning_flow.follow_wheel_speed_CG_(3), DVA_IO_Out);

    DDefInt(NULL, "is_follow_wheel_data_valid_FL", "", &vehicle_positioning_flow.is_follow_wheel_data_valid_(0), DVA_IO_Out);
    DDefInt(NULL, "is_follow_wheel_data_valid_FR", "", &vehicle_positioning_flow.is_follow_wheel_data_valid_(1), DVA_IO_Out);
    DDefInt(NULL, "is_follow_wheel_data_valid_RL", "", &vehicle_positioning_flow.is_follow_wheel_data_valid_(2), DVA_IO_Out);
    DDefInt(NULL, "is_follow_wheel_data_valid_RR", "", &vehicle_positioning_flow.is_follow_wheel_data_valid_(3), DVA_IO_Out);

    DDefDouble(NULL, "follow_wheel_rot_rate_change_FL", "rad/s", &vehicle_positioning_flow.follow_wheel_rot_rate_change_(0), DVA_IO_Out);
    DDefDouble(NULL, "follow_wheel_rot_rate_change_FR", "rad/s", &vehicle_positioning_flow.follow_wheel_rot_rate_change_(1), DVA_IO_Out);
    DDefDouble(NULL, "follow_wheel_rot_rate_change_RL", "rad/s", &vehicle_positioning_flow.follow_wheel_rot_rate_change_(2), DVA_IO_Out);
    DDefDouble(NULL, "follow_wheel_rot_rate_change_RR", "rad/s", &vehicle_positioning_flow.follow_wheel_rot_rate_change_(3), DVA_IO_Out);

    DDefDouble(NULL, "follow_velocity_deviation_FL", "m/s", &vehicle_positioning_flow.follow_velocity_deviation_(0), DVA_IO_Out);
    DDefDouble(NULL, "follow_velocity_deviation_FR", "m/s", &vehicle_positioning_flow.follow_velocity_deviation_(1), DVA_IO_Out);
    DDefDouble(NULL, "follow_velocity_deviation_RL", "m/s", &vehicle_positioning_flow.follow_velocity_deviation_(2), DVA_IO_Out);
    DDefDouble(NULL, "follow_velocity_deviation_RR", "m/s", &vehicle_positioning_flow.follow_velocity_deviation_(3), DVA_IO_Out);

    int i;

    for (i=0; i<N_USEROUTPUT; i++) {
	char sbuf[32];
	sprintf (sbuf, "UserOut_%02d", i);
	DDefDouble (NULL, sbuf, "", &User.Out[i], DVA_IO_Out);
    }
    RBS_DeclQuants();
}


/*
** User_Param_Add ()
**
** Update all modified application specific parameters in the test stand
** parameter file (ECUParameters).
**
** If the variable SimCore.TestRig.ECUParam.Modified set to 1 somewhere else
** CarMaker calls this function to let the user add or change all necessary
** entries before the file is written.
** So, if writing the ECUParam file is necessary, set ECUParam.Modified to 1.
** The next TestRun start or end, CarMaker calls this function and writes
** the file to the harddisk.
**
** Call:
** - in a separate thread (no realtime contitions)
** - when starting a new test run
*/

int
User_Param_Add (void)
{
#if defined(CM_HIL)
    /* ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL)
	return -1;
#endif

    return 0;
}



/*
** User_Param_Get ()
**
** Update all modified application specific parameters from the test stand
** parameter file (ECUParameters).
**
** Call:
** - in a separate thread (no realtime conditions)
** - if User_Param_Get() wasn't called
** - when starting a new test run, if
**   - the files SimParameters and/or
**   - ECUParameters
**   are modified since last reading
**
** return values:
**  0	ok
** -1	no testrig parameter file
** -2	testrig parameter error
** -3	i/o configuration specific error
** -4	no simulation parameters
** -5	simulation parameters error
** -6	FailSafeTester parameter/init error
*/

int
User_Param_Get (void)
{
    int rv = 0;

#if defined(CM_HIL)
    /*** testrig / ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL)
	return -1;

    if (IO_Param_Get(SimCore.TestRig.ECUParam.Inf) != 0)
	rv = -2;
#endif

    /*** simulation parameters */
    if (SimCore.TestRig.SimParam.Inf == NULL)
	return -4;

    return rv;
}



/*
** User_TestRun_Start_atBegin ()
**
** Special things before a new simulation starts like
** - reset user variables to their default values
** - reset counters
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when starting a new test run
** - after (standard) infofiles are read in
** - before reading parameters for Environment, DrivMan, Car, ...
**   the models are NOT in the simulation-can-start-now state
**   (after Start(), before StaticCond())
*/

int
User_TestRun_Start_atBegin (void)
{
    int rv = 0;
    int i;

    for (i=0; i<N_USEROUTPUT; i++)
	User.Out[i] = 0.0;

    /* Reset the SimNet internal buffer */
    SimNet_User_DataDel();
    /* Reset the local buffer */
    memset (&MySimNetData, 0, sizeof(MySimNetData));

    if (IO_None)
	return rv;

#if defined(CM_HIL)
    if (FST_New(SimCore.TestRig.ECUParam.Inf) != 0)
	rv = -6;
#endif

    return rv;
}




/*
** User_TestRun_Start_atEnd ()
**
** Special things before a new simulation starts like
** - reset user variables to there default values
** - reset counters
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when starting a new test run
** - at the end, behind reading parameters for Environment, DrivMan,
**   Car, ...
**   the models are NOT in the simulation-can-start-now state
**   (after Start(), before StaticCond())
*/

int
User_TestRun_Start_atEnd (void)
{
#if defined(XENO)
    IOConf_DeclQuants();
#endif

    /* Get the configuration of current SimNet application */
    SimNet_App_InfoGet(MySimNetData.DataTx.AppStr,
                        sizeof(MySimNetData.DataTx.AppStr),
                        &(MySimNetData.DataTx.AppType),
                        &(MySimNetData.DataTx.AppID),
                        &(MySimNetData.nApps),
                        MySimNetData.DataTx.Role,
                        sizeof(MySimNetData.DataTx.Role));

    return 0;
}



/*
** User_TestRun_Start_StaticCond_Calc ()
**
** called in non RT context
*/

int
User_TestRun_Start_StaticCond_Calc (void)
{
    return 0;
}



/*
** User_TestRun_Start_Finalize ()
**
** called in RT context
*/

int
User_TestRun_Start_Finalize (void)
{
    return 0;
}



/*
** User_TestRun_RampUp ()
**
** Perform a smooth transition of variables (e.g. I/O)
** from their current state  to the new testrun.
** This function is called repeatedly, once during each cycle, until
** it returns true (or issues an error message), so the function should
** return true if transitioning is done, false otherwise.
**
** In case of an error the function should issue an apropriate
** error message and return false;
**
** Called in RT context, in state SCState_StartSim,
** after preprocessing is done, before starting the engine.
** Please note, that in this early initialization state no calculation
** of the vehicle model takes place.
*/

int
User_TestRun_RampUp (double dt)
{
    int IsReady = 1;

    return IsReady;
}



/*
** User_TestRun_End_First ()
**
** Invoked immediately after the end of a simulation is initiated,
** but before data storage ends and before transitioning into SCState_Idle.
** - Send Scratchpad-note
** - ...
**
** Call:
** - in main task, in the main loop (real-time conditions!)
** - when a test run is finished (SimCore.State is SCState_End)
*/

int
User_TestRun_End_First (void)
{
    return 0;
}



/*
** User_TestRun_End ()
**
** Special things after the end of a simulation like
** - switch off an air compressor
** - Write something to a file
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when a test run is finished (SimCore.State is SCState_End<xyz>)
*/

int
User_TestRun_End (void)
{
    return 0;
}



/*
** User_In ()
**
** Assign quantities of the i/o vector to model variables
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - just after IO_In()
*/

void
User_In (const unsigned CycleNo)
{
    if (SimCore.State != SCState_Simulate)
	return;

    
    /* Receive Data from SimNet Data Pools */
    if (MySimNetData.DataTx.AppType == 1)    // Only SimNet master
    {
        int i;
        tMyUserData2Share *pdata = NULL;

        for (i = 0; i < MySimNetData.nApps; i++) {                                     
            pdata = &(MySimNetData.DataRx[i]);
            SimNet_User_DataGet(i, pdata, sizeof(*pdata));

            // 1. Receive IMU Data
            IMUData imu_data;
            imu_data.linear_acceleration_.x = pdata->IMUData[0];
            imu_data.linear_acceleration_.y = pdata->IMUData[1];
            imu_data.linear_acceleration_.z = pdata->IMUData[2];
            imu_data.angular_velocity_.x = pdata->IMUData[3];
            imu_data.angular_velocity_.y = pdata->IMUData[4];
            imu_data.angular_velocity_.z = pdata->IMUData[5];
            imu_data.timestamp_ = pdata->Time;

            if (pdata->AppID == MySimNetData.DataTx.AppID)   
            {
                if(MySimNetData.CycleNoRel%imu_cycle_ms == 1)
                {
                    // Receive IMU data from follow bus (every imu data measurement cycle)
                    data_buff_ptr->follow_imu_data_buff_.push_back(imu_data);
                }
            } 
            else
            {
                groundtruth_lead_states(1) = imu_data.angular_velocity_.z;
                groundtruth_lead_states(2) = imu_data.linear_acceleration_.x;
#if AddV2VCommunicateDelay
                // Add V2V-Communication Delay to Lead Bus IMU Data
                delayed_lead_imu_data_buff.push_back(imu_data);   
                if(static_cast<int>(delayed_lead_imu_data_buff.size()) > v2v_communicate_delay_ms)
                {
                    // Receive IMU data from lead bus (every v2v communication cycle cycle)
                    if(MySimNetData.CycleNoRel%v2v_communicate_cycle_ms == 1 && MySimNetData.CycleNoRel%imu_cycle_ms == 1)
                    {
                        if(UseV2V == 1)
                        {
                            data_buff_ptr->lead_imu_data_buff_.push_back(delayed_lead_imu_data_buff.front());  // With V2V-Communication Delay 
                        }
                        
                    }

                    delayed_lead_imu_data_buff.pop_front();
                }   
#else
                // Without V2V-Communication Delay 
                if(MySimNetData.CycleNoRel%v2v_communicate_cycle_ms == 1 && MySimNetData.CycleNoRel%imu_cycle_ms == 1)
                {
                    if(!ShutDownV2V)
                    {
                        data_buff_ptr->lead_imu_data_buff_.push_back(imu_data);
                    }
                                              
                }
#endif
            }        


            // 2. Receive Wheel Rotation Rate Data
            WheelRotRateData wheel_rot_rate_data;
            wheel_rot_rate_data.wheel_rot_rate_(0) = pdata->WheelRotRateData[0];
            wheel_rot_rate_data.wheel_rot_rate_(1) = pdata->WheelRotRateData[1];
            wheel_rot_rate_data.wheel_rot_rate_(2) = pdata->WheelRotRateData[2];
            wheel_rot_rate_data.wheel_rot_rate_(3) = pdata->WheelRotRateData[3];
            wheel_rot_rate_data.timestamp_ = pdata->Time;

            if (pdata->AppID == MySimNetData.DataTx.AppID)
            {
                if (MySimNetData.CycleNoRel%wheel_rot_rate_cycle_ms == 1)
                {
                    // Receive Wheel ratation rate data from follow bus (every wheel rotation rate data measurement cycle)
                    data_buff_ptr->follow_wheel_rot_rate_data_buff_.push_back(wheel_rot_rate_data);
                }              
            }
            else
            {
#if AddV2VCommunicateDelay
                // Add V2V-Communication Delay to Lead Bus Wheel Data
                delayed_lead_wheel_rot_rate_data_buff.push_back(wheel_rot_rate_data);   
                if(static_cast<int>(delayed_lead_wheel_rot_rate_data_buff.size()) > v2v_communicate_delay_ms)
                {
                    // Receive Wheel data from lead bus (every v2v communication cycle cycle)
                    if(MySimNetData.CycleNoRel%v2v_communicate_cycle_ms == 1 && MySimNetData.CycleNoRel%wheel_rot_rate_cycle_ms == 1)
                    {
                        if(UseV2V == 1)
                        {
                            data_buff_ptr->lead_wheel_rot_rate_data_buff_.push_back(delayed_lead_wheel_rot_rate_data_buff.front());  // With V2V-Communication Delay 
                        }
                    }

                    delayed_lead_wheel_rot_rate_data_buff.pop_front();
                } 
#else
                // Without V2V-Communication Delay 
                if(MySimNetData.CycleNoRel%v2v_communicate_cycle_ms == 1 && MySimNetData.CycleNoRel%wheel_rot_rate_cycle_ms == 1)
                {
                    if(!ShutDownV2V)
                    {
                        data_buff_ptr->lead_wheel_rot_rate_data_buff_.push_back(wheel_rot_rate_data);
                    }                         
                }
#endif
            }


            // 3. Receive Wheel Steering Angle Data
            WheelSteerAngleData wheel_steer_angle_data;
            wheel_steer_angle_data.wheel_steer_angle_(0) = pdata->WheelSteerAngleData[0];
            wheel_steer_angle_data.wheel_steer_angle_(1) = pdata->WheelSteerAngleData[1];
            wheel_steer_angle_data.wheel_steer_angle_(2) = pdata->WheelSteerAngleData[2];
            wheel_steer_angle_data.wheel_steer_angle_(3) = pdata->WheelSteerAngleData[3];
            wheel_steer_angle_data.timestamp_ = pdata->Time;

            if (pdata->AppID == MySimNetData.DataTx.AppID)
            {
                if (MySimNetData.CycleNoRel%wheel_steer_angle_cycle_ms == 1)
                {
                    // Receive wheel steer angle data from follow bus (every wheel steer angle data measurement cycle)
                    data_buff_ptr->follow_wheel_steer_angle_data_buff_.push_back(wheel_steer_angle_data);
                }              
            }
            else
            {
#if AddV2VCommunicateDelay
                // Add V2V-Communication Delay to Lead Bus Wheel Steering Angle Data
                delayed_lead_wheel_steer_angle_data_buff.push_back(wheel_steer_angle_data);   
                if(static_cast<int>(delayed_lead_wheel_steer_angle_data_buff.size()) > v2v_communicate_delay_ms)
                {
                    // Receive wheel steer angle data from lead bus (every v2v communication cycle cycle)
                    if(MySimNetData.CycleNoRel%v2v_communicate_cycle_ms == 1 && MySimNetData.CycleNoRel%wheel_steer_angle_cycle_ms == 1)
                    {
                        if(UseV2V == 1)
                        {
                            data_buff_ptr->lead_wheel_steer_angle_data_buff_.push_back(delayed_lead_wheel_steer_angle_data_buff.front());  // With V2V-Communication Delay 
                        }
                    }
                    delayed_lead_wheel_steer_angle_data_buff.pop_front();
                } 
#else
                // Without V2V-Communication Delay 
                if(MySimNetData.CycleNoRel%v2v_communicate_cycle_ms == 1 && MySimNetData.CycleNoRel%wheel_steer_angle_cycle_ms == 1)
                {
                    if(!ShutDownV2V)
                    {
                        data_buff_ptr->lead_wheel_steer_angle_data_buff_.push_back(wheel_steer_angle_data);  
                    }                   
                }
#endif
            }


            /* 4. Receive Groundtruth Data from follow bus and lead bus (every CM cycle) */
            if (pdata->AppID == MySimNetData.DataTx.AppID)                         
            {                                                                    
                groundtruth_follow_abs_pose = Eigen::Vector3d(pdata->AbsPoseActual[0], pdata->AbsPoseActual[1], pdata->AbsPoseActual[2]);
                groundtruth_follow_longi_velocity = pdata->VelocityActual[0];
                groundtruth_follow_yaw_rate = pdata->YawRateActual;
                groundtruth_follow_sideslip_angle = pdata->SideslipAngleActual;
            }
            else                                                                                                
            {                                                                       
                groundtruth_lead_abs_pose = Eigen::Vector3d(pdata->AbsPoseActual[0], pdata->AbsPoseActual[1], pdata->AbsPoseActual[2]);
                groundtruth_lead_longi_velocity = pdata->VelocityActual[0];
                groundtruth_lead_yaw_rate = pdata->YawRateActual;
                groundtruth_lead_sideslip_angle = pdata->SideslipAngleActual;

                groundtruth_lead_states(0) = pdata->VelocityActual[0];
                groundtruth_lead_states(3) = pdata->SideslipAngleActual;
            }

        }      
    }

    /* Erase old sensor data, if data buffer is full */
    data_buff_ptr->CheckBuffLength();

}




/*
** User_DrivMan_Calc ()
**
** called
** - in RT context
** - after DrivMan_Calc()
*/

int
User_DrivMan_Calc (double dt)
{
    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving)
	return 0;

    return 0;
}


/*
** User_VehicleControl_Calc ()
**
** called
** - in RT context
** - after VehicleControl_Calc()
*/

int
User_VehicleControl_Calc (double dt)
{
    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving)
	return 0;

    return 0;
}



/*
** User_Brake_Calc ()
**
** called
** - in RT context
** - after Brake_Calc() in Vhcl_Calc()
*/

int
User_Brake_Calc (double dt)
{
    /* Modify the total brake torque from the brake system model Brake.Trq_tot[]
       or the target drive source torque from the brake control unit
       Brake.HydBrakeCU_IF.Trq_DriveSrc_trg[]
    */

    return 0;
}



/*
** User_Traffic_Calc ()
**
** called
** - in RT context
** - after Traffic_Calc()
*/

int
User_Traffic_Calc (double dt)
{
    if (SimCore.State != SCState_Simulate)
	return 0;

    return 0;
}



/*
** User_Calc ()
**
** called in RT context
*/

int
User_Calc (double dt)
{
    /* Starting with CM 6.0 User_Calc() will be invoked in EVERY simulation
       state. Uncomment the following line in order to restore the behaviour
       of CM 5.1 and earlier. */
    /*if (!UserCalcCalledByAppTestRunCalc) return 0;*/

    if (SimCore.State == SCState_Start) {           // Set the initial value at the simulation beginning
        UseLidar = 1;
        UseCamera = 1;
        UseV2V = 1;
        vehicle_positioning_flow.is_initialized_ = false;
    }

    return 0;
}



/*
** User_Check_IsIdle ()
**
** Checking, if the simulation model is in idle conditions (stand still,
** steeringwheel angle zero, cluch pedal pressed, ...).
** If reached idle state, the calculation of vehicle model and driving
** manoevers is stopped.
** Ready for start new simulation.
**
** Return:
** 1  idle state reached
** 0  else
**
** Call:
** - in main task, in the main loop
** - pay attention to realtime condition
** - while SimCore.State==SCState_EndIdleGet
*/

int
User_Check_IsIdle (int IsIdle)
{
    double val;

    /*** ECU / carmodel signals */

    /* vehicle and wheels: stand still */
    val = 0.5*kmh2ms;
    if (Vehicle.v > val
     || fabs(Vehicle.Wheel[0]->vBelt) > val || fabs(Vehicle.Wheel[1]->vBelt) > val
     || fabs(Vehicle.Wheel[2]->vBelt) > val || fabs(Vehicle.Wheel[3]->vBelt) > val) {
	IsIdle = 0;
    }

    /* SteerAngle: drive  straight forward position */
    val = 1.0*deg2rad;
    if (Vehicle.Steering.Ang > val || Vehicle.Steering.Ang < -val)
	IsIdle = 0;

    return IsIdle;
}



/*
** User_Out ()
**
** Assigns model quantities to variables of the i/o vector
**
** call:
** - in the main loop
** - pay attention to realtime condition
** - just before IO_Out();
*/

void
User_Out (const unsigned CycleNo)
{
    RBS_OutMap(CycleNo);
#if defined(XENO)
    IOConf_OutMap();
#endif

    if (SimCore.State != SCState_Simulate)
	return;


    /* Calculate actual relative pose (only Master Application) */
    if(MySimNetData.DataTx.AppType == 1)
    {
        groundtruth_rel_pose = tools::CalculateActualRelPose(groundtruth_follow_abs_pose, groundtruth_lead_abs_pose);
    }


    /* Provide Sensor Data (IMU, Wheel Ratation Rate Data, Wheel Steering Angle, Other Actual Vehicle Data) (Master and Slave Application) */
    tMyUserData2Share *pdata = &MySimNetData.DataTx;
    pdata->Time = SimCore.Time;

    // IMU
    pdata->IMUData[0] = InertialSensor[0].Acc_B[0];
    pdata->IMUData[1] = InertialSensor[0].Acc_B[1];
    pdata->IMUData[2] = InertialSensor[0].Acc_B[2];

    pdata->IMUData[3] = InertialSensor[0].Omega_B[0];
    pdata->IMUData[4] = InertialSensor[0].Omega_B[1];
    pdata->IMUData[5] = InertialSensor[0].Omega_B[2];

    // Wheel Rotation Rate [rad/s] Data
    pdata->WheelRotRateData[0] = Car.Tire[0].WheelSpd;
    pdata->WheelRotRateData[1] = Car.Tire[1].WheelSpd;
    pdata->WheelRotRateData[2] = Car.Tire[2].WheelSpd;
    pdata->WheelRotRateData[3] = Car.Tire[3].WheelSpd;

    // Wheel Steering Angle [rad] Data
    pdata->WheelSteerAngleData[0] = Car.Susp[0].SteerAngle;
    pdata->WheelSteerAngleData[1] = Car.Susp[1].SteerAngle;
    pdata->WheelSteerAngleData[2] = Car.Susp[2].SteerAngle;
    pdata->WheelSteerAngleData[3] = Car.Susp[3].SteerAngle; 

    // Actual Velocity (vx, vy)
    pdata->VelocityActual[0] = Car.ConBdy1.v_1[0];
    pdata->VelocityActual[1] = Car.ConBdy1.v_1[1];

    // Actual Yaw Rate
    pdata->YawRateActual = InertialSensor[0].Omega_B[2];

    // Actual Sideslip Angle
    pdata->SideslipAngleActual = Car.ConBdy1.SideSlipAngle;

    // Actual Absolute Pose of Vehicle Rear (Origin Fr1)
    pdata->AbsPoseActual[0] = Car.Fr1.t_0[0];
    pdata->AbsPoseActual[1] = Car.Fr1.t_0[1];
    pdata->AbsPoseActual[2] = Car.Fr1.r_zyx[2];

    /* Provide Lidar Scan Point Data (only Master Application) */
    if(MySimNetData.DataTx.AppType == 1 && MySimNetData.CycleNoRel%lidar_cycle_ms == 0)
    {
        ScanPointData scan_point_data;

        scan_point_data.scan_point_ptr_[0] = LidarRSI[LidarL].ScanPoint;
        scan_point_data.scan_point_ptr_[1] = LidarRSI[LidarR].ScanPoint;

        scan_point_data.num_ScanPoints_[0] = LidarRSI[LidarL].nScanPoints;
        scan_point_data.num_ScanPoints_[1] = LidarRSI[LidarR].nScanPoints;

        scan_point_data.timestamp_ = pdata->Time;

        data_buff_ptr->scan_point_data_buff_.push_back(scan_point_data);
    }
        
    /* Send Message */
    SimNet_User_DataOut(pdata, sizeof(*pdata));

    /* Increase relative cycle number at very beginning or at end of cycle */
    MySimNetData.CycleNoRel++;




    if(MySimNetData.DataTx.AppType == 1)
    {
        // "vehicle_positioning_flow" estimates the relative pose and absolute pose at every CPU compute cycle
        if(MySimNetData.CycleNoRel%compute_cycle_ms == 0)
        {
            vehicle_positioning_flow.Run(data_buff_ptr);
            states_stdd_positiv = vehicle_positioning_flow.states_stdd_;
            states_stdd_negativ = - vehicle_positioning_flow.states_stdd_;

            lidar_rel_pose = vehicle_positioning_flow.lidar_rel_pose_;
            camera_rel_pose = vehicle_positioning_flow.camera_rel_pose_;
            fusion_rel_pose = vehicle_positioning_flow.fusion_rel_pose_;

            estimated_follow_abs_pose = vehicle_positioning_flow.estimated_follow_abs_pose_;

            estimated_lead_states = vehicle_positioning_flow.estimated_lead_states_;
            lead_states_estimate_error = estimated_lead_states - groundtruth_lead_states;

            // Calculte Estimation Error
            camera_rel_pose_estimate_error = camera_rel_pose - groundtruth_rel_pose;
            lidar_rel_pose_estimate_error = lidar_rel_pose - groundtruth_rel_pose;
            fusion_rel_pose_estimate_error = fusion_rel_pose - groundtruth_rel_pose;

            follow_abs_pose_estimate_error = estimated_follow_abs_pose - groundtruth_follow_abs_pose;

            is_lidar_rel_pose_valid = vehicle_positioning_flow.is_lidar_rel_pose_valid_;
            num_detected_markers = vehicle_positioning_flow.num_detected_markers_;

            is_camera_rel_pose_valid = vehicle_positioning_flow.is_camera_rel_pose_valid_;

            follow_longi_velocity_estimate_error = vehicle_positioning_flow.estimated_follow_longi_velocity_ - groundtruth_follow_longi_velocity;
            lead_longi_velocity_estimate_error = vehicle_positioning_flow.estimated_lead_longi_velocity_ - groundtruth_lead_longi_velocity;

            follow_sideslip_angle_estimate_error = vehicle_positioning_flow.estimated_follow_sideslip_angle_ - groundtruth_follow_sideslip_angle;
            lead_sideslip_angle_estimate_error = vehicle_positioning_flow.estimated_lead_sideslip_angle_ - groundtruth_lead_sideslip_angle;
        }
    }

}



/*
** User_ApoMsg_Eval ()
**
** Communication between the application and connected GUIs.
** Evaluate messages from GUIs
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - near the end of the main loop, if the function SimCore_ApoMsg_Eval()
**    skips the message
**
** Return:
**   0 : message evaluated
**  -1 : message not handled
*/

int
User_ApoMsg_Eval (int Ch, char *Msg, int len, int who)
{
#if defined(CM_HIL)
    /*** FailSafeTester */
    if (Ch == ApoCh_CarMaker) {
	if (FST_ApoMsgEval(Ch, Msg, len) <= 0)
	    return 0;
    }
#endif
    return -1;
}



/*
** User_ApoMsg_Send ()
**
** Communication between the application and connected GUIs.
** Sends messages to GUIs
**
** Call:
** - near the end of the main loop, in MainThread_FinishCycle()
** - pay attention to realtime condition
*/

void
User_ApoMsg_Send (double T, const unsigned CycleNo)
{
}



/*
** User_ShutDown ()
**
** Prepare application for shut down
**
** Call:
** - at end of program
** - no realtime conditions
*/

int
User_ShutDown (int ShutDownForced)
{
    int IsDown = 0;

    /* Prepare application for shutdown and return that
       shutdown conditions are reached */
    if (1) {
	IsDown = 1;
    }

    return IsDown;
}



/*
** User_End ()
**
** End all models of the user module
**
** Call:
** - one times at end of program
** - no realtime conditions
*/

int
User_End (void)
{
    return 0;
}



/*
** User_Cleanup ()
**
** Cleanup function of the User module
**
** Call:
** - one times at end of program, just before exit
** - no realtime conditions
*/

void
User_Cleanup (void)
{

}
