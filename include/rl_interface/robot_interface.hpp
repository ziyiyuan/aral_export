#ifndef RL_ROBOT_INTERFACE_H
#define RL_ROBOT_INTERFACE_H

#include <string>
#include <vector>
#include "data_structure_definition.hpp"

namespace ARAL {

typedef struct
{
    //share memory data with robot server
    int enableForceControl;
    int startFlag;    // Set by user, read by algorithm
    int endFlag;       // Set by algorithm, read by user
    int mode;
    int errCode;
    double sigma;
    double curRefPose[ROBOT_DOF];
    double curRefVel[ROBOT_DOF];
    double curRefAcc[ROBOT_DOF];
    double curCurrent[ROBOT_DOF];
    double curJointPos[ROBOT_DOF];
    double curJointVel[ROBOT_DOF];
    double curJointAcc[ROBOT_DOF];
    double cmdJointPos[ROBOT_DOF];
    double cmdJointVel[ROBOT_DOF];
    double cmdJointAcc[ROBOT_DOF];
    double cmdCurrent[ROBOT_DOF];
}ForceControlData;

enum InterfaceFlag
{
    //InterfaceFlag: bit8 -> bit0
    Creat_Share_Memory = 0x01,  //bit0: for debug use
    LOAD_CONFIG_PARA   = 0x02,  //bit1: load default para
    LOG_DEBUG          = 0x00,  //bit3-4
    LOG_INFO           = 0x04,  //bit3-4
    LOG_WARN           = 0x08,  //bit3-4
    LOG_ERROR          = 0x0C,  //bit3-4
};

enum ReturnValue
{
    RET_SUCCESS = 0,
    ROBOT_DOF_DISMATCH = -100,
    FORCE_CONTROL_DISABLE = -101,
    INVALID_CONFIG_FILE = -102,
};

class RLIntface
{
public:
    /************ Robot Library Interface ************/
    /**
     * @brief Constructor, This constructor performs several procedures. It
     * 1)initializes all class attributes
     * 2)reads all parameters of the Robot system specified by URDF file using the class URDF_PARSER.
     * 3)creates the DataLogging object Logging, which provides the possibility of logging low-level control data under real-time conditions.
     * 4)In case,
     *          i)the initialization file specified by modelName cannot be opened
     *          ii)
     *   the constructor lets the calling process terminate and exits with a value of EXIT_FAILURE.
     * @param modelName: A pointer to an array of char values containing the path and filename of the URDF file
     * @param flag
     */
    RLIntface(const char * modelName, unsigned char flag = 0x00);
    /**
     * @brief Destructor: This destructor performs a set of procedures to cleanly shutdown the robot algorithm module
     *
     */
    ~RLIntface();
    /**
     * @brief Reads the initialization file.
     * @param configFileName: A pointer to an array of char containing the name of the file that provides the desired initialization values/parameters.
     * @return 1)The number of read parameter
     *         2)-1 if the file specified by InitFileName could not be opened
     */
    int initialRLInterface(const char * configFileName = "./config/force_control.ini");

    void setLogLevel(unsigned int level);

    int getLatestError(int& errCode, std::string& errDes);

    /************ Robot Model ************/
    void setGravityVectorOfRobotBase(double vecX, double vecY, double vecZ, int isVector = 1);
    //!
    int getRobotDOF(unsigned int dof);
    //!
    void setToolPose(const double *pose, int type = POS_RPY);
    //!
    void setToolInertial(const double m, const double* com, const double * inertial);
    //!
    void setToolInertialFromFTSensor(const double m, const double* com, const double * inertial);
    //!
    void setEndSensorPose(double * pose, int type = POS_RPY);       // place here for temporary
    //!
    void setEndSensorInertial(const double m, const double* com, const double * inertial);       // place here for temporary


    /************ Robot Status ************/
    //! flag: bit8 -> bit0
    //! bit0: current position feedback : joint space
    //! bit1: current current feedback
    //! bit2: current velocity feedback : joint space
    //! bit3: current ref velocity feedin : cart space
    //! bit4: current ref acceleration feedin : cart space
    //! bit5: target velocity output : joint space
    //! bit6: target acceleration output : joint space
    void setFeedBackOptions(const unsigned char flag);

    /**
     * @brief Reads the measured joint position vector from the latest data telegram of the controller.
     * @param q: A pointer to an array of double values; the array has to have at least a size of DOF elements. The measured joint position vector is written into this array.
     * @param qd: Get the parameter from the Joint State Estimation module. If not specified the algorithm will do it.
     * @param qdd: Get the parameter from the Joint State Estimation module. If not specified the algorithm will do it.
     */
    void updatJointPVAStatus(const double* q, const double* qd = NULL, const double* qdd = NULL);
    //!
    void updateBaseFTSensorData(const Wrench& ftData);
    //!
    void updateEndFTSensorData(const Wrench& ftData);
    //!
    void updateEndFTSensorData(const double* ftData);
    /**
     * @brief Reads the measured joint torque vector from the latest data telegram of the controller.
     * @param torqueData: A pointer to an array of double values; the array has to have at least a size of DOF elements. The measured joint torque vector is written into this array.
     */
    void updateJointTorqueSensorData(const JointArray& torqueData);

    //! set the reference trajectory
    //! type: 0 -> joint space; 1 -> cartesian space
    void setRefTraj(const double* positions, const double* velocities = NULL, const double* accelerations = NULL, unsigned int type = 0);

    //!
    void updateRobotStatus(const double* curPos, const double* tarPos, unsigned int type = 0);

    /**
     * @brief Reads the commanded joint position/torque vector from the latest data telegram of algorithm.
     * @param res: A pointer to an array of double values; the array has to have at least a size of DOF elements. The commanded joint position/torque vector is written into this array
     * @param qd: If specified, then it can be used to realize the joint velocity feedforward control.
     * @param qdd: If specified, then it can be used to realize the dynamics feedforward control.
     * @param type: 0:for admittance control-> output: q, qd, qdd
     *              1:for impedance control-> output: joint torque
     * @return return the status of the calculation process.
     */
    int calJointCommand(double* res, double* qd = NULL, double *qdd = NULL, int type = 0);

    /************ Motion Control ************/
    void setControlPeriod(const double period);
    double getControlPeriod();

    //!
    int setControlType(int type);
    int getControlType();

    void setMaxTranSpeed(double vel);
    double getMaxTranSpeed();

    void setMaxRotSpeed(double rot);
    double getMaxRotSpeed();

    /************ Force Control ************/
    void enableForceContol(bool flag);

    void setSelectMatrix(const double *value);
    const double *getSelectMatrix();

    void setCalThread(unsigned int type);

    void setCalMethod(unsigned int type);       // for test only

    void setDragMode(unsigned int type);

    void setForceControlMode(unsigned int value);
    unsigned int getForceControlMode();

    void setForceControlSpace(unsigned int space);

    void setSensorFilter1(const double value);

    void setSensorFilter2(const double value);

    void setEndFTSensorThreshold(double data[SENSOR_DIMENSION]);
    const double *getEndFTSensorThreshold();

    void setEndFTSensorLimit(double data[SENSOR_DIMENSION]);
    const double * getEndFTSensorLimit();

    int setCartStiffness(double data[CARTESIAN_FREEDOM]);
    const double * getCartStiffness();


    int setCartDamp(double data[CARTESIAN_FREEDOM]);
    const double * getCartDamp();

    int setCartMass(double data[CARTESIAN_FREEDOM]);
    const double * getCartMass();

    void setOverEstimatedDis(const double dis);

    void enableSingularityConsistent(bool flag);

    void setConstrainPara(const double* value);

    READ_ONLY void getRobotEndWrench(double * wrench);

    void setGoalWrench(const double* value);

    //! set the goal wrench
    int setWrenchTarget(EmcPose *frame, EmcPose *sel_vec, Wrench *wrench, int type, EmcPose *limits);

    /************ Calibration Module ************/
    int calibToolAndSensor(const JointArray joints[FT_SENSOR_CALIB__NUM], const Wrench measurement[FT_SENSOR_CALIB__NUM], FtSensorCalibrationResult& result);

    int calibRobotKinematicsPara(std::vector<std::vector<double> > measureData, std::vector<std::vector<double> > jointAngle,
                                 bool selectionVector[], unsigned int type, KinematicsCalibrationResult& result);

    /**
     * @brief Workpiece Coordinate calibration, robot's real tool should be set before call this function
     * @param jointAngle
     * @param type
     * @param result
     * @return
     */
    int calibWorkpieceCoordinatePara(const std::vector<std::vector<double> > &jointAngle, unsigned int type, double *result);

    /************ Custom Module ************/
    int calTrajectoryTrackingCommand(const double* jointPos, const double* sensorData, /*const double* refPos,*/ double* cmdPos);

public:

};

}
#endif // ROBOT_MODEL_H
