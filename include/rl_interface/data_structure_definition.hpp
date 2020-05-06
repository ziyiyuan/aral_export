#ifndef RL_DATA_STRUCTURE_DEFINITION_H
#define RL_DATA_STRUCTURE_DEFINITION_H

#include <stddef.h>

namespace ARAL {

#define CARTESIAN_FREEDOM 6
#define ROBOT_DOF 6
#define SENSOR_DIMENSION 6
#define FT_SENSOR_CALIB__NUM 3
#define CONTROL_PERIOD 0.005
#define READ_ONLY
#define MAX_IDEN_PARA_NUM 40  //maximum identification parameter number, all dh para;



typedef union EmcPose
{
    struct {
        struct {
            double x, y, z;
        } trans, rot;
        struct  {
            double w, x, y, z;
        } quant;
    };
    double val[10];
} EmcPose;

typedef struct _jointArray
{
    double mData[ROBOT_DOF];
    double& operator()(int i){return mData[i];}
    double operator()(int i)const{return mData[i];}
    double operator[] ( int i ) const{return this->operator() ( i );}
    double& operator[] ( int i ){return this->operator() ( i );}
    double *data(){return mData;}
    const double *data() const{return mData;}
    void setToZero()
    {
        for (size_t i = 0; i < ROBOT_DOF; i++)
            mData[i] = 0.;
    }
}JointArray;

typedef struct _cartArray
{
    double mData[CARTESIAN_FREEDOM];
    double& operator()(int i){return mData[i];}
    double operator()(int i)const{return mData[i];}
    double operator[] ( int i ) const{return this->operator() ( i );}
    double& operator[] ( int i ){return this->operator() ( i );}
    double *data(){return mData;}
    const double *data() const{return mData;}
    void setToZero()
    {
        for (size_t i = 0; i < CARTESIAN_FREEDOM; i++)
            mData[i] = 0.;
    }

    void setConstant(const double& v)
    {
        for (size_t i = 0; i < CARTESIAN_FREEDOM; i++)
            mData[i] = v;
    }
}CartArray;

typedef struct _vector
{
    double mData[3];
    double operator()(int i) const{return mData[i];}
    double& operator() (int i){return mData[i];}
    double operator[] ( int i ) const{return this->operator() ( i );}
    double& operator[] ( int i ){return this->operator() ( i );}
    double *data(){return mData;}
    const double *data() const{return mData;}
}Vector;

typedef struct _wrench
{
    union
    {
        struct
        {
            Vector force;
            Vector torque;
        };
        double mData[SENSOR_DIMENSION];
    };

    double& operator()(int i){return (i < 3)? force(i) : torque(i-3);}
    double operator()(int i) const{return (i < 3)? force(i) : torque(i-3);}
    double operator[] ( int i ) const{return this->operator() ( i );}
    double& operator[] ( int i ){return this->operator() ( i );}
    double *data(){return mData;}
    const double *data() const{return mData;}
    void setToZero()
    {
        for (size_t i = 0; i < SENSOR_DIMENSION; i++)
            mData[i] = 0.;
    }
    void setConstant(const double& v)
    {
        for (size_t i = 0; i < SENSOR_DIMENSION; i++)
            mData[i] = v;
    }
    struct _wrench operator+(const struct _wrench & lhs)
    {
        struct _wrench tmp;
        for(int i = 0; i < SENSOR_DIMENSION; i++)
            tmp[i] = lhs[i] + mData[i];

        return tmp;
    }
} Wrench;


enum {
    WP_TYPE_JOINT = (int)1,
    WP_TYPE_CART,
    WP_TYPE_BOTH
};

typedef struct WayPoint
{
    int type;
    JointArray q, qd, qdd;
    EmcPose pose;
} WayPoint;


enum {
    FORCE_POS_CONTROL = (int)0, //! 普通位置控制
    FORCE_ADM_CONTROL,          //! 导纳控制
    FORCE_IMP_CONTROL,          //! 阻抗控制
};

enum POSE_ARRAY_DESCRIPTION
{
    POS_ROTATION = (int)0,
    POS_RPY,
    POS_ZYZ,
    POS_ZYX,
    POS_QUATERNION
};

typedef struct _ftSensorCalibResult
{
    Wrench offset;
    Vector com;
    double mass;
    double angle[2];

}FtSensorCalibrationResult;

typedef struct _kinematicsCalibResult
{
    double all_d_para_[MAX_IDEN_PARA_NUM];

    double mean_value;
    double max_value;
    double rms_value;

}KinematicsCalibrationResult;

typedef struct _rigidBodyInertia
{
    double inertial[6];
    Vector com;
    double mass;

}RigidBodyInertia;
}


#endif
