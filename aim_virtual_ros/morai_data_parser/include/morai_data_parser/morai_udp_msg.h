#ifndef MORAI_UDP_MSG_H
#define MORAI_UDP_MSG_H

#pragma pack(push, 1) //to read packet correctly
struct ctrl_msg{
    double auto_manual ;
    double brake;
    double estop;
    double gear;
    double speed;
    double steer;
    double tmp1;
    double tmp2;
    double tmp3;
    double tmp4;
};

struct ego_vehicle_cmd{

    char prefix[14];
    unsigned int data_length;
    char aux_data[12];

    //data
    char ctrl_mode;
    char gear;
    char cmdtype;
    
    float velocity;
    float acceleration;
    float accelerator;
    float brake;
    float steering;

    char zeroD;
    char zeroA;
};

struct ego_vehicle_status{

    char sharp;
    char MaraiInfo[9];
    char dollor;
    unsigned int data_length;
    char aux_data[12];

    //data
    char ctrl_mode;
    char gear;
    float signed_velocity;
    unsigned int map_data_id;
    float accel;
    float brake;
    float sizeX;
    float sizeY;
    float sizeZ;
    float overhang;
    float wheelbase;
    float rear_overhang;


    //data
    float posX;
    float posY;
    float posZ;
    float roll;
    float pitch;
    float yaw;
    float velocityX;
    float velocityY;
    float velocityZ;
    float accelX;
    float accelY;
    float accelZ;
    float steer;
    char linkID[38];

    char zeroD;
    char zeroA;
};

struct objectInfo_data{
    unsigned short objId;
    short objType;
    float posX;
    float posY;
    float posZ;
    float heading;
    float sizeX;
    float sizeY;
    float sizeZ;
    float overhang;
    float wheelbase;
    float rear_overhang;
    float velocityX;
    float velocityY;
    float velocityZ;
    float accelX;
    float accelY;
    float accelZ;
    char linkID[38];
};

struct objectInfo{

    char sharp;
    char MoraiObjInfo[12];
    char dollor;
    unsigned int data_length;
    char aux_data[12];

    //data
    struct objectInfo_data data[20];

    char zeroD;
    char zeroA;
};
#pragma pack(pop)

#endif