#include <ros/ros.h>
#include <boost/format.hpp>

#include <chrono>


#include <morai_data_parser/data_parser.hpp>

#include <comm_msgs/vehicleStatus.h>
#include <comm_msgs/objsInfo.h>
#include <comm_msgs/gpsLocal.h>

ros::Publisher pubVeh;
ros::Publisher pubObjs;
ros::Publisher pubGPS;

class VehicleSensing{
public:
    VehicleSensing(){}

    void run(ego_vehicle_status *evs, objectInfo *oi){

        senseVehicleStatus(evs);
        senseObject(oi);
        senseLocal(evs);

        pubVeh.publish(this->vehicle);
        pubObjs.publish(this->objs);
        pubGPS.publish(this->maplocal);
    }

    void senseVehicleStatus(ego_vehicle_status *evs){

        this->vehicle.posX      = evs->posX;
        this->vehicle.posY      = evs->posY;
        this->vehicle.posZ      = evs->posZ;
        this->vehicle.roll      = evs->roll;
        this->vehicle.pitch     = evs->pitch;
        this->vehicle.yaw       = evs->yaw;
        this->vehicle.mode      = evs->ctrl_mode;
        this->vehicle.gear      = evs->gear;
        this->vehicle.speed     = evs->signed_velocity;
        this->vehicle.brake     = evs->brake;
        this->vehicle.steer     = evs->steer;
        
        this->vehicle.yaw       = convertHeadingE2NCW(this->vehicle.yaw); //CONVERT HEADING ENU(EAST = 0. CCW INCREASE) TO NED(NORTH = 0. CW INCREASE) 헤딩 변환

    }
        
    void senseObject(objectInfo *oi){

        this->objs.objNum = 0; 
        this->objs.data.clear();

        for(int i = 0; oi->data[i].objId != 0; ++i){
        // for(this->objs.objNum = 0; oi->data[this->objs.objNum].objId == 0; ++this->objs.objNum){

            comm_msgs::objInfo tmpObj;
            ++(this->objs.objNum);

            tmpObj.classes   = oi->data[i].objType;
            tmpObj.idx       = oi->data[i].objId;
            tmpObj.posX      = oi->data[i].posX;
            tmpObj.posY      = oi->data[i].posY;
            tmpObj.posZ      = oi->data[i].posZ;
            tmpObj.sizeX     = oi->data[i].sizeX;
            tmpObj.sizeY     = oi->data[i].sizeY;
            tmpObj.sizeZ     = oi->data[i].sizeZ;
            tmpObj.velX      = oi->data[i].velocityX;
            tmpObj.velY      = oi->data[i].velocityY;
            tmpObj.velZ      = oi->data[i].velocityZ;
            tmpObj.heading   = oi->data[i].heading;
            this->objs.data.push_back(tmpObj);

        }

        convertObj2VehCoor(true);
    }

    void senseLocal(ego_vehicle_status *evs){
        this->maplocal.longtitude   = evs->posX;
        this->maplocal.latitude     = evs->posY;
        this->maplocal.altitude     = evs->posZ;
        this->maplocal.timestamp    = ros::Time::now();
    }

    inline float convertHeadingE2NCW(float headingE){
        return ((90 - headingE) > 180) ? ( -270 - headingE) : (90 - headingE);
    }

    void convertObj2VehCoor(bool swch){ //true -> vehicle coordinate, false -> ENU coordinate
        if (swch == false ) return;

        float tmpX, tmpY, tmpZ;
        for(int i = 0; i < this->objs.objNum; i++){
            //translation
            tmpX = this->objs.data[i].posX - vehicle.posX;
            tmpY = this->objs.data[i].posY - vehicle.posY;
            tmpZ = this->objs.data[i].posZ - vehicle.posZ;

            //rotation
            this->objs.data[i].posX =  cos(vehicle.yaw * M_PI / 180) * tmpX + sin(vehicle.yaw * M_PI / 180) * tmpY;
            this->objs.data[i].posY = -sin(vehicle.yaw * M_PI / 180) * tmpX + cos(vehicle.yaw * M_PI / 180) * tmpY;
            this->objs.data[i].posZ = tmpZ;
        }
    }
    
private:

    comm_msgs::vehicleStatus vehicle;
    comm_msgs::objsInfo objs;
    comm_msgs::gpsLocal maplocal;

};



int main(int argc, char* argv[]){
    mdpar::EgoStatusParser rd(8909);
    mdpar::ObjectParser od(7505);
    VehicleSensing vs;

    ros::init(argc, argv, "morai_data_parser");
    ros::NodeHandle nh;

    pubVeh  = nh.advertise<comm_msgs::vehicleStatus> ("/morai_data_parser/vehicle_status", 1);
    pubObjs = nh.advertise<comm_msgs::objsInfo>      ("/morai_data_parser/objects", 1);
    pubGPS  = nh.advertise<comm_msgs::gpsLocal>      ("/morai_data_parser/gpsLocal", 1);
    
    while(ros::ok()){

        rd.recv();
        od.recv();
        rd.printData();
        od.printData();

        vs.run(rd.data, od.data);

        ros::spinOnce();
    }
}