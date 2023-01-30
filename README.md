# MORAI-DATA-PARSER
UDP IP 통신을 통한 상태 수신 시스템

## How to Use
1. respository 내용 전체를 catkin_ws/src에 다운로드
1. catkin 빌드 시스템을 통해서 'comm_msgs, morai-data-parser' 2개의 패키지 build
1. Morai 내 출력 포트 설정
    
    ![morai_nerwork_setting](/readmeImg/morai_nerwork_setting.png)
        
        Ego Vehicle Status Destination Port     = 8909
        Object Info Destination Port            = 7505
        
1. morai-data-parser 패키지 구동

        rosrun morai_data_parser morai_data_parser_node

    ![node_execute](/readmeImg/node_execute.png)

1. parsing 된 data는 rostopic으로 발행된다.

        /morai_data_parser/vehicle_status
        /morai_data_parser/objects
        /morai_data_parser/gps_local

        
    [vehicle_status 정보 실행 예](/readmeImg/vehicle_status_execute.png)
    
    [주변 객체 정보 실행 예](/readmeImg/objs_execute.png)
    
    [gps_local 정보 실행 예](/readmeImg/gps_local_execute.png)

1. 발행된 topic 정보를 subscribe 하여 이용

    
----


## Generate ROS Message
|Message|발행 메시지 정보|File|
|------|---|---|
|vehicleStatus|차량 상태 정보 (속도, 가속도, Accel, Brake, Gear 등)|[vehicleStatus.msg](/aim_virtual_ros/comm_msgs/msg/vehicleStatus.msg)|
|objInfo|단일 객체 상태 정보 (위치<차량 기준>, 크기, index, class, 속도 등)|[objInfo.msg](/aim_virtual_ros/comm_msgs/msg/objInfo.msg)|
|objㄴInfo|전체 객체 수, 각각의 객체 정보 vector|[objsInfo.msg](/aim_virtual_ros/comm_msgs/msg/objsInfo.msg)|
|gpsLocal|위도, 경도, 고도, timestamp 정보|[gpsLocal.msg](/aim_virtual_ros/comm_msgs/msg/gpsLocal.msg)|


## 비고
* 차량 위치 정보는 Map의 특정 위치 기준 ENU coordinate을 따름
* **터미널에 출력되는 정보와 topic 발행 정보는 아래와 같은 이유로 다를 수 있음**
    * 차량 heading 정보는 ENU -> NED로 변환하여 topic으로 발행 (NED = North = 0deg, CW increase)
    * 주변 객체 정보는 차량 좌표계 기준으로 변환하여 topic으로 발행
    * 주변 객체의 heading은 ENU coordinate 기준


