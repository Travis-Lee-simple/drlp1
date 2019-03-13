#include "robotcontrol.h"


#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>

#define M_PI 3.14159265358979323846

#define ROBOT_ADDR "192.168.1.100"
#define ROBOT_PORT 8899



#define userName "Aubo"
#define possword "123456"

using namespace aubo_robot_namespace;

//¿ØÖÆ¹ñÓÃ»§£Ä£ÉÃû³Æ
const char* USER_DI_00 = "U_DI_00";
const char* USER_DI_01 = "U_DI_01";
const char* USER_DI_02 = "U_DI_02";
const char* USER_DI_03 = "U_DI_03";
const char* USER_DI_04 = "U_DI_04";
const char* USER_DI_05 = "U_DI_05";
const char* USER_DI_06 = "U_DI_06";
const char* USER_DI_07 = "U_DI_07";
const char* USER_DI_10 = "U_DI_10";
const char* USER_DI_11 = "U_DI_11";
const char* USER_DI_12 = "U_DI_12";
const char* USER_DI_13 = "U_DI_13";
const char* USER_DI_14 = "U_DI_14";
const char* USER_DI_15 = "U_DI_15";
const char* USER_DI_16 = "U_DI_16";
const char* USER_DI_17 = "U_DI_17";

//¿ØÖÆ¹ñÓÃ»§£Ä£ÏÃû³Æ
const char* USER_DO_00 = "U_DO_00";
const char* USER_DO_01 = "U_DO_01";
const char* USER_DO_02 = "U_DO_02";
const char* USER_DO_03 = "U_DO_03";
const char* USER_DO_04 = "U_DO_04";
const char* USER_DO_05 = "U_DO_05";
const char* USER_DO_06 = "U_DO_06";
const char* USER_DO_07 = "U_DO_07";
const char* USER_DO_10 = "U_DO_10";
const char* USER_DO_11 = "U_DO_11";
const char* USER_DO_12 = "U_DO_12";
const char* USER_DO_13 = "U_DO_13";
const char* USER_DO_14 = "U_DO_14";
const char* USER_DO_15 = "U_DO_15";
const char* USER_DO_16 = "U_DO_16";
const char* USER_DO_17 = "U_DO_17";

const char* TOOL_IO_0 = "T_DI/O_00";
const char* TOOL_IO_1 = "T_DI/O_01";
const char* TOOL_IO_2 = "T_DI/O_02";
const char* TOOL_IO_3 = "T_DI/O_03";



void printRoadPoint(aubo_robot_namespace::wayPoint_S  *wayPoint)
{
	std::cout<<"pos.x="<<wayPoint->cartPos.position.x<<std::endl;
	std::cout<<"pos.y="<<wayPoint->cartPos.position.y<<std::endl;
	std::cout<<"pos.z="<<wayPoint->cartPos.position.z<<std::endl;

	std::cout<<"ori.w="<<wayPoint->orientation.w<<std::endl;
	std::cout<<"ori.x="<<wayPoint->orientation.x<<std::endl;
	std::cout<<"ori.y="<<wayPoint->orientation.y<<std::endl;
	std::cout<<"ori.z="<<wayPoint->orientation.z<<std::endl;

	//¹Ø½Ú½Ç¶ÈÐÅÏ¢
	std::cout<<"joint_1="<<wayPoint->jointpos[0]*180.0/M_PI<<std::endl;
	std::cout<<"joint_2="<<wayPoint->jointpos[1]*180.0/M_PI<<std::endl;
	std::cout<<"joint_3="<<wayPoint->jointpos[2]*180.0/M_PI<<std::endl;
	std::cout<<"joint_4="<<wayPoint->jointpos[3]*180.0/M_PI<<std::endl;
	std::cout<<"joint_5="<<wayPoint->jointpos[4]*180.0/M_PI<<std::endl;
	std::cout<<"joint_6="<<wayPoint->jointpos[5]*180.0/M_PI<<std::endl;
}

void callback_RealTimeRoadPoint(aubo_robot_namespace::wayPoint_S  *wayPoint, void *arg)
{
	printRoadPoint(wayPoint);
}


//pos Ä¿±êÎ»ÖÃx,y,z µ¥Î»Ã×
//joint6Angle 6Öá½Ç¶È(¶È)


bool auboi5_login(const char * addr, int port)
{
	ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(ROBOT_ADDR, ROBOT_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录成功."<<std::endl;
    }
    else
    {
        std::cerr<<"登录failed."<<std::endl;
    }

    robotService.robotServiceLogout();
}

void auboi5_logout()
{
	ServiceInterface robotService;
	robotService.robotServiceLogout();
}


void auboi5_robotStartup()
{
	ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(ROBOT_ADDR, ROBOT_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录成功."<<std::endl;
    }
    else
    {
        std::cerr<<"登录成功."<<std::endl;
    }


    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"机械臂初始化成功."<<std::endl;
    }
    else
    {
        std::cerr<<"机械臂初始化失败."<<std::endl;
    }
    robotService.robotServiceLogout();
}


/*
void auboi5_robotShutdown()
{
	ServiceInterface robotService;
	
	robotService.robotServiceRobotShutdown();
}*/


/*
int auboi5_callbackRobotRoadPoint()
{
	bool result = false;
	ServiceInterface robotService;

	//ÔÊÐíÊµÊ±Â·µãÐÅÏ¢ÍÆËÍ
	if (ErrnoSucc == robotService.robotServiceSetRealTimeRoadPointPush(true))
	{
		if (ErrnoSucc == rs_setcallback_realtime_roadpoint(rshd, callback_RealTimeRoadPoint, NULL))
		{
			result = true;
		}
		else
		{
			std::cerr<<"call rs_setcallback_realtime_roadpoint failed"<<std::endl;
		}
	}
	else
		std::cerr<<"call rs_enable_push_realtime_roadpoint failed!"<<std::endl;

	return result;

}


*/


void auboi5_movetosetposition(double x, double y, double z, double rx, double ry, double rz)
{


	ServiceInterface robotService;

    aubo_robot_namespace::wayPoint_S targetPoint;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    double startPointJointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    startPointJointAngle[0] = 0.0/180.0*M_PI;
    startPointJointAngle[1] = 0.0/180.0*M_PI;
    startPointJointAngle[2] = 90.0/180.0*M_PI;
    startPointJointAngle[3] = 0.0/180.0*M_PI;
    startPointJointAngle[4] = 0.0/180.0*M_PI;
    startPointJointAngle[5] = 0.0/180.0*M_PI;
    //Util::initJointAngleArray(startPointJointAngle, 0.0/180.0*M_PI,  0.0/180.0*M_PI,  0.0/180.0*M_PI, 0.0/180.0*M_PI, 0.0/180.0*M_PI,0.0/180.0*M_PI);

    aubo_robot_namespace::Pos targetPosition;
    targetPosition.x = x;
    targetPosition.y = y;
    targetPosition.z = z;

    aubo_robot_namespace::Rpy rpy;
    aubo_robot_namespace::Ori targetOri;

    rpy.rx = rx/180.0*M_PI;
    rpy.ry = ry/180.0*M_PI;
    rpy.rz = rz/180.0*M_PI;

    robotService.RPYToQuaternion(rpy, targetOri);

    ret = robotService.robotServiceRobotIk(startPointJointAngle, targetPosition, targetOri, targetPoint);

    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        printRoadPoint(&targetPoint);
    }
    else
    {
        std::cerr<<"调用逆解函数失败"<<std::endl;
    }

    

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(ROBOT_ADDR, ROBOT_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录成功."<<std::endl;
    }
    else
    {
        std::cerr<<"登录失败."<<std::endl;
    }

    /** 运动到初始位姿 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    jointAngle[0] = targetPoint.jointpos[0];
    jointAngle[1] = targetPoint.jointpos[1];
    jointAngle[2] = targetPoint.jointpos[2];
    jointAngle[3] = targetPoint.jointpos[3];
    jointAngle[4] = targetPoint.jointpos[4];
    jointAngle[5] = targetPoint.jointpos[5];
    ret = robotService.robotServiceJointMove(jointAngle, true);   //关节运动至准备点
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动0失败.　ret:"<<ret<<std::endl;
    }

    robotService.robotServiceLogout();
}


void auboi5_move_bytool(const Pos *pos_onuser, const Ori ori_onuser, 
	const CoordCalibrateByJointAngleAndTool *user_coord, 
	const ToolInEndDesc *tool_pos, 
	 Pos *pos_onbase,  Ori ori_onbase)
{
	ServiceInterface robotService;
	robotService.userToBaseCoordinate(*pos_onuser,    //基于用户座标系的工具末端位置信息
                                     ori_onuser, //基于用户座标系的工具末端姿态信息
                                     *user_coord,  //用户坐标系
                                     *tool_pos,                 //工具信息
                                     *pos_onbase,    //基于基座标系的法兰盘中心位置信息
                                     ori_onbase //基于基座标系的法兰盘中心姿态信息
                                     );



    aubo_robot_namespace::wayPoint_S startwayPoint;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    robotService.robotServiceGetCurrentWaypointInfo(startwayPoint);

    double startPoint[aubo_robot_namespace::ARM_DOF] = {0};
    startPoint[0] = startwayPoint.jointpos[0]/180.0*M_PI;
    startPoint[1] = startwayPoint.jointpos[1]/180.0*M_PI;
    startPoint[2] = startwayPoint.jointpos[2]/180.0*M_PI;
    startPoint[3] = startwayPoint.jointpos[3]/180.0*M_PI;
    startPoint[4] = startwayPoint.jointpos[4]/180.0*M_PI;
    startPoint[5] = startwayPoint.jointpos[5]/180.0*M_PI;

    aubo_robot_namespace::wayPoint_S targetPoint;

    ret = robotService.robotServiceRobotIk(startPoint, *pos_onbase, ori_onbase, targetPoint);

    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        printRoadPoint(&targetPoint);
    }
    else
    {
        std::cerr<<"调用逆解函数失败"<<std::endl;
    }

    

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(ROBOT_ADDR, ROBOT_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录成功."<<std::endl;
    }
    else
    {
        std::cerr<<"登录失败."<<std::endl;
    }

    /** 运动到初始位姿 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    jointAngle[0] = targetPoint.jointpos[0];
    jointAngle[1] = targetPoint.jointpos[1];
    jointAngle[2] = targetPoint.jointpos[2];
    jointAngle[3] = targetPoint.jointpos[3];
    jointAngle[4] = targetPoint.jointpos[4];
    jointAngle[5] = targetPoint.jointpos[5];
    ret = robotService.robotServiceJointMove(jointAngle, true);   //关节运动至准备点
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动0失败.　ret:"<<ret<<std::endl;
    }

    robotService.robotServiceLogout();
}


/*
void auboi5_moveJ()
{
	double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    jointAngle[0] = 0.0/180.0*M_PI;
    jointAngle[1] = 0.0/180.0*M_PI;
    jointAngle[2] = 90.0/180.0*M_PI;
    jointAngle[3] = 0.0/180.0*M_PI;
    jointAngle[4] = 90.0/180.0*M_PI;
    jointAngle[5] = 0.0/180.0*M_PI;
    ret = robotService.robotServiceJointMove(jointAngle, true);
}
*/



void auboi5_moveL()
{
	ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(ROBOT_ADDR, ROBOT_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录成功."<<std::endl;
    }
    else
    {
        std::cerr<<"登录失败."<<std::endl;
    }

    /** 运动到初始位姿 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    jointAngle[0] = 0.0/180.0*M_PI;
    jointAngle[1] = 0.0/180.0*M_PI;
    jointAngle[2] = 90.0/180.0*M_PI;
    jointAngle[3] = 0.0/180.0*M_PI;
    jointAngle[4] = 90.0/180.0*M_PI;
    jointAngle[5] = 0.0/180.0*M_PI;
    ret = robotService.robotServiceJointMove(jointAngle, true);   //关节运动至准备点
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动0失败.　ret:"<<ret<<std::endl;
    }

    robotService.robotServiceLogout();
}




void auboi5_setpose(double x, double y, double z, double rx, double ry, double rz) 
{
	auboi5_movetosetposition(x, y, z, rx, ry, rz);
}




void auboi5_backtozero(double joint_1, double joint_2, double joint_3, double joint_4, double joint_5, double joint_6)
{
	ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(ROBOT_ADDR, ROBOT_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录成功."<<std::endl;
    }
    else
    {
        std::cerr<<"登录失败."<<std::endl;
    }

    /** 运动到初始位姿 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    jointAngle[0] = joint_1/180.0*M_PI;
    jointAngle[1] = joint_2/180.0*M_PI;
    jointAngle[2] = joint_3/180.0*M_PI;
    jointAngle[3] = joint_4/180.0*M_PI;
    jointAngle[4] = joint_5/180.0*M_PI;
    jointAngle[5] = joint_6/180.0*M_PI;
    ret = robotService.robotServiceJointMove(jointAngle, true);   //关节运动至准备点
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动0失败.　ret:"<<ret<<std::endl;
    }

    robotService.robotServiceLogout();
}



void auboi5_setmaxacc(double joint1_maxacc, double joint2_maxacc, double joint3_maxacc, double joint4_maxacc, double joint5_maxacc, double joint6_maxacc)
{
	ServiceInterface robotService;
	robotService.robotServiceInitGlobalMoveProfile();

    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = joint1_maxacc/180.0*M_PI;
    jointMaxAcc.jointPara[1] = joint2_maxacc/180.0*M_PI;
    jointMaxAcc.jointPara[2] = joint3_maxacc/180.0*M_PI;
    jointMaxAcc.jointPara[3] = joint4_maxacc/180.0*M_PI;
    jointMaxAcc.jointPara[4] = joint5_maxacc/180.0*M_PI;
    jointMaxAcc.jointPara[5] = joint6_maxacc/180.0*M_PI;   //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    robotService.robotServiceLogout();
}



void auboi5_setmaxvelc(double joint1_maxvelc, double joint2_maxvelc, double joint3_maxvelc, double joint4_maxvelc, double joint5_maxvelc, double joint6_maxvelc)
{
    ServiceInterface robotService;
	robotService.robotServiceInitGlobalMoveProfile();

	aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = joint1_maxvelc/180.0*M_PI;
    jointMaxVelc.jointPara[1] = joint2_maxvelc/180.0*M_PI;
    jointMaxVelc.jointPara[2] = joint3_maxvelc/180.0*M_PI;
    jointMaxVelc.jointPara[3] = joint4_maxvelc/180.0*M_PI;
    jointMaxVelc.jointPara[4] = joint5_maxvelc/180.0*M_PI;
    jointMaxVelc.jointPara[5] = joint6_maxvelc/180.0*M_PI;   //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    robotService.robotServiceLogout();
}




void auboi5_setproperty(double * joint_maxvelc, double * joint_maxacc)
{
	auboi5_setmaxvelc(joint_maxvelc[0], joint_maxvelc[1], joint_maxvelc[2], joint_maxvelc[3], joint_maxvelc[4], joint_maxvelc[5]);
	auboi5_setmaxacc(joint_maxacc[0], joint_maxacc[1], joint_maxacc[2], joint_maxacc[3], joint_maxacc[4], joint_maxacc[5]);
}




int auboi5_movebysetway(int num)
{
	//ÒÆ¶¯·½Ê½£ºÎ»×ËÄ³Ò»Î¬¸üÐÂ»òÎ»×ËÈ«²¿¸üÐÂ
	int move_type;

	//ÒÆ¶¯·½Ê½ÊäÈëÊÇ·ñÓÐÐ§
	bool type_valid = false;

	//Ö¸¶¨Ä¿±êÎ»×Ë
	double p[6];

	//ÊäÈë±äÁ¿
	double x;
	
	cout << "ÇëÊäÈëÒÆ¶¯µÄÎ¬¶È£º" << endl;
	cout << "x:1  y:2  z:3  rx:4  ry:5  rz:6  all:7" << endl;
	cout << "ÇëÊäÈë£º";
	cin >> move_type;

	switch (move_type) {
	case 1:
		cout << "ÊäÈëÄ¿±êÎ»×ËxµÄÖµ£º";
		cin >> x;
		p[0] = x;
		type_valid = true;
		break;
	case 2:
		cout << "ÊäÈëÄ¿±êÎ»×ËyµÄÖµ£º";
		cin >> x;
		p[1] = x;
		type_valid = true;
		break;
	case 3:
		cout << "ÊäÈëÄ¿±êÎ»×ËzµÄÖµ£º";
		cin >> x;
		p[2] = x;
		type_valid = true;
		break;
	case 4:
		cout << "ÊäÈëÄ¿±êÎ»×ËrxµÄÖµ£º";
		cin >> x;
		p[3] = x;
		type_valid = true;
		break;
	case 5:
		cout << "ÊäÈëÄ¿±êÎ»×ËryµÄÖµ£º";
		cin >> x;
		p[4] = x;
		type_valid = true;
		break;
	case 6:
		cout << "ÊäÈëÄ¿±êÎ»×ËrzµÄÖµ£º";
		cin >> x;
		p[5] = x;
		type_valid = true;
		break;
	case 7:
		cout << "ÇëÊäÈëÄ¿±êÎ»×Ë£º" << endl;
		for (int i = 0; i < 6; i++) {
			cin >> p[i];
		}
		type_valid = true;
		break;
	default:
		cout << "ÊäÈëÎÞÐ§£¡ÇëÊäÈëÒÆ¶¯·½Ê½±àºÅ¡®1¡ª6¡¯" << endl;
		type_valid = false;
	}
	if (type_valid == true) {
		auboi5_movetosetposition(p[0], p[1], p[2], p[3], p[4], p[5]);
		num++;
	}
	return num;
}


void auboi5_printcurrentpose(aubo_robot_namespace::wayPoint_S  *wayPoint)
{
    ServiceInterface robotService;

    cout << "当前法兰盘末端相对于基坐标系的位姿是：" << endl;
	std::cout << " x =" << wayPoint->cartPos.position.x << std::endl;
	std::cout << " y =" << wayPoint->cartPos.position.y << std::endl;
	std::cout << " z =" << wayPoint->cartPos.position.z << std::endl;

	aubo_robot_namespace::Rpy rpy;
	robotService.quaternionToRPY(wayPoint->orientation, rpy);

	std::cout << "rx =" << rpy.rx / M_PI * 180 << std::endl;
	std::cout << "ry =" << rpy.ry / M_PI * 180 << std::endl;
	std::cout << "rz =" << rpy.rz / M_PI * 180 << std::endl;

	robotService.robotServiceLogout();
}

void auboi5_printcurrentpose_bytool(aubo_robot_namespace::Pos *pos, aubo_robot_namespace::Ori ori)
{		
	ServiceInterface robotService;
	cout << "当前工具末端点相对于基坐标系的位姿是："<<endl;
	std::cout << " x =" << pos->x << std::endl;
	std::cout << " y =" << pos->y << std::endl;
	std::cout << " z =" << pos->z << std::endl;
    

    aubo_robot_namespace::Rpy rpy;
	robotService.quaternionToRPY(ori, rpy);

	std::cout << "rx =" << rpy.rx / M_PI * 180 << std::endl;
	std::cout << "ry =" << rpy.ry / M_PI * 180 << std::endl;
	std::cout << "rz =" << rpy.rz / M_PI * 180 << std::endl;

	robotService.robotServiceLogout();
}

void auboi5_euler_to_quaternion(double x, double y, double z, double rx, double ry, double rz,
	 aubo_robot_namespace::Pos *pos, aubo_robot_namespace::Ori ori)
{
	//位置简单赋值就可以
	pos->x = x;
	pos->y = y;
	pos->z = z;

	//姿态要从欧式空间的角度，转变为四元数
    //先定义一个欧式空间的角度类的对象
	aubo_robot_namespace::Rpy rpy;
	rpy.rx = rx / 180.0*M_PI;
	rpy.ry = ry / 180.0*M_PI;
	rpy.rz = rz / 180.0*M_PI;  

	//定义Ori用来存储转换后的四元数

	ServiceInterface robotService;
    robotService.RPYToQuaternion(rpy, ori);	

    robotService.robotServiceLogout();
}

