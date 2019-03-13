#pragma once

#include <string>
#include "serviceinterface.h"
#include "AuboRobotMetaType.h"


using namespace std;
using namespace aubo_robot_namespace;


//×ÚÉêÏîÄ¿ÐèÇó´úÂë½è¿Úº¯Êý

//µÇÂ½»úÐµ±Û
bool auboi5_login(const char * addr, int port);

//ÍË³öµÇÂ½
void auboi5_logout();

//Æô¶¯»úÐµ±Û(±ØÐëÁ¬½ÓÕæÊµ»úÐµ±Û£©
void auboi5_robotStartup();

//¹Ø±Õ»úÐµ±Û£¨±ØÐëÁ¬½ÓÕæÊµ»úÐµ±Û£©
void auboi5_robotShutdown();

//»úÐµ±ÛÖá¶¯²âÊÔ
void auboi5_moveJ();

//»úÐµ±Û±£³Öµ±Ç°×ËÌ¬Ö±ÏßÔË¶¯²âÊÔ
void auboi5_moveL();

//ÊµÊ±Â·µãÐÅÏ¢»Øµ÷º¯Êý²âÊÔ
int auboi5_callbackRobotRoadPoint();

//»úÐµ±ÛÒÆ¶¯µ½Ö¸¶¨Î»×Ë
void auboi5_movetosetposition(double x, double y, double z, double rx, double ry, double rz);

//»úÐµ±ÛÄ¿±êÎ»×ËÉèÖÃ
void auboi5_setpose(double x, double y, double z, double rx, double ry, double rz);

//»úÐµ±Û»Øµ½³õÊ¼Î»×Ë
void auboi5_backtozero(double joint_1, double joint_2, double joint_3, double joint_4, double joint_5, double joint_6);

//»úÐµ±ÛÉèÖÃ¹Ø½ÚÊôÐÔ
void auboi5_setproperty(double * joint_maxvelc, double * joint_maxacc);

//»úÐµ±ÛÉèÖÃ¸÷¹Ø½Ú×î´ó¼ÓËÙ¶È
void auboi5_setmaxacc(double joint1_maxacc, double joint2_maxacc, double joint3_maxacc, double joint4_maxacc, double joint5_maxacc, double joint6_maxacc);

//»úÐµ±ÛÉèÖÃ¸÷¹Ø½Ú×î´óËÙ¶È
void auboi5_setmaxvelc(double joint1_maxvelc, double joint2_maxvelc, double joint3_maxvelc, double joint4_maxvelc, double joint5_maxvelc, double joint6_maxvelc);

//»úÐµ±Û°´ÕÕÓÃ»§ÊäÈëÔÚËæÒâÖ¸¶¨µÄ·½ÏòÉÏÒÆ¶¯
int auboi5_movebysetway(int number);

//»úÐµ±Û´òÓ¡µ±Ç°Î»×Ë
void auboi5_printcurrentpose(aubo_robot_namespace::wayPoint_S  *wayPoint);

void auboi5_printcurrentpose_bytool(aubo_robot_namespace::Pos *pos, aubo_robot_namespace::Ori ori);

//机械臂移动，使得工具末端到达指定位姿
void auboi5_move_bytool(const Pos *pos_onuser, const Ori ori_onuser, 
	const CoordCalibrateByJointAngleAndTool *user_coord, 
	const ToolInEndDesc *tool_pos, 
	Pos *pos_onbase,  Ori ori_onbase);

//欧式空间六维位姿转换为三维位置及四元数
void auboi5_euler_to_quaternion(double x, double y, double z, double rx, double ry, double rz, aubo_robot_namespace::Pos *pos, aubo_robot_namespace::Ori ori);





 