#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt16MultiArray.h>
#include <common/Encoder.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <boost/assign/list_of.hpp>

using namespace std;

#define SERVPORT 	50000

#define HEADSIZE 		4
#define DATASIZE 		80
#define TAILSIZE 			3

#define HEADER 				0xFA
#define IMU_ID 				0x10
#define ENCODER_ID 		0x11
#define SONAR_ID 			0x12
#define VEL_ID 				0x13
#define RESET_ID 			0x14

#define IMU_DATA_LEN 				(40+1)
#define ENCODER_DATA_LEN 		(16+1)
#define SONAR_DATA_LEN 			(24+1)

#define Q30 1073741824.0

typedef unsigned char uchar;

static int sockfd;
static int confd;
static volatile uchar tcp_state_ok = 0;

static ros::Publisher encoder_pub;
static ros::Publisher sonar_pub;
static ros::Publisher imu_pub;
static ros::Subscriber smooth_sub;
static ros::Subscriber enco_reset_sub;

static common::Encoder motor_encoder;
static std_msgs::UInt16MultiArray sonar_data;	
static sensor_msgs::Imu imu_data;

static uchar buf_geom[16]= {0};
static uchar buf_reset_enc[8]= {0xFD, 0xFE, 0xFF, 1, RESET_ID, 0xFA, 0xFB, 0xFC};

static void init_send_buff(void)
{
	buf_geom[0] = 0xFD, buf_geom[1] = 0xFE, buf_geom[2] = 0xFF;
	buf_geom[3] = 1+8, buf_geom[4] = VEL_ID;
	buf_geom[13] = 0xFA, buf_geom[14] = 0xFB, buf_geom[15] = 0xFC;
}

static void smoother_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
	if(tcp_state_ok)
	{
		int linear_x = (msg->linear.x)*1000;
		int angular_z = (msg->angular.z)*1000;

        // ROS_INFO("l_x: %d, a_z: %d", linear_x, angular_z);
		
        memcpy(&buf_geom[5], &linear_x, sizeof(linear_x));
		memcpy(&buf_geom[9], &angular_z, sizeof(angular_z));

		send(confd, buf_geom, sizeof(buf_geom), 0);
	}	
}

static void encoder_reset_callback(const std_msgs::Empty::ConstPtr &msg)
{
	if(tcp_state_ok)
	{
        ROS_INFO("encoder_reset");
		send(confd, buf_reset_enc, sizeof(buf_reset_enc), 0);
	}
}

static uchar parse_stm_data(uchar cmd, const uchar data[], uchar len)
{
	switch (cmd)
	{
		case IMU_ID:
		{
			if(len != IMU_DATA_LEN)
				return 0;
				
			imu_data.header.stamp = ros::Time::now();
			imu_data.header.frame_id = "imu";
			//角速度
			imu_data.angular_velocity.x = ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3])/65536.0*500.0/57.3;
			imu_data.angular_velocity.y = ((data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7])/65536.0*500.0/57.3;
			imu_data.angular_velocity.z = ((data[8] << 24) | (data[9] << 16) | (data[10] << 8) | data[11])/65536.0*500.0/57.3;
			//线加速度
			imu_data.linear_acceleration.x = ((data[12] << 24) | (data[13] << 16) | (data[14] << 8) | data[15])/65536.0*16.0*9.8;
			imu_data.linear_acceleration.y = ((data[16] << 24) | (data[17] << 16) | (data[18] << 8) | data[19])/65536.0*16.0*9.8;
			imu_data.linear_acceleration.z = ((data[20] << 24) | (data[21] << 16) | (data[22] << 8) | data[23])/65536.0*16.0*9.8;
			//四元数位姿
			imu_data.orientation.w = ((data[24] << 24) | (data[25] << 16) | (data[26] << 8) | data[27])/Q30;
			imu_data.orientation.x = ((data[28] << 24) | (data[29] << 16) | (data[30] << 8) | data[31])/Q30;
			imu_data.orientation.y = ((data[32] << 24) | (data[33] << 16) | (data[34] << 8) | data[35])/Q30;
			imu_data.orientation.z = ((data[36] << 24) | (data[37] << 16) | (data[38] << 8) | data[39])/Q30;
		    // Define imu orientation covariance
            imu_data.orientation_covariance = boost::assign::list_of(0.05) (0) (0) 
                                                                    (0) (0.05) (0) 
                                                                    (0) (0) (0.05);
            imu_data.angular_velocity_covariance = boost::assign::list_of(0.025) (0) (0) 
                                                                         (0) (0.025) (0) 
                                                                         (0) (0) (0.025);
            imu_data.linear_acceleration_covariance = boost::assign::list_of(0.1) (0) (0) 
                                                                         (0) (0.1) (0) 
                                                                         (0) (0) (0.1);
            // ROS_INFO("cmd: %x", cmd);
			return IMU_ID;
		}
		
		case ENCODER_ID:
		{
			if(len != ENCODER_DATA_LEN)
				return 0;
			//左右电机编码数
			motor_encoder.leftEncoder = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
			motor_encoder.rightEncoder = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
			//线速度与角速度
			motor_encoder.vx = (short)((data[10] << 8) | data[11]);
			motor_encoder.w = (short)((data[14] << 8) | data[15]);
		
            // ROS_INFO("cmd: %x", cmd);
			return ENCODER_ID;
		}
		
		case SONAR_ID:
		{	
			if(len != SONAR_DATA_LEN)
				return 0;
			//6个超声波数据
			for(int i = 0; i < 6; i++)
			{
				sonar_data.data[i] = data[4*i + 3] | (data[4*i + 2] << 8);
			}
			
            // ROS_INFO("cmd: %x", cmd);
			return SONAR_ID;
		}
	}
    return 0;
}

static void publish_stm_data(uchar msg_id)
{
	switch (msg_id)
	{
		case IMU_ID:
			//ROS_INFO("pub imu");
			imu_pub.publish(imu_data);
			break;

		case ENCODER_ID:
			//ROS_INFO("pub encoder");
            encoder_pub.publish(motor_encoder);
			break;

		case SONAR_ID:
			//ROS_INFO("pub sonar");
			sonar_pub.publish(sonar_data);
			break;
			
		default:
			break;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tcp_comm");
	ros::NodeHandle handle;

	/* advertise and  subscribe init */
	encoder_pub = handle.advertise<common::Encoder>("encoder", 10);
	sonar_pub = handle.advertise<std_msgs::UInt16MultiArray>("sonar", 10);
	imu_pub = handle.advertise<sensor_msgs::Imu>("imu", 10);

	smooth_sub = handle.subscribe("cmd_vel", 1, smoother_callback);
	enco_reset_sub = handle.subscribe("encoder_reset", 1, encoder_reset_callback);

	/* create thread for two subscribe */
	ros::AsyncSpinner spinner(2);
	spinner.start();

	init_send_buff();

    sonar_data.data.resize(6);

	/* tcp server set up*/
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if(sockfd < 0)
	{
		perror("socket");
		exit(1);
	}

	int optval = 1;
	setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

	struct sockaddr_in myaddr, peeraddr;
	myaddr.sin_family=AF_INET;
	myaddr.sin_port=htons(24000);
	myaddr.sin_addr.s_addr=inet_addr("0.0.0.0");

	if(bind(sockfd,(struct sockaddr*)&myaddr,sizeof(myaddr)) < 0)
	{
		perror("socket");
		exit(1);
	}

	listen(sockfd, 1);

	socklen_t len=sizeof(peeraddr);
	while(ros::ok())
	{
		memset(&peeraddr, 0, sizeof(peeraddr));

		confd = accept(sockfd, (struct sockaddr*)&peeraddr, &len);
		if(confd < 0)
		{
			perror("accept");
			exit(1);
		}

		tcp_state_ok = 1;
		ROS_INFO("peer ip: %s", inet_ntoa(peeraddr.sin_addr));

		int ret_recv;
		uchar recv_len = HEADSIZE, data_len = HEADSIZE;  //msg head size is 4 (FD FE FF + LEN) 
		uchar tmp_len = 0;
		uchar head_ok = 0, parse_data_ok = 0;
		uchar msg_id = 0;

		static uchar recvbuf[DATASIZE] = {0};
		static uchar buf_tmp[DATASIZE] = {0};
		while(ros::ok())
		{
			memset(&buf_tmp, 0, recv_len);

			ret_recv = recv(confd, &buf_tmp, recv_len, 0);
			if(ret_recv < 0)
			{
				tcp_state_ok = 0;
				perror("recv");
				close(confd);
				// close(sockfd);
				// exit(1);
				break;
			}
			else if(ret_recv == 0)
			{
				tcp_state_ok = 0;
				ROS_INFO("peer is shutdown");
				close(confd);
				break;
			}
			else
			{
				for(int i = 0; i < ret_recv; i++)
						recvbuf[tmp_len + i] = buf_tmp[i];
				
				tmp_len += ret_recv;
				if(tmp_len < data_len)
				{
					recv_len = data_len - tmp_len;	//剩余读取长度
					continue;
				}
				
				if((head_ok == 0) && (recvbuf[0] == 0xFD) && (recvbuf[1] == 0xFE) && (recvbuf[2] == 0xFF))
				{
					data_len = recv_len = recvbuf[HEADSIZE - 1];	//获得数据内容长度
					memset(recvbuf, 0, HEADSIZE);
					tmp_len = 0;
					head_ok = 1;
					continue;
				}
				
				if(head_ok)
				{
					msg_id = parse_stm_data(recvbuf[0], &recvbuf[1], data_len);	//解析数据内容

					data_len = recv_len = TAILSIZE;
					memset(recvbuf, 0, DATASIZE);
					tmp_len = 0;
					head_ok = 0;
					parse_data_ok = 1;
					continue;
				}

				if((parse_data_ok) && (recvbuf[0] == 0xFA) && (recvbuf[1] == 0xFB) && (recvbuf[2] == 0xFC))
				{
					publish_stm_data(msg_id);		//发布数据
					
					data_len = recv_len = HEADSIZE;
					memset(recvbuf, 0, TAILSIZE);
					tmp_len = 0;
					parse_data_ok = 0;
				}
			}
		}
	}
}

