
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <thread>
using namespace std;

#include "hitbot_interface.h"

#define PORT 9054
#define MAXDATASIZE 100
#define BUFFER_SIZE 128


#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

const int ROBOT_SUM=256;


//int robot_ids[robot_sum]{23,52,61,74};

ControlBeanEx * robots[ROBOT_SUM];

int movj(ControlBeanEx *bean, float* pos,float speed){
    int ret = bean->movej_angle(pos[0],pos[1],pos[2],pos[3],speed,0);
    return ret;
}

bool reset_angle(ControlBeanEx *bean, double init_z=-130.0, float angle1=-90, float angle2=45, float r=0, bool joint_home=false, float speed=100);
bool move_xyz(ControlBeanEx *bean, double x, double y, double init_z=-130.0, float r=0, bool joint_home=false, float speed=100);
bool move_xyz(ControlBeanEx *bean, double x, double y, double init_z, float r, bool joint_home, float speed){
    while(true){
        int sign = bean->movej_xyz(x,y,init_z,r,speed,0);
        if(sign == 1){
            bean->wait_stop();
            return true;
        }
        else if(sign == 102)
            if(joint_home){
                cout<<"joint_home init"<<endl;
                bean->joint_home(1);
                bean->joint_home(2);
                bean->wait_stop();
            }
            else{
                cout<<"robot return 102,Try again in 1 s"<<endl;
                sleep(1);
                continue;
            }
        else{
            cout<<"CannotReachError Number: "<< sign <<endl;
            bean->initial(1, 210);
            return false;
        }
    }
}
bool reset_angle(ControlBeanEx *bean, double init_z, float angle1, float angle2, float r, bool joint_home, float speed){
    while(true){
        int ans = bean->movej_angle(angle1, angle2, init_z, r, speed, 0);   //初始化采用角度设置，括号里为（大臂角度，小臂角度， 初始位置高度， 爪子角度，速度，运动方式）
        if(ans == 1){
            bean->wait_stop();
            cout<<"robot has reached the initial position"<<endl;
            return true;
        }
        else if(ans == 102){
            if(joint_home){
                cout<<"joint_home init"<<endl;
                bean->joint_home(1);
                bean->joint_home(2);
                bean->wait_stop();
            }
            else{
                cout<<"robot return 102,Try again in 1 s"<<endl;
                sleep(1);
                continue;
            }
        }
        else{
            cout<<"cannot reach the initial position :"<<ans<<endl;
            bean->initial(1, 210);
            return false;
        }
    }
}
void deal(char *buf, float &x, float &y, int &z, int &th, int begin)
{
	char * temp = new char[7];
	int length = begin;
	int i;
	string str;
	
	// find x
	for(i=length; buf[i] != ';'; i++) //字符串前7个 origin;所以i从7开始
	{
		temp[i-length] = buf[i];
	}
	temp[i-length] = '\0';
	length = i+1;
	str = temp;
	delete [] temp;
	
	x = atof(str.c_str());
	
	char * temp1 = new char [7];
	// find y
	for(i=length; buf[i] != ';'; i++)
	{
		temp1[i-length] = buf[i];
	}
	temp1[i-length] = '\0';
	length = i+1;
	str = temp1;
	delete [] temp1;
	y = atof(str.c_str());
	
	char * temp2 = new char [7];
	// find y
	for(i=length; buf[i] != ';'; i++)
	{
		temp2[i-length] = buf[i];
	}
	temp2[i-length] = '\0';
	length = i+1;
	str = temp2;
	delete [] temp2;
	z = atof(str.c_str());
	
	//find th
	char * temp3 = new char [5];
	for(i=length; buf[i]; i++)
	{
		temp3[i-length] = buf[i];
	}
	temp3[i-length] = '\0';
	str = temp3;
	delete [] temp3;
	th = atof(str.c_str());
	//cout<<x<<"  "<<y<<"  "<<z<<" "<<th<<endl;
}
void *mov_thread(void * attr){


    ControlBeanEx *bean=(ControlBeanEx *)attr;

    while(true){
        if(bean->is_connected()) break;
        sleep(1);
    }
    cout<<"robot "<<bean->get_robot_id()<<" connected,start move"<<endl;

    int ret = bean->initial(1, 210);
    if(ret == 1){
	    bean->get_scara_param();
	    cout<<"x: "<<bean->x<<"y: "<<bean->y<<"z: "<<bean->z<<endl;
	    bean->unlock_position();
	    reset_angle(bean);
	}
	
    //socket
	int numbytes; 
	char buf[MAXDATASIZE];

	struct hostent *he; 
	struct sockaddr_in their_addr;

	/* 取得主机信息 */ 
	if ((he=gethostbyname("192.168.0.100")) == NULL) 
	//if ((he=gethostbyname("192.168.0.100")) == NULL) 
	{
		/* 如果gethostbyname()发生错误，则显示错误信息并退出 */ 
		herror("gethostbyname" ); 
	    exit(1); 
	} 
	int sockfd = -1;
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) 
	{ 
		/* 如果socket()调用出现错误则显示错误信息并退出 */ 
		perror(" socket "); 
		exit(1); 
	}

	/* 主机字节顺序 */    
	their_addr.sin_family = AF_INET;

	/* 网络字节顺序，短整型 */ 
	their_addr.sin_port = htons(PORT);
	their_addr.sin_addr = *((struct in_addr *)he->h_addr); 
	/* 将结构剩下的部分清零*/ 
	memset(&(their_addr.sin_zero),0,8);
	
	while(true){
	    if(connect(sockfd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1)
	    { 
		    cout<<"Failed to connect to server, Will retry in 3 second"<<endl;
            sleep(3);
	    }
	    else{
	        cout<<"Connect to host"<<endl;
	        break;
	    }
	        
	}
	
	
	//初始化通讯
	sprintf(buf,"armThread\n");                
	if (send(sockfd, buf, strlen(buf), 0) == -1) 
	{ 
		perror("send err"); 
		close(sockfd); 
		exit(0); 
	} 
	char recvBuf[100];
	int recvLen;
	
	while(1){		
		memset(recvBuf, 0, MAXDATASIZE);

 		if((numbytes=recv(sockfd, recvBuf, MAXDATASIZE, 0)) >0){ 			
			if(recvBuf[numbytes-1]=='\n'){
				recvBuf[numbytes-1]='\0'; 
				numbytes--;
			} 
			cout<<"I receive: "<<recvBuf<<endl;
		}			  
        if(strncmp(recvBuf,"pose;", 5)==0){
            float posX(0.0), posY(0.0);
            int posZ(0), posTh(0);
            deal(recvBuf, posX, posY, posZ, posTh, 5);
            if(posX==-1 && posY==-1 && posZ==-1){
                cout<<"end"<<endl;
                sleep(3);
                break;
            }
            if(move_xyz(bean, posX, posY, posZ, posTh)){
                sprintf(buf,"poseSuccess\n");                
	            if (send(sockfd, buf, strlen(buf), 0) == -1){ 
		            perror("send err"); 
	            } 
	        }
            else{
                sprintf(buf,"poseError\n");                
	            if (send(sockfd, buf, strlen(buf), 0) == -1){ 
		            perror("send err"); 
	            } 
            }
		}
		else if(strncmp(recvBuf,"armReset", 8)==0){  
            if(reset_angle(bean)){
                sprintf(buf,"poseReset\n");                
	            if (send(sockfd, buf, strlen(buf), 0) == -1){ 
		            perror("send err"); 
	            } 
	        }
            else{
                sprintf(buf,"poseError\n");                
	            if (send(sockfd, buf, strlen(buf), 0) == -1){ 
		            perror("send err"); 
	            } 
            }
		}
		else if(strncmp(recvBuf,"setAngle;", 9)==0){
		    float thX(0.0), thY(0.0);
		    int posZ(0), posTh(0);
		    deal(recvBuf, thX, thY, posZ, posTh, 9);
            if(reset_angle(bean, posZ, thX, thY, posTh)){
                sprintf(buf,"poseSuccess\n");                
	            if (send(sockfd, buf, strlen(buf), 0) == -1){ 
		            perror("send err"); 
	            } 
	        }
            else{
                sprintf(buf,"poseError\n");                
	            if (send(sockfd, buf, strlen(buf), 0) == -1){ 
		            perror("send err"); 
	            } 
            }
		}
		else
		    ;
		sleep(1);	//100ms
	}
    close(sockfd); 
}


int main(int argc, char *argv[])
{

    cout<<"start network initial"<<endl;
    int ret= net_port_initial();

    if(ret != 1){
        cout<<"initial failed"<<endl;


    }
    else{
        cout<<"initial successed"<<endl;


        pthread_t p_ids[ROBOT_SUM];

        for(int i=0;i<ROBOT_SUM;i++){

            robots[i]=get_robot(i);
            if(robots[i]->is_connected()){

                pthread_create(&p_ids[i],NULL,mov_thread,(void*)(robots[i]));
                pthread_detach(p_ids[i]);
            }
        }

    }
	
    while(true){
        sleep(1);
    }
    return 0;
}

