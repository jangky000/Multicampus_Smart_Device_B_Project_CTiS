#include "stm32f10x.h"
#include "udf.h"
#include <stdio.h>
#include <string.h>
#include "libemqtt.h"

//LCD, IC2,
#include "device_driver.h"
#include "st7735.h"

#define printf		Uart1_Printf
#define BUF_SIZE	512
#define	SEVER_IP	"192.168.0.200"
#define PORT		1883

#define RCVBUFSIZE 1024
uint8_t packet_buffer[RCVBUFSIZE];

#define CAR_NUMBER 1

int socket_id;
mqtt_broker_handle_t broker;
int keepalive = 0;

typedef struct _car
{
  int y;
  int x;
}Car;

//Car car[5];
int gyro_accident_flag;
int honk;

int car[2][9];
//char color[2][5]={{"RED"}, {"BLUE"}};

//int xy_car[10];
//int prev_xy_car[10];

int flag[2];

//lcd 크기는 가로 159 세로 127

extern mqtt_broker_handle_t broker;
extern CHCONFIG_TYPE_DEF Chconfig_Type_Def;
//
extern volatile int TIM4_Expired;
extern volatile int ADC1_value;
extern volatile int ADC1_flag;
extern volatile int Key_Value;

RCC_ClocksTypeDef RCC_ClockFreq;

/* Private function prototypes -----------------------------------------------*/
extern void Ethernet_Test(void);
extern void Ethernet_Init(void);

extern void loopback_tcps(char s, unsigned short port);
extern void loopback_tcpc(char s, char *str_ip,unsigned short port);
extern void loopback_udp(char s, unsigned short port);

/* Private functions ---------------------------------------------------------*/
void Board_Init(void);

int send_packet(void* socket_info, const void* buf, unsigned int count)
{
	int8_t fd = *((int8_t*)socket_info);
	return send(fd, (unsigned char*)buf, count);
}

int init_socket(mqtt_broker_handle_t* broker, const char* hostname, short port)
{
	socket_id = 2;
	socket_id = socket(socket_id, SOCK_STREAM, 8522, 0x00 );
	if( socket_id  < 0)
	{
		printf("socket create error\n");
		return -1;
	}

	if( ( connect(socket_id,Chconfig_Type_Def.destip, port ) ) < 0)
		return -1;

	// MQTT stuffs
	mqtt_set_alive(broker, keepalive);
	broker->socket_info = (void*)&socket_id;
	broker->send = send_packet;

	return 0;
}

int close_socket(mqtt_broker_handle_t* broker)
{
	int8_t fd = *((int8_t*)broker->socket_info);
	return close(fd);
}


int read_packet(int timeout)
{
	int total_bytes = 0, bytes_rcvd, packet_length;

	memset(packet_buffer, 0, sizeof(packet_buffer));

	while(total_bytes < 2) // Reading fixed header
	{
    if((bytes_rcvd = recv(socket_id, (packet_buffer+total_bytes), RCVBUFSIZE)) <= 0){
    printf("check %d\n", bytes_rcvd);        
		return -1;
  }
		total_bytes += bytes_rcvd; // Keep tally of total bytes

	}

	packet_length = packet_buffer[1] + 2; // Remaining length + fixed header length
	while(total_bytes < packet_length) // Reading the packet
	{
		if((bytes_rcvd = recv(socket_id, (packet_buffer+total_bytes), RCVBUFSIZE)) <= 0)
			return -1;
		total_bytes += bytes_rcvd; // Keep tally of total bytes
	}
	return packet_length;

}

//논블로킹 방식의 recv 함수를 사용
int read_packet_v2(int timeout)
{
	int total_bytes = 0, bytes_rcvd, packet_length;

	memset(packet_buffer, 0, sizeof(packet_buffer));

	while(total_bytes < 2) // Reading fixed header
	{
    if((bytes_rcvd = recv_v2(socket_id, (packet_buffer+total_bytes), RCVBUFSIZE)) <= 0){
		return -1;
  }
		total_bytes += bytes_rcvd; // Keep tally of total bytes

	}

	packet_length = packet_buffer[1] + 2; // Remaining length + fixed header length
	while(total_bytes < packet_length) // Reading the packet
	{
		if((bytes_rcvd = recv(socket_id, (packet_buffer+total_bytes), RCVBUFSIZE)) <= 0)
			return -1;
		total_bytes += bytes_rcvd; // Keep tally of total bytes
	}
	return packet_length;

}

void main(void)
{
  //int led_stat = 0;
	int packet_length;
	uint16_t msg_id, msg_id_rcv;
  signed char x, y, z; // 가속도 센서 값
  float AD_value;//조도 센서 값
  int i, j;
  int battery;
  char battery_str[10];
  char car_str[20];
  int tt = 0;
  
  //파싱을 위해.
  char *result;
  
	RCC_GetClocksFreq(&RCC_ClockFreq);
	Uart1_Init(115200);
	
  //LED
  LED_Init();
  
  //Key
  Key_ISR_Enable(1);
	
  //가속도 센서
  I2C_Setup();
  //조도 센서
  ADC1_Init();
	ADC1_ISR_Enable(1,9);
  //Uart1_RX_Interrupt_Enable(1); // 필요한가?
  
  TIM4_Repeat_Interrupt_Enable(1, 500); // 타이머4
  
  //타이머 다음에 사용해야 함
  // LCD
  Lcd_Init(); // 타이머2 사용
  Uart1_Printf("Clear Screen\n");
  //Lcd_Clr_Screen(BLUE);
  
  
  //BUZZER
  TIM3_Buzzer_Init();
  TIM3_Buzzer_Beep(0, 1000);

  ////////////////////////////////////////////////////////////////
  Uart1_Printf("Draw Rectangular\n");
	Lcd_Clr_Screen(BLACK);
	/*
  //도착 예정
  Lcd_Printf(0,0,WHITE,BLACK, 1,1,"MY CAR1");
  Lcd_Printf(0,20,WHITE,BLACK,1,1,"ARR TIME"); //
  Lcd_Printf(0,40,WHITE,BLACK,1,1,"T-20");
  //차가 움직이는 그림(필수)
  */
  
  /*
  //차가 들어왔을때
  Lcd_Printf(0,0,WHITE,BLACK,1,1,"MY CAR1");
  Lcd_Printf(0,20,WHITE,BLACK,1,1,"Cst: $2"); //요금
  Lcd_Printf(0,40,WHITE,BLACK,1,1,"Bat: 100%"); // 배터리
  Lcd_Printf(0,60,WHITE,BLACK,1,1,"Acc: 5"); // 사고 횟수
  Lcd_Printf(0,80,WHITE,BLACK,1,1,"Drv: 2km"); // 주행거리
  */
  
  /*
  // 사고가 났을 때
  Lcd_Printf(0,40,WHITE,BLACK,1,1,"MY_CAR1");
  //지도(1/2 크기로)와 위치는 좌표 표시
  Lcd_Printf(0,40,WHITE,BLACK,1,1,"");//좌표 표시
  */
  /*
  Lcd_Draw_Bar(0,0,79,127,WHITE)
  Lcd_Draw_Rect(0+1,0+1,79-1,127-1,BLUE);
  Lcd_Draw_Rect(0+5,0+5,79-5,127-5,BLUE);
  
  Lcd_Draw_Bar(80,0,159,127,WHITE);
  Lcd_Draw_Rect(80+1,0+1,159-1,127-1,BLUE);
  Lcd_Draw_Rect(80+5,0+5,159-5,127-5,BLUE);
  */
  
  /*
  //차선
  Lcd_Draw_Rect(28,28,108,108,WHITE);             // 여기부터 두줄 : 차선
	Lcd_Draw_Rect(12,12,124,124,WHITE);
	
	//차선에서 집으로 이어지는 도로 표시 -> 08/14 수정완료
	Lcd_Draw_Hline(44,108,144,WHITE);
	
	// 집 표시 -> 08/14수정완료
	Lcd_Draw_Rect(137-1,42-5-1,153-1,58-5-1,YELLOW);

	// 집 지붕 그리기 (대각선 포함)	
	for( i=0; i< 9 ; i++)
	{
		Lcd_Draw_Vline(137-1+2*i,37-5-1,41-5-1,YELLOW);
	}
	Lcd_Draw_Hline(37-5-1,137-1,153-1,YELLOW);	
	Lcd_Draw_Hline(42-5-1,137-1-4,153-1+4,YELLOW);
	Lcd_Draw_Rect(137-1+10,37-5-1-5,137-1+10+3,37-5-1,YELLOW);
	Lcd_Draw_Line(137-1+1,37-5-1-1,137-1-4,42-5-1,YELLOW);
	Lcd_Draw_Line((137-1+4)+8+4-1,37-5-1-1,(137-1+4)+8+4+4,42-5-1,YELLOW);
	
	for (i=0; i<3 ; i++)   // 왼쪽 대각선 채우기
	{
		Lcd_Draw_Line(137-1,37-5-1+i,137-1-4+i,42-5-1,YELLOW);
	}
	
	for (i=0; i<3 ; i++)  // 오른쪽 대각선 채우기
	{
		Lcd_Draw_Line((137-1+4+8+4),37-5-1+i,(137-1+4)+8+4+4-i,42-5-1,YELLOW);
	}
	
	for (i=0; i<17 ; i++)
		Lcd_Draw_Vline(137-1-1+i+1,37-5-1,41-5-1-2-2,BLACK);
	for (i=0; i<19 ; i++)
		Lcd_Draw_Vline(137-1-1+i,37-5-1+2,41-5-1-2,BLACK);
	for (i=0; i<21 ; i++)
		Lcd_Draw_Vline(137-1-1+i-1,37-5-1+2+2,41-5-1-2+2,BLACK);
	
	Lcd_Draw_Hline(37-5-1-1,137-1,153-1,YELLOW);
	
	// 집 굴뚝에서 연기나는거 표현하기 -> 연기 움직이는거 추가하기
	//for (i=0; i<10 ; i++)
	//Lcd_Draw_Bar(137-1+10+1,37-5-1-5-6+1-5*i,137-1+10+3-1,37-5-1-5-6+3-1-5*i,WHITE);
	
	// 예시로 집 안에서 차가 충전 되는 모습 구현 -> 나중에 지우기.
	//Lcd_Draw_Bar(140,40,148,48,BLUE);
	
	// 배터리 표시 -> 08/14수정완료
	Lcd_Draw_Rect(134,78,156,97,WHITE);
	Lcd_Draw_Bar(136,80,139,95,GREEN);	
	Lcd_Draw_Bar(141,80,144,95,GREEN);
	Lcd_Draw_Bar(146,80,149,95,GREEN);	
	Lcd_Draw_Bar(151,80,154,95,GREEN);
	Lcd_Draw_Bar(156,85,158,89,WHITE);
	
  Lcd_Printf(127,103,WHITE,BLACK,1,1,"100%");
  
	// 배터리쪽 지나갈 때 닿는지 안닿는지 체크 -> 나중에 지우기.
	//Lcd_Draw_Bar(120,84,128,92,WHITE);
	
	// 톨게이트 구현
	Lcd_Draw_Rect(28-8,42-5-1+24,28-8+16,58-5-1+24,WHITE);
	
	// 톨게이트 모습 구현을 위해 X자 표시하기 
	
	Lcd_Draw_Line(28-8,42-5-1+24,(28-8)+16,(42-5-1+24)+16,PINK);
	Lcd_Draw_Line((28-8)+16,42-5-1+24,(28-8),(42-5-1+24)+16,PINK);
	
	// 톨게이트 안에 차선 지우기
	Lcd_Draw_Vline(28,61,59+16,BLACK);
	
	// 톨게이트 안에 있는 차량 모습 예시 -> 나중에 지우기.
	//Lcd_Draw_Bar(28-8+4,42-5-1+24+4,28-8+4+8,42-5-1+24+4+8,RED);
	
	// 큰 차선 바깥으로 벗어나는지 테스트 -> 나중에 지우기.
	//Lcd_Draw_Bar(56,120,64,128,WHITE);
  
  */
  ////////////////////////////////////////////////////////////////
  
	Ethernet_Init();
	
	mqtt_init(&broker, "my_home");
	mqtt_init_auth(&broker, NULL, NULL);
	init_socket(&broker, SEVER_IP, PORT);

	// >>>>> CONNECT
	mqtt_connect(&broker);

	// <<<<< CONNACK
	packet_length = read_packet(0);

	printf("receive packet length = %d\n",packet_length);

	if(packet_length < 0)
	{
		printf("Error(%d) on read packet!\n", packet_length);
		return;
	}

	if(MQTTParseMessageType(packet_buffer) != MQTT_MSG_CONNACK)
	{
		printf("CONNACK expected!\n");
		return;
	}

	if(packet_buffer[3] != 0x00)
	{
		printf("CONNACK failed!\n");
		return;
	}

	printf("CONNACK received..\n");

	// >>>>>> subscribe
	printf("subscribe\n");
	mqtt_subscribe(&broker, "home", &msg_id);
 	TIM2_Delay(100);

	// <<<<< SUBACK
	packet_length = read_packet(1);
	if(packet_length < 0)
	{
		printf("Error(%d) on read packet!\n", packet_length);
		return ;
	}

	if(MQTTParseMessageType(packet_buffer) != MQTT_MSG_SUBACK)
	{
		printf("SUBACK expected!\n");
		return ;
	}

	msg_id_rcv = mqtt_parse_msg_id(packet_buffer);
	if(msg_id != msg_id_rcv)
	{
		printf("%d message id was expected, but %d message id was found!\n", msg_id, msg_id_rcv);
		return ;
	}

	printf("SUBACK received..\n");
/////////////////////////////////////////////////////
	for(;;)
	{
		// <<<<<
		packet_length = read_packet_v2(0);
		if(packet_length == -1)
		{
			//printf("Error(%d) on read packet!\n", packet_length);
			//return ;
		}
		else if(packet_length > 0)
		{
			printf("Packet Header: 0x%x...\n", packet_buffer[0]);
			if(MQTTParseMessageType(packet_buffer) == MQTT_MSG_PUBLISH)
			{
				uint8_t topic[255], msg[1000];
				uint16_t len;
				len = mqtt_parse_pub_topic(packet_buffer, topic);
				topic[len] = '\0'; // for printf
				len = mqtt_parse_publish_msg(packet_buffer, msg);
				msg[len] = '\0'; // for printf
				printf("%s %s\n", topic, msg);
        //msg -> 파싱 y, x 정보를 추출하고
        
        //i=0;
        result = strtok(msg, " ");
        //차1
        for(i=0; i<9; i++)
        {
          car[0][i] = atoi(result);
          result=strtok(NULL, " ");
        }
        //차2
        for(i=0; i<9; i++)
        {
          car[1][i] = atoi(result);
          result=strtok(NULL, " ");
        }
        
        for(i = 0; i < 2; i++)
        {
          //화면 처리 3가지
          if(car[i][2] == 0) // vel
          {
            if(car[i][0] == 1 ) // accident flag
            {
							if (flag[i] != 1){
								//1.사고 발생
								Lcd_Draw_Bar(0+80*i,0,79+80*i,127,WHITE);
								Lcd_Draw_Rect(0+1+80*i,0+1,79-1+80*i,127-1,BLUE);
								Lcd_Draw_Rect(0+5+80*i,0+5,79-5+80*i,127-5,BLUE);
								
								sprintf(car_str, "MY_CAR%d", i+1);
								Lcd_Printf(10+80*i,10,WHITE,RED,1,1,car_str);
								flag[i] = 1;
							}
  
              //지도(1/2 크기로)와 위치는 좌표 표시
              Lcd_Printf(10+80*i,30,RED,WHITE,1,1,"Accident");//좌표 표시
              sprintf(car_str, "posX:%d", car[i][8]);
              Lcd_Printf(10+80*i,50,BLACK,WHITE,1,1,car_str);//좌표 표시
              sprintf(car_str, "posY:%d", car[i][7]);
              Lcd_Printf(10+80*i,70,BLACK,WHITE,1,1, car_str);//좌표 표시
            }
            else
            {
              //집으로 들어왔거나 정차함
							printf("home info\n");
              if(car[i][7] == 5 && car[i][8] == 17) 
              {
								//2.
								
								 
			//					printf("home info\n");
								//=======다끝나면 이 부분을 지우면 된다.
								
                //집 도착
                /*Lcd_Draw_Bar(0+80*i,0,79+80*i,127,WHITE);
                Lcd_Draw_Rect(0+1+80*i,0+1,79-1+80*i,127-1,BLUE);
                Lcd_Draw_Rect(0+5+80*i,0+5,79-5+80*i,127-5,BLUE); 
              
                Lcd_Printf(10+80*i,10,WHITE,BLUE,1,1,"MY CAR1"); 
                sprintf(car_str, "C:$%d", car[i][3]/1000);
                Lcd_Printf(10+80*i,30,BLACK,WHITE,1,1,car_str); //요금
                sprintf(car_str, "Bat:%d%%", car[i][4]);
                Lcd_Printf(10+80*i,50,BLACK,WHITE,1,1,car_str); // 배터리
                sprintf(car_str, "Acc:%d", car[i][5]);
                Lcd_Printf(10+80*i,70,BLACK,WHITE,1,1,car_str); // 사고 횟수
                sprintf(car_str, "Dv:%3dm", car[i][6]);
                Lcd_Printf(10+80*i,90,BLACK,WHITE,1,1,car_str); // 주행거리
								*/
								//========여기까지.
								
								//======================================//
								
								///
								if(flag[i] != 2)
								{
									//고정 그래픽들
									Lcd_Draw_Bar(0+80*i,0,79+80*i,127,BLACK);
									Lcd_Draw_Vline(80,0,128,WHITE);
									if(i==0)
										Lcd_Printf(45+80*i,10,WHITE,RED,1,1,"%s", "Car1");
									else 		
										Lcd_Printf(45+80*i,10,WHITE,BLUE,1,1,"%s", "Car2");
										
									//주차장 라인
									Lcd_Draw_Rect(5+80*i,10,40+80*i,67,WHITE);
									
									// 차량 몸체
									if(i==0)
										Lcd_Draw_Bar(12+80*i,17,33+80*i,60,RED);
									else
										Lcd_Draw_Bar(12+80*i,17,33+80*i,60,BLUE);
									Lcd_Draw_Bar(15+80*i,17,18+80*i,19,YELLOW);
									Lcd_Draw_Bar(27+80*i,17,30+80*i,19,YELLOW);
									Lcd_Draw_Bar(15+80*i,58,18+80*i,60,YELLOW);	
									Lcd_Draw_Bar(27+80*i,58,30+80*i,60,YELLOW);
									Lcd_Draw_Rect(26+80*i,60,28+80*i,64,WHITE);
									
									// 차량 모서리 지우기
									Lcd_Draw_Bar(12+80*i,17,14+80*i,19,BLACK);
									Lcd_Draw_Bar(31+80*i,17,33+80*i,19,BLACK);
									Lcd_Draw_Bar(12+80*i,58,14+80*i,60,BLACK);	
									Lcd_Draw_Bar(31+80*i,58,33+80*i,60,BLACK);
									
									// 차량 창문 전면유리
									for(j=0;j<5;j++)
									{	
										Lcd_Draw_Vline(14+j+80*i,28,28+j,WHITE);
									}	
									Lcd_Draw_Bar(18+80*i,28,27+80*i,32,WHITE);
									for(j=0;j<5;j++)
									{
										Lcd_Draw_Vline(31-j+80*i,28,28+j,WHITE);
									}	
									// 차량 창문 측면유리
									for(j=0;j<4;j++)
									{
										Lcd_Draw_Vline(15+j+80*i,33+j,52-j,WHITE);
									}
									Lcd_Draw_Hline(43,15+80*i,18+80*i,BLACK);
									for(j=0;j<4;j++)
									{
										Lcd_Draw_Vline(31-j+80*i,33+j,52-j,WHITE);
									}
									Lcd_Draw_Hline(43,28+80*i,31,BLACK);	
								
									// 차량 바퀴
									Lcd_Draw_Rect(9+80*i,28,11+80*i,32,WHITE);
									Lcd_Draw_Rect(9+80*i,48,11+80*i,52,WHITE);	
									Lcd_Draw_Rect(34+80*i,28,36+80*i,32,WHITE);
									Lcd_Draw_Rect(34+80*i,48,36+80*i,52,WHITE);
									
									//베터리 
									Lcd_Draw_Rect(49+81*i,30,71+81*i,49,WHITE); //몸체
									Lcd_Draw_Bar(71+81*i,37,73+81*i,41,WHITE);  //튀어나온 부분
									// 톨게이트 구현
									Lcd_Draw_Rect(7+80*i,73,23+80*i,89,WHITE);
								
									// 톨게이트 모습 구현을 위해 X자 표시하기 
									Lcd_Draw_Line(7+80*i,73,23+80*i,89,PINK);
									Lcd_Draw_Line(23+80*i,73,7+80*i,89,PINK);
										
									// 주행거리
									Lcd_Draw_Rect(7+80*i,91,23+80*i,107,WHITE);
									Lcd_Draw_Bar(10+80*i,97,16+80*i,101,YELLOW);
									for(j=0; j<6; j++)
									{
										Lcd_Draw_Vline(16+j+80*i,94+j,104-j,YELLOW);
									}
									
									// 사고횟수	
									Lcd_Draw_Rect(7+80*i,109,23+80*i,125,WHITE);
									Lcd_Draw_Bar(13+80*i,111,17+80*i,119,WHITE);
									Lcd_Draw_Bar(13+80*i,121,17+80*i,123,WHITE);
									
									flag[i] = 2;
								}
								//Lcd_Draw_Bar(0,0,159,127,BLACK);
								
								///
								
								
								//Lcd_Draw_Vline(80,0,128,WHITE);
								
								//////////// CAR 1/2 그래픽 /////////////
								/*if(i==0)
									Lcd_Printf(45+80*i,10,WHITE,RED,1,1,"%s", "Car1");
								else 		
									Lcd_Printf(45+80*i,10,WHITE,BLUE,1,1,"%s", "Car2");
								*/
								// 주차장 라인
								//Lcd_Draw_Rect(5+80*i,10,40+80*i,67,WHITE);
								
								// 차량 몸체
								/*if(i==0)
									Lcd_Draw_Bar(12+80*i,17,33+80*i,60,RED);
								else
									Lcd_Draw_Bar(12+80*i,17,33+80*i,60,BLUE);
								Lcd_Draw_Bar(15+80*i,17,18+80*i,19,YELLOW);
								Lcd_Draw_Bar(27+80*i,17,30+80*i,19,YELLOW);
								Lcd_Draw_Bar(15+80*i,58,18+80*i,60,YELLOW);	
								Lcd_Draw_Bar(27+80*i,58,30+80*i,60,YELLOW);
								Lcd_Draw_Rect(26+80*i,60,28+80*i,64,WHITE);
								
								// 차량 모서리 지우기
								Lcd_Draw_Bar(12+80*i,17,14+80*i,19,BLACK);
								Lcd_Draw_Bar(31+80*i,17,33+80*i,19,BLACK);
								Lcd_Draw_Bar(12+80*i,58,14+80*i,60,BLACK);	
								Lcd_Draw_Bar(31+80*i,58,33+80*i,60,BLACK);
								
								// 차량 창문 전면유리
								for(j=0;j<5;j++)
								{	
									Lcd_Draw_Vline(14+j+80*i,28,28+j,WHITE);
								}	
								Lcd_Draw_Bar(18+80*i,28,27+80*i,32,WHITE);
								for(j=0;j<5;j++)
								{
									Lcd_Draw_Vline(31-j+80*i,28,28+j,WHITE);
								}	
								// 차량 창문 측면유리
								for(j=0;j<4;j++)
								{
									Lcd_Draw_Vline(15+j+80*i,33+j,52-j,WHITE);
								}
								Lcd_Draw_Hline(43,15+80*i,18+80*i,BLACK);
								for(j=0;j<4;j++)
								{
									Lcd_Draw_Vline(31-j+80*i,33+j,52-j,WHITE);
								}
								Lcd_Draw_Hline(43,28+80*i,31,BLACK);	
							
								// 차량 바퀴
								Lcd_Draw_Rect(9+80*i,28,11+80*i,32,WHITE);
								Lcd_Draw_Rect(9+80*i,48,11+80*i,52,WHITE);	
								Lcd_Draw_Rect(34+80*i,28,36+80*i,32,WHITE);
								Lcd_Draw_Rect(34+80*i,48,36+80*i,52,WHITE);*/
								
								// 배터리 잔량 표시
								memset(car_str, 0, sizeof(car_str));
								sprintf(car_str, "%3d%%", car[i][4]); ////////////////////////
								if (i==0)
									Lcd_Printf(45+80*i,53,WHITE,RED,1,1,car_str); 
								else
									Lcd_Printf(45+80*i,53,WHITE,BLUE,1,1,car_str); 							
								// 배터리 표시 

								//클리어가 되는지 모르겠어서 일단 지움								
								//Lcd_Draw_Bar(51+81*i,32,69+81*i,47,BLACK);

								//Lcd_Draw_Rect(49+81*i,30,71+81*i,49,WHITE); //몸체
								
								switch((car[i][4]+24)/25)
								{
								case 4:
									Lcd_Draw_Bar(51+81*i,32,54+81*i,47,GREEN);	//1번
									Lcd_Draw_Bar(56+81*i,32,59+81*i,47,GREEN);  //2번
									Lcd_Draw_Bar(61+81*i,32,64+81*i,47,GREEN);	//3번
									Lcd_Draw_Bar(66+81*i,32,69+81*i,47,GREEN);  //4번
									break;
								case 3:
									Lcd_Draw_Bar(66+81*i,32,69+81*i,47,BLACK); //4번
									Lcd_Draw_Bar(61+81*i,32,64+81*i,47,GREEN);	//3번
									Lcd_Draw_Bar(51+81*i,32,54+81*i,47,GREEN);	//1번
									Lcd_Draw_Bar(56+81*i,32,59+81*i,47,GREEN);  //2번
									break;
								case 2:
									Lcd_Draw_Bar(61+81*i,32,64+81*i,47,BLACK);	//3번
									Lcd_Draw_Bar(56+81*i,32,59+81*i,47,GREEN);  //2번
									Lcd_Draw_Bar(51+81*i,32,54+81*i,47,GREEN);	//1번
									break;
								case 1:
									Lcd_Draw_Bar(56+81*i,32,59+81*i,47,BLACK);  //2번
									Lcd_Draw_Bar(51+81*i,32,54+81*i,47,GREEN);	//1번
									break;
								case 0:
									Lcd_Draw_Bar(51+81*i,32,54+81*i,47,BLACK);	//1번
									Lcd_Draw_Bar(56+81*i,32,59+81*i,47,BLACK);  //2번
									Lcd_Draw_Bar(61+81*i,32,64+81*i,47,BLACK);	//3번
									Lcd_Draw_Bar(66+81*i,32,69+81*i,47,BLACK);  //4번
									break;
								}
								
								/*if(1<=car[i][4]&&car[i][4]<=25)
									Lcd_Draw_Bar(51+81*i,32,54+81*i,47,GREEN);	//1번
								else if(26<=car[i][4] && car[i][4]<=50)
								{
									Lcd_Draw_Bar(51+81*i,32,54+81*i,47,GREEN);	//1번
									Lcd_Draw_Bar(56+81*i,32,59+81*i,47,GREEN);  //2번
								}
								else if(51<=car[i][4] && car[i][4]<=75)
								{
									Lcd_Draw_Bar(51+81*i,32,54+81*i,47,GREEN);	//1번
									Lcd_Draw_Bar(56+81*i,32,59+81*i,47,GREEN);  //2번
									Lcd_Draw_Bar(61+81*i,32,64+81*i,47,GREEN);	//3번
								}
								else if(76<=car[i][4]&&car[i][4]<=100){
									Lcd_Draw_Bar(51+81*i,32,54+81*i,47,GREEN);	//1번
									Lcd_Draw_Bar(56+81*i,32,59+81*i,47,GREEN);  //2번
									Lcd_Draw_Bar(61+81*i,32,64+81*i,47,GREEN);	//3번
									Lcd_Draw_Bar(66+81*i,32,69+81*i,47,GREEN);  //4번
								}*/
								
								
								
								//Lcd_Draw_Bar(71+81*i,37,73+81*i,41,WHITE);  //튀어나온 부분
								// 톨게이트 구현
								//Lcd_Draw_Rect(7+80*i,73,23,89,WHITE);
								
								// 톨게이트 모습 구현을 위해 X자 표시하기 
								//Lcd_Draw_Line(7+80*i,73,23+80*i,89,PINK);
								//Lcd_Draw_Line(23+80*i,73,7+80*i,89,PINK);
									
								// 톨게이트 요금 표시
								sprintf(car_str, "$%d", car[i][3]/1000);
								Lcd_Printf(29+80*i,73,WHITE,BLACK,1,1, car_str);
								
								// 주행거리
								//Lcd_Draw_Rect(7+80*i,91,23+80*i,107,WHITE);
								//Lcd_Draw_Bar(10+80*i,97,16+80*i,101,YELLOW);
								//for(j=0; j<6; j++)
								//{
								//	Lcd_Draw_Vline(16+j+80*i,94+j,104-j,YELLOW);
								//}
								sprintf(car_str, "%3dm", car[i][6]);
								Lcd_Printf(29+80*i,91,WHITE,BLACK,1,1,car_str);
								
								// 사고횟수	
								//Lcd_Draw_Rect(7+80*i,109,23+80*i,125,WHITE);
								//Lcd_Draw_Bar(13+80*i,111,17+80*i,119,WHITE);
								//Lcd_Draw_Bar(13+80*i,121,17+80*i,123,WHITE);
								sprintf(car_str, "%dtimes", car[i][5]);
								Lcd_Printf(29+80*i,109,WHITE,BLACK,1,1, car_str);		
              }
              else
              {
                //집이 아니고 길가에 정차함. 지운다. 그러고보니 얘는 왜지..??
                //Lcd_Draw_Bar(0,0,159,127,BLACK);
              }
            }
          }
          else //이 그래픽은 안한 거 맞겠지.
          {
            if(car[i][1] != 0) // hdis
            {
							if(flag[i] != 3)
							{
								Lcd_Draw_Bar(0+80*i,0,79+80*i,127,WHITE);
								Lcd_Draw_Rect(0+1+80*i,0+1,79-1+80*i,127-1,BLUE);
								Lcd_Draw_Rect(0+5+80*i,0+5,79-5+80*i,127-5,BLUE); 
								
								sprintf(car_str, "MY_CAR%d", i+1);
             	 	Lcd_Printf(10+80*i,10,WHITE,RED,1,1,car_str);
              	Lcd_Printf(10+80*i,30,BLACK,WHITE,1,1,"ARR TIME");
								
								flag[i] = 3;
							}
							
              //3.집으로 들어갈 예정
              /*Lcd_Draw_Bar(0+80*i,0,79,127,WHITE);
              Lcd_Draw_Rect(0+1+80*i,0+1,79-1,127-1,BLUE);
              Lcd_Draw_Rect(0+5+80*i,0+5,79-5,127-5,BLUE); 
              
              sprintf(car_str, "MY_CAR%d", i+1);
              Lcd_Printf(10+80*i,10,WHITE,RED,1,1,car_str);
              Lcd_Printf(10+80*i,30,BLACK,WHITE,1,1,"ARR TIME");*/
              sprintf(car_str, "- %3dm", car[i][1]);
              Lcd_Printf(10+80*i,50,BLACK,WHITE,1,1,car_str);
              //차가 움직이는 그림(필수)
            }
            else
            {
							//한번만 해주면 되지 않을까..?? 근데 경우의 수가 좀 헷갈린다.
							//이렇게 되면 집에서 나가면 나간 거에 대해서는 지워질 것 같기는 한데.
							if(flag[i]!=4){
								Lcd_Draw_Bar(0+80*i,0,79+80*i,127,BLACK);
								flag[i]=4;
							}
              //도로를 달리는 중
            }
          }
        }
        
        /*
        //차가 들어왔을때
        Lcd_Printf(0,0,WHITE,BLACK,1,1,"MY CAR1");
        Lcd_Printf(0,20,WHITE,BLACK,1,1,"Cst: $2"); //요금
        Lcd_Printf(0,40,WHITE,BLACK,1,1,"Bat: 100%"); // 배터리
        Lcd_Printf(0,60,WHITE,BLACK,1,1,"Acc: 5"); // 사고 횟수
        Lcd_Printf(0,80,WHITE,BLACK,1,1,"Drv: 2km"); // 주행거리
        */
        
        /*
        // 사고가 났을 때
        Lcd_Printf(0,40,WHITE,BLACK,1,1,"MY_CAR1");
        //지도(1/2 크기로)와 위치는 좌표 표시
        Lcd_Printf(0,40,WHITE,BLACK,1,1,"");//좌표 표시
        
        
        
        
        /*
        //배터리 정보
        for(int i=0; i<2; i++)
        {
          if(i == CAR_NUMBER-1 )
          {
            //1번 차량 
            battery = atoi(result);
            sprintf(battery_str, "%3d%%", battery);
          }
          result=strtok(NULL, " ");
        }
        */
       
        /*
        //자동차 표시
        for(i=0;i<5;i++)
        {
          //차 지우기 
          Lcd_Draw_Bar(prev_xy_car[2*i+1]*8, prev_xy_car[2*i]*8, prev_xy_car[2*i+1]*8+8, prev_xy_car[2*i]*8+8, BLACK); 
        }
        
        //차선 다시 그리기 
        Lcd_Draw_Rect(28,28,108,108,WHITE);             // 여기부터 두줄 : 차선
	      Lcd_Draw_Rect(12,12,124,124,WHITE);
          
        //차선에서 집으로 이어지는 도로 표시 -> 08/14 수정완료
	      Lcd_Draw_Hline(44,108,144,WHITE);
        // 집 표시 -> 08/14수정완료
	      Lcd_Draw_Rect(137-1,42-5-1,153-1,58-5-1,YELLOW);
          
        // 톨게이트 구현
        Lcd_Draw_Rect(28-8,42-5-1+24,28-8+16,58-5-1+24,WHITE);
          
        // 톨게이트 모습 구현을 위해 X자 표시하기 
	      Lcd_Draw_Line(28-8,42-5-1+24,(28-8)+16,(42-5-1+24)+16,PINK);
	      Lcd_Draw_Line((28-8)+16,42-5-1+24,(28-8),(42-5-1+24)+16,PINK);
        
        // 톨게이트 안에 차선 지우기
	      Lcd_Draw_Vline(28,61,59+16,BLACK);
        
        
        //자동차 표시
        //차 그리기
        for(i=0;i<5;i++)
        {
          if(i == 0)
            Lcd_Draw_Bar(xy_car[2*i+1]*8, xy_car[2*i]*8, xy_car[2*i+1]*8+8, xy_car[2*i]*8+8, YELLOW);
          else
            Lcd_Draw_Bar(xy_car[2*i+1]*8, xy_car[2*i]*8, xy_car[2*i+1]*8+8, xy_car[2*i]*8+8, RED);
          
          prev_xy_car[2*i+1]=xy_car[2*i+1];
          prev_xy_car[2*i]=xy_car[2*i];
          
        }
            
        Lcd_Printf(127,103,WHITE,BLACK,1,1, battery_str);
        
        switch( (battery+12)/25 )
        {
        case 4:
          Lcd_Draw_Bar(136,80,139,95,GREEN);	
          Lcd_Draw_Bar(141,80,144,95,GREEN);
          Lcd_Draw_Bar(146,80,149,95,GREEN);	
          Lcd_Draw_Bar(151,80,154,95,GREEN);
          break;
        case 3:
          Lcd_Draw_Bar(151,80,154,95,BLACK);
          Lcd_Draw_Bar(146,80,149,95,GREEN);	
          break;
        case 2:
          Lcd_Draw_Bar(146,80,149,95,BLACK);
          Lcd_Draw_Bar(141,80,144,95,GREEN);
          break;
        case 1:
          Lcd_Draw_Bar(141,80,144,95,BLACK);
          Lcd_Draw_Bar(136,80,139,95,GREEN);
          break;
        case 0:
          Lcd_Draw_Bar(136,80,139,95,BLACK);
          break;
        }
        */
        
        
			}
		}
    
    /*
    //자이로 체크
    if(TIM4_Expired)
    {
      //printf("%d\n", tt);
      
      for (i=0; i<10 ; i++)
	        Lcd_Draw_Bar(137-1+10+1,37-5-1-5-6+1-6*i - tt*2,137-1+10+3-1,37-5-1-5-6+3-1-6*i - tt*2, BLACK);
      
      tt++;
      tt = tt%3;
      
      //printf("%d\n", tt);
      
      for (i=0; i<10; i++)
	        Lcd_Draw_Bar(137-1+10+1,37-5-1-5-6+1-6*i - tt*2,137-1+10+3-1,37-5-1-5-6+3-1-6*i - tt*2, WHITE);
      
      TIM4_Expired = 0;
      x = I2C_getbyte(0x38,0x29); //X-axis to read..
      y = I2C_getbyte(0x38,0x2b); //Y-Axis to read..
      z = I2C_getbyte(0x38,0x2d); //Z Axis to read..

      
      if(z<0)
      {
        if(gyro_accident_flag == 0)
        {
          //Uart1_Printf("lock\n");
          gyro_accident_flag = 1;
          mqtt_publish(&broker, "sensor/1/gyro", "accident!", 0);
        }
      }
      else
      {
        if(gyro_accident_flag == 1)
        {
          //Uart1_Printf("unlock\n");
          gyro_accident_flag = 0;
        }
      }
      
      Uart1_Printf("X: %3d, Y: %3d, Z: %3d\n",x, y, z);
      ADC1_ISR_Start();
    }
    //조도
    if(ADC1_flag)
		{
			AD_value = (ADC1_value*3.3)/4096.;
			//Uart1_Printf("The current AD value = %4.2fV \r\n", AD_value);
			ADC1_flag = 0;   
		}
    
      
    if(Key_Value)
    {
      switch(Key_Value)
      {
      case 1:
        mqtt_publish(&broker, "sensor/1/1", "change lane left", 0);
        break;
      case 2:
        mqtt_publish(&broker, "sensor/1/2", "change lane right", 0);
        break;
      case 4:
        mqtt_publish(&broker, "sensor/1/4", "go Home! / leave Home!", 0);
        break;
      }
      //mqtt_publish(&broker, "home", "go Home!", 0);
      //mqtt_publish(&broker, "key/2", "go Home!", 0);
      Key_Value = 0;
    }
    
    if(honk == 1)
    {
      TIM3_Buzzer_Beep(5, 10);
      TIM3_Buzzer_Beep(6, 10);
      TIM2_Delay(100);
      
      honk = 0;
    }
    */
    //LED_Display(1);
    //TIM2_Delay(1000);
    //LED_Display(0);
    //TIM2_Delay(1000);

	}
}
 

