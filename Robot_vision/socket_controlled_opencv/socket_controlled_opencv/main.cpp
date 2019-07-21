#include <calibration_fcn.h>
#include <polygon_detection.h>
#include <common_fcn.h>

// if defined show_cmd then show command window,otherwise 
// it will be hidden
#ifndef show_cmd
#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" ) // hide command window
#endif
#pragma comment (lib, "ws2_32.lib")				//���� ws2_32.dll

void socket_initialization(SOCKET &clntSock, SOCKET &servSock) {
	// Socket things.
	WSADATA wsaData;							//����һ���ṹ�����
	WSAStartup(MAKEWORD(2, 2), &wsaData);		//��ʼ��WSAStartup()����,,,(�淶�İ汾�ţ�ָ��WSADATA�ṹ���ָ��),,,MSKEWORD(2,2)���汾��2�����汾��2


	servSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);		//������1��IP��ַ����PF_INET6ΪIPv6����2�����ݴ��䷽ʽ SOCK_STREAM �� SOCK_DGRAM ��3������Э�� IPPROTO_TCP �� IPPTOTO_UDP�����д0ϵͳ���Զ����㴦ʹ������Э��
	sockaddr_in sockAddr;										//����sockaddr_in�ṹ�����
	memset(&sockAddr, 0, sizeof(sockAddr));						//ÿ���ֽڶ���0���
	sockAddr.sin_family = PF_INET;								//ʹ��IPv4��ַ
	sockAddr.sin_port = htons(1234);							//�˿ں�   Ҫ�õ�htons()����ת��

	/*
	struct in_addr addr;
	inet_pton(AF_INET, "192.168.137.1", (void*)&addr);
	sockAddr.sin_addr = addr;			//�����IP��ַ32λ
	*/

	sockAddr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);

	bind(servSock, (SOCKADDR*)&sockAddr, sizeof(SOCKADDR));				//���׽��֣�
	listen(servSock, 20);
	printf("Server ready,waiting for connection.\n");
	SOCKADDR clntAddr;
	int nSize = sizeof(SOCKADDR);
	clntSock = accept(servSock, (SOCKADDR*)&clntAddr, &nSize);
	printf("Connected.\n");
	// End of socket things.
}

extern uchar conveyor_color;
// first,select camera and robot mounting method
// then chose function
int main(int argc, char** argv) {
	SOCKET clntSock, servSock;
	socket_initialization(clntSock, servSock);
	// robot-cam mounting method selection
	uint8_t choice;
	bool function_selected = false;
	bool robot_model_selected = false;
	uchar robot_model = NOT_SELECTED;
	while (true) {
		while (!function_selected) {
			char buffer[10] = { 0 };
			while (!robot_model_selected) {
				socket_send(clntSock, "Please input camera calibration method.\n1.Fixed_cam_above\n2.cam_with_conveyor\n3.eye_in_hand\nQ.quit\n");
				recv(clntSock, buffer, strlen(buffer) + sizeof(char), NULL);
				if (!strcmp(buffer, "1")) {
					robot_model = Fixed_cam_above;
					robot_model_selected = true;
					break;
				}
				else if (!strcmp(buffer, "2")) {
					robot_model = Cam_with_conveyor;
					robot_model_selected = true;
					break;
				}
				else if (!strcmp(buffer, "3")) {
					robot_model = Eye_in_hand;
					robot_model_selected = true;
					break;
				}
				else if (!strcmp(buffer, "Q") || !strcmp(buffer, "q")) {
					robot_model = QUIT;
					robot_model_selected = false;
					break;
				}
			}
			if (robot_model == QUIT) {
				choice = 0;
				socket_send(clntSock, "\nExiting program.\n");
				closesocket(clntSock);
				closesocket(servSock);
				WSACleanup();
				return 1;
			}
			else {
				if (robot_model == Fixed_cam_above) {
					while (true) {
						socket_send(clntSock, "Fixed cam above program,please input conveyor ID, 1 is white conveyor,2 is green conveyor\n");
						recv(clntSock, buffer, strlen(buffer) + sizeof(char), NULL);
						if (!strcmp(buffer, "1")) {
							function_selected = true;
							conveyor_color = 'W';
							break;
						}
						else if (!strcmp(buffer, "2")) {
							function_selected = true;
							conveyor_color = 'G';
							break;
						}
					}
					while (true) {
						socket_send(clntSock, "OpenCV calibration and polygon detection program.\nSelect function:\n1:calibration\n2:polygons detection\nQ:quit\n");
						recv(clntSock, buffer, strlen(buffer) + sizeof(char), NULL);
						if (!strcmp(buffer, "1")) {
							function_selected = true;
							choice = 1;
							break;
						}
						else if (!strcmp(buffer, "2")) {
							function_selected = true;
							choice = 2;
							break;
						}
						else if (!strcmp(buffer, "Q")) {
							socket_send(clntSock, "\nExiting program.\n");
							closesocket(clntSock);
							closesocket(servSock);
							WSACleanup();
							return 2;
						}
					}	
				}
				else if (robot_model == Cam_with_conveyor) {
					while (true) {
						// function selection
						socket_send(clntSock, "OpenCV calibration and polygon detection program.\nSelect function:\n1:calibration\n2:polygons detection\nQ:quit\n");
						recv(clntSock, buffer, strlen(buffer) + sizeof(char), NULL);
						if (!strcmp(buffer, "1")) {
							function_selected = true;
							choice = 1;
							break;
						}
						else if (!strcmp(buffer, "2")) {
							function_selected = true;
							choice = 2;
							break;
						}
						else if (!strcmp(buffer, "Q")) {
							socket_send(clntSock, "\nExiting program.\n");
							closesocket(clntSock);
							closesocket(servSock);
							WSACleanup();
							return 2;
						}
					}
				}
				else if(robot_model==Eye_in_hand){
					while (true) {
						// function selection
						socket_send(clntSock, "OpenCV calibration and polygon detection program.\nSelect function:\n1:calibration for polygonbox\n2:polygons detection\n3:calibration for glass cover\n4:iPhone glass cover\nQ:quit\n");
						recv(clntSock, buffer, strlen(buffer) + sizeof(char), NULL);
						if (!strcmp(buffer, "1")) {
							function_selected = true;
							choice = 1;
							break;
						}
						else if (!strcmp(buffer, "2")) {
							function_selected = true;
							choice = 2;
							break;
						}
						else if (!strcmp(buffer, "3")) {
							function_selected = true;
							choice = 3;
							break;
						}
						else if (!strcmp(buffer, "4")) {
							function_selected = true;
							choice = 4;
							break;
						}
						else if (!strcmp(buffer, "Q")) {
							socket_send(clntSock, "\nExiting program.\n");
							closesocket(clntSock);
							closesocket(servSock);
							WSACleanup();
							return 2;
						}
					}
					
				}
			}
		}
		switch (choice) {
			case 1:	calibration_wizzad(servSock, clntSock, robot_model, polygon_box);	break;
			case 2: polygon_detect_wizzad(servSock, clntSock, robot_model, polygon_box); break;
			case 3:	calibration_wizzad(servSock, clntSock, robot_model, glass_cover);	break;
			case 4: polygon_detect_wizzad(servSock, clntSock, robot_model,glass_cover); break;
		}
		function_selected = false;
		robot_model_selected = false;
	}
	return 0;
}


