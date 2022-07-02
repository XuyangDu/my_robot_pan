#include<iostream>
#include<string>
#include<stdlib.h>
#include<string.h>
#include<stdio.h>
#include<boost/thread/thread.hpp>
#include<boost/thread/pthread/recursive_mutex.hpp>
#include"ControlCan.h"
#include <ros/ros.h>
#include <fstream>
#include <sstream>


#define SLEEP usleep(100000)

using namespace std;

DWORD deviceType = 3;
DWORD deviceID= 0;
DWORD canID = 0;
boost::recursive_mutex can_lock;

ULONG Xmit(BYTE id, int16 index, int16 sub, int32 size, BYTE* data)
{
    if(size > 4)
    {
        cout<<"Xmit error! Data length must <= 4 !"<<endl;
        return 0;
    }
    VCI_CAN_OBJ frame;
    memset(&frame, 0, sizeof(VCI_CAN_OBJ));
    frame.ID = 0x000000600 + id;
    frame.SendType = 0;
    frame.RemoteFlag = 0;
    frame.ExternFlag = 0;
    frame.DataLen = 8;
    frame.Data[0] = (size == 1? 47:(size == 2? 43 : 35));
    frame.Data[1] = ByteCast(index);
    frame.Data[2] = ByteCast(index >>8);
    frame.Data[3] = ByteCast(sub);
    for(int i = 0; i < size; i++)
        frame.Data[4+i] = data[i];

    can_lock.lock();
    ULONG e= VCI_Transmit(deviceType, deviceID, canID, &frame, 1);
    can_lock.unlock();

    printf("CAN.X: 0x%08X - ",frame.ID);
    for(int j = 0; j < frame.DataLen; j++)
        printf("0x%02X ", frame.Data[j]);
    printf("  -- %d\n", e);

    return e;
}

ULONG Xmit(BYTE id, int16 index, int16 sub, uint32 data)
{
    BYTE arr[4];
    arr[0] = ByteCast(data);
    arr[1] = ByteCast(data>>8);
    arr[2] = ByteCast(data>>16);
    arr[3] = ByteCast(data>>24);
    return Xmit(id, index, sub, 4, arr);
}

ULONG Xmit(BYTE id, int16 index, int16 sub, uint16 data)
{
    BYTE arr[2];
    arr[0] = ByteCast(data);
    arr[1] = ByteCast(data>>8);
    return Xmit(id, index, sub, 2, arr);
}

ULONG Xmit(BYTE id, int16 index, int16 sub, BYTE data)
{
    return Xmit(id, index, sub, 1, &data);
}

ULONG Xmit(unsigned char id, int16 index, int16 sub, int32 data)
{
    return Xmit(id, index, sub, (uint32)(data));
}

ULONG Xmit(unsigned char id, int16 index, int16 sub, int16 data)
{
    return Xmit(id, index, sub, (uint16)(data));
}
ULONG Xmit(unsigned char id, int16 index, int16 sub, char data)
{
    return Xmit(id, index, sub, (BYTE)data);
}

ULONG Xmit(uint32 id, BYTE*data, int len)
{
    if (len > 8)
        return 0;
    VCI_CAN_OBJ frame;
    memset(&frame, 0, sizeof(VCI_CAN_OBJ));
    frame.ID = id;
    frame.SendType = 0;
    frame.RemoteFlag = 0;
    frame.ExternFlag = 0;
    frame.DataLen = len;
    for(int i = 0; i < len; i++)
        frame.Data[i] = data[i];

    can_lock.lock();
    ULONG e= VCI_Transmit(deviceType, deviceID, canID, &frame, 1);
    can_lock.unlock();

    printf("CAN.X: 0x%08X - ",frame.ID);
    for(int j = 0; j < frame.DataLen; j++)
        printf("0x%02X ", frame.Data[j]);
    printf("  -- %d\n", e);

    return e;
}

ULONG Xmit(uint32 id, BYTE data1 = 0, BYTE data2 = 0, BYTE data3 = 0, BYTE data4 = 0, BYTE data5 = 0, BYTE data6 = 0, BYTE data7 = 0, BYTE data8 = 0 )
{
    BYTE data[8];
    data[0] = data1;
    data[1] = data2;
    data[2] = data3;
    data[3] = data4;
    data[4] = data5;
    data[5] = data6;
    data[6] = data7;
    data[7] = data8;
    return Xmit(id,data,8);
}


void receive()
{

    VCI_CAN_OBJ dataBuffer[1024];
    int num = 0;
    while(1)
    {
        can_lock.lock();
        num = VCI_Receive(deviceType,deviceID,canID,dataBuffer,1024, 10);
        can_lock.unlock();
        if(num == 0xFFFFFFFF)
        {
            cout<<"Receive data error!"<<endl;
        }

        if(num)
        {
            for(int i = 0; i != num; i++)
            {
                printf("CAN.R: 0x%08X - ",dataBuffer[i].ID);
                for(int j = 0; j < dataBuffer[i].DataLen; j++)
                    printf("0x%02X ", dataBuffer[i].Data[j]);
                putchar('\n');
                putchar('\n');



                if(dataBuffer[i].Data[0] == ByteCast(0x80))
                {
                    cout <<" *******************  Download Data Error ******************* "<<endl;
                }

            }
        }


        usleep(10000);
    }
}

void loadFile(const char* path)
{
    ifstream file(path);
    string line;
    stringstream ss;
    uint32 id;
    int data[8];
    char c[100];
    if(file)
    {
        while(getline(file,line))
        {
            if(line[0] == 'C')
            {
              //  cout <<line<<endl;
                memset(data,0,sizeof(int));
                sscanf(line.c_str(),"%s %X - %X %X %X %X %X %X %X %X",c, &id, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6], &data[7]);
                //printf("%s %#010X - %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X\n\n",c, id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
                Xmit(id,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
                usleep(1);

            }

        }

    }

}


int main()
{


    DWORD err = VCI_OpenDevice(deviceType, deviceID, 0);
    if(! err)
    {
        cout<<"Open can failed!"<<endl;
        return 0;
    }

    VCI_INIT_CONFIG initCfg;
    memset(&initCfg, 0, sizeof(VCI_INIT_CONFIG));
    initCfg.AccMask = 0xffffffff;
    initCfg.Filter = 1;
    initCfg.Timing0 = 0x00;
    initCfg.Timing1 = 0x14;
    initCfg.Mode = 0;
    err = VCI_InitCAN(deviceType,deviceID,canID,&initCfg);
    if(!err)
    {
        VCI_CloseDevice(deviceType,deviceID);
        cout<<"Init can failed!"<<endl;
        return 0;
    }

    VCI_BOARD_INFO boardInfo;
    err = VCI_ReadBoardInfo(deviceType,deviceID,&boardInfo);
    if(!err)
    {
        VCI_CloseDevice(deviceType,deviceID);
        cout<<"Read board info failed"<<endl;
        return 0;
    }
    else
    {
        cout <<"------------ Board Info ------------"<<endl;
        cout <<"         HW V: "<<boardInfo.hw_Version<<endl;
        cout <<"         FW V: "<<boardInfo.fw_Version<<endl;
        cout <<"         Dr V: "<<boardInfo.dr_Version<<endl;
        cout <<"         Lb V: "<<boardInfo.in_Version<<endl;
        cout <<"         CN V: "<<(int)boardInfo.can_Num<<endl;
        cout <<"         Sn V: "<<boardInfo.str_Serial_Num<<endl;
        cout <<"         Ty V: "<<boardInfo.str_hw_Type<<endl;
        cout <<"------------------------------------"<<endl;
    }

    err = VCI_ClearBuffer(deviceType, deviceID, canID);
    if(!err)
    {
        VCI_CloseDevice(deviceType,deviceID);
        cout<<"Clear can buffer failed"<<endl;
        return 0;
    }

    err = VCI_StartCAN(deviceType, deviceID, canID);
    if(!err)
    {
        VCI_CloseDevice(deviceType,deviceID);
        cout<<"Start can failed"<<endl;
        return 0;
    }

    boost::thread a = boost::thread(boost::bind(&receive));


    int nodeID =12;
    //stop mode
    Xmit(nodeID, 0x605A, 0, (int16)2);
    //halt mode
    Xmit(nodeID, 0x605D, 0, (int16)1);
    //set amp mode
    Xmit(nodeID, 0x2300, 0, (int16)(0x1E00>>8));
    //start node
    Xmit(0x00000000, 0x01, ByteCast(nodeID));
    //set profile mode
    Xmit(nodeID, 0x6060, 0, (char)1);
    //clear event status
    Xmit(nodeID, 0x2181, 0, (int32)0xFFFFFFFF);
    //clear faults
    Xmit(nodeID, 0x2183, 0, (uint32)0);
    //Enable
    Xmit(nodeID, 0x6040, 0, (int16)0x0F);

    //set profie
    Xmit(nodeID, 0x6086, 0, (int16)0);
    Xmit(nodeID, 0x607A, 0, 0);
    Xmit(nodeID, 0x6081, 0, 3000000);
    Xmit(nodeID, 0x6083, 0, 55000);
    Xmit(nodeID, 0x6084, 0, 55000);
    Xmit(nodeID, 0x2121, 0, 80000);
    Xmit(nodeID, 0x6085, 0, 50000);

    //set Halt mode
   // Xmit(nodeID, 0x605d, 0, (int16)3);
    //enable
    Xmit(nodeID, 0x6040, 0, (int16)0x0F);
    //set profile mode
    Xmit(nodeID, 0x6060, 0, (char)1);






    //测试正反转


    sleep(1);
    printf("    ------------        正转         --------------------\n");
    Xmit(nodeID, 0x607A, 0, 100000000);
    Xmit(nodeID, 0x6040, 0, (int16)0x0F);
    Xmit(nodeID, 0x6040, 0, (int16)0x3F);
    sleep(6);

    printf("    ------------        反转         --------------------\n");
    Xmit(nodeID, 0x607A, 0, -100000000);
    Xmit(nodeID, 0x6040, 0, (int16)0x0F);
    Xmit(nodeID, 0x6040, 0, (int16)0x3F);
    sleep(6);

    printf("    ------------       stop          --------------------\n");
    Xmit(nodeID, 0x6040, 0, (int16)0x010F);
    sleep(6);

    printf("    ------------        continue         --------------------\n");
    Xmit(nodeID, 0x6040, 0, (int16)0x000F);
    Xmit(nodeID, 0x6040, 0, (int16)0x003F);
    sleep(6);

    printf("    ------------        break         --------------------\n");
    Xmit(nodeID, 0x6040, 0, (int16)0x05);
    sleep(10);


    printf("    ------------        enable         --------------------\n");
    Xmit(nodeID, 0x6040, 0, (int16)0x0F);
    sleep(10);



    string cmd;
    while(1)
    {
        cin>>cmd;
        loadFile(cmd.c_str());
    }

    VCI_CloseDevice(deviceType,deviceID);
    return 0;
}
