#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"

#define CHECK_POINT 100

void *receive_func(void* param)
{
    int reclen=0;
    VCI_CAN_OBJ rec[1000];
    int i, j;
    unsigned long total;
    int check_point;

    int *run=(int*)param;
    int ind=((*run)>>4);
    VCI_CAN_OBJ snd;
    snd.ID=0;
    snd.SendType=0;
    snd.RemoteFlag=0;
    snd.ExternFlag=1;
    snd.DataLen=8;

    total = 0;
    check_point = 0;
    while((*run)&0x0f)
    {
        if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,1000,100))>0)
        {
	    total += reclen;
#if 0
	    for (j = 0; j < reclen; j++) {
                printf("CAN%d RX: %08X",ind,rec[j].ID);
                for(i=0;i<rec[j].DataLen;i++)
                    printf(" %02X",rec[j].Data[i]);
                printf("\n");
	    }
#endif
	    
#if 1
	    if (total / CHECK_POINT >= check_point) {
                printf("CAN%d: %ld frames received\n", ind, total);
		        check_point++;
	    }
#endif
        }
    }

    printf("CAN%d RX thread terminated, %ld frames received\n", ind, total);

    pthread_exit(0);
}


main()
{
    if(VCI_OpenDevice(VCI_USBCAN2,0,0)!=1)
    {
        printf("VCI_OpenDevice failed\n");
        goto End;
    }
    printf("VCI_OpenDevice succeeded\n");

    VCI_INIT_CONFIG config;
    config.AccCode=0;
    config.AccMask=0xffffffff;
    config.Filter=1;
    config.Mode=0;

    config.Timing0=0x00;
    config.Timing1=0x14;


    if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
    {
        printf("VCI_InitCAN(0) failed\n");
        goto End;
    }
    printf("VCI_InitCAN(0) succeeded\n");

    if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
    {
        printf("VCI_StartCAN(0) failed\n");
        goto End;
    }
    printf("VCI_StartCAN(0) succeeded\n");

    if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
    {
        printf("VCI_InitCAN(1) failed\n");
        goto End;
    }
    printf("VCI_InitCAN(1) succeeded\n");

    if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
    {
        printf("VCI_StartCAN(1) failed\n");
        goto End;
    }
    printf("VCI_StartCAN(1) succeeded\n");

    VCI_CAN_OBJ send[6];
    send[0].ID=0;
    send[0].SendType=2;
    send[0].RemoteFlag=0;
    send[0].ExternFlag=1;
    send[0].DataLen=8;
    send[1]=send[0];
    send[2]=send[0];
    send[3]=send[0];
    send[4]=send[0];
    send[5]=send[0];
    send[1].ID=1;
    send[2].ID=2;

    int i=0;

    for(i=0;i<send[0].DataLen;i++)
    {
        send[0].Data[i]=i;
        send[1].Data[i]=i;
        send[2].Data[i]=i;
    }

    int m_run0=1;
    int m_run1=0x11;

    pthread_t threadid;
    pthread_t threadid1;

    int ret;
    ret=pthread_create(&threadid,NULL,receive_func,&m_run0);

    ret=pthread_create(&threadid1,NULL,receive_func,&m_run1);

    int frames=0x0000;
    int sendind=1;
    int reclen=0;
    VCI_CAN_OBJ rec[100];
    time_t tm1,tm2;

    printf("Test transferring %d frames per-channel, press <ENTER> to start...\n", frames);

    getchar();

    time(&tm1);

    int c = frames;
    while(c--)
    {
        send[0].ID=sendind++;
        if(VCI_Transmit(VCI_USBCAN2,0,0,send,1)>0)
        {
#if 0
            printf("CAN0 TX: %08X",send[0].ID);
            for(i=0;i<send[0].DataLen;i++)
            {
                printf(" %02X",send[0].Data[i]);
            }
            printf("\n");
#endif
        }
        else
            break;

#if 1
        send[0].ID=sendind++;
        if(VCI_Transmit(VCI_USBCAN2,0,1,send,1)>0)
        {
#if 0
            printf("CAN1 TX: %08X",send[0].ID);
            for(i=0;i<send[0].DataLen;i++)
            {
                printf(" %02X",send[0].Data[i]);
            }
            printf("\n");
#endif
        }
        else
            break;
#endif
    }

    time(&tm2);

    printf("TX completed, press <ENTER> to exit RX-thread...\n");
    getchar();

    m_run0=0;
    m_run1=0x10;
    pthread_join(threadid,NULL);
    pthread_join(threadid1,NULL);

    printf("%d frames/channel transferred\n", frames);
    printf("%ld min, %ld sec\n", (tm2-tm1)/60, (tm2-tm1)%60);
    if (tm2 - tm1) {
        printf("speed: %ld fps\n", frames / (tm2 - tm1));
    }

End:
    VCI_CloseDevice(VCI_USBCAN2,0);
}
