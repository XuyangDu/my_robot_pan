#include "SerialCom.h"
#include <string.h>
#include <ros/ros.h>

uint16 byte_to_uint16( byte *b )
{
    uint16 ret;
    ret  = (uint16)(ByteCast(b[1]));
    ret |= (uint16)(ByteCast(b[0])) << 8;
    return ret;
}
int16 byte_to_int16( byte *b )
{
    return (int16)byte_to_uint16(b);
}


CSerialCom::CSerialCom(const char * _name, int _speed, int _databits, char _parity, int _stopbits)
{
  name = std::string(_name);
  speed = _speed;
  databits = _databits;
  parity = _parity;
  stopbits = _stopbits;
  m_port_file = -1;
  lastPanPose = lastTiltPose = 0;
  lastPanSpeed = lastTiltSpeed = 0;
}

CSerialCom::~CSerialCom()
{
  stopRecive();
  closeDevice();
}


bool CSerialCom::openDevice()
{
    if (!OpenPort(name.c_str()))
      return false;

    if (!SetPortSpeed(speed))
      return false;

    if (!SetParity(databits, parity, stopbits))
      return false;

    return true;
}

int CSerialCom::PushData(const void *data, int length)
{
  if(!opened)
      return 0;
  int len = 0;
  deviceLock.lock();
  len = write(m_port_file, data, (size_t)length);
  deviceLock.unlock();
  return len;
}

int CSerialCom::ReceiveData(void *buf, int length)
{
    if(!opened)
    {//std::cout<<"not opened!!"<<std::endl;
        return 0;}
  int len = 0;
  deviceLock.lock();
  len = read(m_port_file, buf, length);
  deviceLock.unlock();
  //std::cout<<"opened!!"<<len<<std::endl;
  return len;
}

void CSerialCom::registPose(std::string name, IntValMutex pair)
{
  if(name != std::string("pan") && name != std::string("tilt"))
    ROS_ERROR("Regist pan tilt name error");
  valMap[name]=pair;
}

void CSerialCom::processData()
{
  ROS_INFO("Com receive thread start...");
  char cmd[100];
  strcpy(cmd, "\x52\x54\x02\x01\x01\x01\x0A\x0D");

  byte recvBuffer[100];
  byte dataBuffer[100];
  int size = 0;
  int len = 0;
  int nfds = m_port_file + 1;
  int re;
  fd_set  readfds;
  struct timeval tv;

  int dataLen = 11;

  while(runReceiveThread)
  {
    if(PushData(cmd, 8) == 8)
    {
      len = 0;
      while(1)
      {
        FD_ZERO(&readfds);
        FD_SET(m_port_file, &readfds);
        tv.tv_sec = 0;
        tv.tv_usec = 150000;
        re = select(nfds,&readfds,NULL, NULL, &tv);
        if(re > 0)
        {
            size = ReceiveData(recvBuffer, 100);
            if(size > 0)
            {
                memcpy(dataBuffer+len, recvBuffer, size);
                len += size;
                if(len >= dataLen && dataBuffer[len - 2] == 0x0A && dataBuffer[len - 1] == 0x0D)
                {
                  //  printf("Hei hei %d 0x%02X 0x%02X [%d:%d]\n", len, dataBuffer[len - 2], dataBuffer[len - 1], tv.tv_sec, tv.tv_usec);
                    break;
                }
            }
        }
        else if(re == 0 )
        {
            ROS_ERROR("Receive cam feedback time out!");
            closeDevice();
            usleep(50000);
            if(openDevice())
                printf("Rest com port!\n");
            FD_ZERO(&readfds);
            FD_SET(m_port_file, &readfds);
            nfds = m_port_file + 1;

            break;
        }
        else
        {
            ROS_ERROR("Select cam error!");
            break;
        }
      }

      /*
      for(int i = 0; i < len ; i ++)
          printf("0x%02X ", dataBuffer[i]);
      printf("\n");
       */

      if(len >= dataLen)
      {
         int base = 0;
         if(len > dataLen)
             base = len - dataLen;


         if(dataBuffer[base] == 0x52 && dataBuffer[base + 1] == 0x54 && dataBuffer[base + 9] == 0x0A && dataBuffer[base + 10] == 0x0D)
         {
             int val, val2 = 0;
            val = byte_to_int16((byte*)dataBuffer + 4 + base);
            valMap["pan"].second->lock();
            *(valMap["pan"].first) = val * 0.01;
            valMap["pan"].second->unlock();


            val2=byte_to_int16((byte*)dataBuffer + 6 + base);
            valMap["tilt"].second->lock();
            *(valMap["tilt"].first) = val2 * 0.01;
            valMap["tilt"].second->unlock();

            /*
           // printf("%.2f, %.2f, [%.2f]\n", val * 0.01, val2 *0.01 + 6, rate);
            count +=1;
            if(count == 1)
              startTime = ros::Time::now();
            else if(count == 100)
            {
              rate = count / (ros::Time::now() - startTime).toSec();
              count = 0;
             // printf("Rate : %.2f\n", rate);
            }
            */

         }
         else
         {
             ROS_ERROR("Receive wrong com feedback data!");
             char str[100];
             int t = 0;
             for(int i = 0; i < len ; i ++)
                 t += sprintf(str + t,"0x%02X ", dataBuffer[i]);
             ROS_ERROR(str);

         }
      }
      else
      {
          ROS_ERROR("Receive wrong com feedback data!");
          char str[100];
          int t = 0;
          for(int i = 0; i < len ; i ++)
              t += sprintf(str + t,"0x%02X ", dataBuffer[i]);
          ROS_ERROR(str);
      }
    }
    else
    {
    //  ROS_ERROR("Com send feedback cmd error!");
    }
    usleep(30000);
  }
  ROS_INFO("Com receive thread stop.");
}


void CSerialCom::sendPanCmd(int pose, int speed)
{
    lastPanPose = pose;
    lastPanSpeed = speed;
    sendPanTiltCmd(pose, speed, lastTiltPose, lastTiltSpeed);
}

void CSerialCom::sendTiltCmd(int pose, int speed)
{
    lastTiltPose = pose;
    lastTiltSpeed = speed;
    sendPanTiltCmd(lastPanPose, lastPanSpeed, pose, speed);
}

void CSerialCom::sendPanTiltCmd(int panPose, int panSpeed, int tiltPose, int tiltSpeed)
{
  //  printf("pan: [%d : %d]   tilt: [%d : %d]\n", panPose, panSpeed, tiltPose, tiltSpeed);
    unsigned char CRC=0x00;
    memset(cmd, 0, LIST_LEN);

    cmd[0]=0x52;
    cmd[1]=0x54;

    cmd[2]=0x01;
    cmd[3]=0x0e;

    cmd[4]=0x04;
    cmd[5]=0x03;

    cmd[6]=char(panSpeed>>8);
    cmd[7]=char(panSpeed);
    cmd[8]=char(tiltSpeed>>8);
    cmd[9]=char(tiltSpeed);

    cmd[10]=0x00;
    cmd[11]=0x00;

    cmd[12]=char(panPose>>8);
    cmd[13]=char(panPose);
    cmd[14]=char(tiltPose>>8);
    cmd[15]=char(tiltPose);

    cmd[16]=0x00;
    cmd[17]=0x00;

    for(int i = 4; i < 16; i++ )
        CRC += cmd[i];

    cmd[18]=CRC;
    cmd[19]=0x0a;
    cmd[20]=0x0d;

    int len = PushData(cmd, 21);
    if(len != 21)
    {
        ROS_ERROR("Com set pan tilt motor cmd return an error!");
    }
    /*
    for(int i = 0; i < 21; i++)
        printf("0x%02X ", cmd[i]);
    printf("\n");
    */
}

void CSerialCom::startRecive()
{
  runReceiveThread = true;
  receiveThread = new boost::thread(&CSerialCom::processData, this);
}

bool CSerialCom::closeDevice()
{
  if (opened)
  {
    close(m_port_file);
    opened = false;
    ROS_INFO("Close com %s", name.c_str());
  }
  return true;
}


bool CSerialCom::OpenPort(const char * portName)
{
  m_port_file = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
  if (m_port_file == -1)
  {
    opened = false;
    ROS_ERROR("Open port %s failed!", portName);
    return false;
  }

  fcntl(m_port_file, F_SETFL, FNDELAY);
  opened = true;
  ROS_INFO("Successfully opened %s", portName);
  return true;
}


bool CSerialCom::SetPortSpeed(int speed)
{
  struct termios options;
  if (tcgetattr(m_port_file, &options) != 0)
  {

    ROS_ERROR("Get port attribute error!");
    return false;
  }

  unsigned int setSpeed;
  switch (speed)
  {
    case 300:
      setSpeed = B300;
      break;
    case 1200:
      setSpeed = B1200;
      break;
    case 2400:
      setSpeed = B2400;
      break;
    case 4800:
      setSpeed = B4800;
      break;
    case 9600:
      setSpeed = B9600;
      break;
    case 19200:
      setSpeed = B19200;
      break;
    default:
      ROS_ERROR( "Error! Baud rate not supported!");
      return false;
  }
  if (cfsetispeed(&options, setSpeed) != 0)
  {
    ROS_ERROR("Error! Set in baud rate failed!");
    return false;
  }
  if (cfsetospeed(&options, setSpeed) != 0)
  {
    ROS_ERROR( "Error! Set out baud rate failed!" );
    return false;
  }
  options.c_cflag |= (CLOCAL | CREAD);
  if (tcsetattr(m_port_file, TCSANOW, &options) != 0)
  {
    ROS_ERROR("Error! Set port attribute failed1");
    return false;
  }
  return true;
}

bool CSerialCom::SetParity(int databits, char parity, int stopbits)
{
  struct termios options;
  if (tcgetattr(m_port_file, &options) != 0)
  {
    ROS_ERROR("Error! Get port attribute failed1");
    return false;
  }
  options.c_cflag &= ~CSIZE;

  switch (databits)
  {
    case 7:
      options.c_cflag |= CS7;
      break;
    case 8:
      options.c_cflag |= CS8;
      break;
    default:
      ROS_ERROR("Unsupported data size");
      return false;
  }

  switch (parity)
  {
    case 'n':
    case 'N':
      options.c_cflag &= ~PARENB; // Clear parity enable
      options.c_iflag &= ~INPCK; // Enable parity checking
      break;
    case 'o':
    case 'O':
      options.c_cflag |= (PARODD | PARENB); //Set odd
      options.c_iflag |= INPCK; // Disnable parity checking
      break;
    case 'e':
    case 'E':
      options.c_cflag |= PARENB; // Enable parity
      options.c_cflag &= ~PARODD; // SetEven
      options.c_iflag |= INPCK; //Disnable parity checking
      break;
    case 'S':
    case 's': //as no parity
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      break;
    default:
      ROS_ERROR("Unsupported parity" );
      return false;
  }

  switch (stopbits)
  {
    case 1:
      options.c_cflag &= ~CSTOPB;
      break;
    case 2:
      options.c_cflag |= CSTOPB;
      break;
    default:
      ROS_ERROR("Unsupported stop bits");
      return false;
  }

  /* Set input parity option */
  if (parity != 'n')
    options.c_iflag |= INPCK;

  options.c_cc[VTIME] = 0;
  options.c_cc[VMIN] =  0;

  tcflush(m_port_file, TCIFLUSH);

  if (tcsetattr(m_port_file, TCSANOW, &options) != 0)
  {
    ROS_ERROR("Error! Set port attribute failed1");
    return false;
  }
  return true;
}

