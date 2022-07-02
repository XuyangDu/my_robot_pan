#ifndef DEVICEINTERFACE_H
#define DEVICEINTERFACE_H
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>
class DeviceInterface
{
public:
  DeviceInterface();
  virtual bool openDevice()=0;
  virtual void startRecive() =0;
  virtual void processData()=0;

  void stopRecive();
  bool isOpened();

protected:
  virtual bool closeDevice()=0;
  bool opened;
  bool runReceiveThread;
  boost::thread *receiveThread;
  boost::mutex deviceLock;

};

#endif // DEVICEINTERFACE_H
