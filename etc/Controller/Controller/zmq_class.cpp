#include "zmq_class.h"

ZMQ_CLASS::ZMQ_CLASS()
  :context_(1)	//zmq constructor dealing with the initialisation and termination of a zmq context
{
  if(!readParameters())
  {
	  perror("readParameters");
	  exit(1);
  }

  init();
}

ZMQ_CLASS::~ZMQ_CLASS()
{
  std::cout << "Disconnected" << std::endl;
  controlDone_ = true;
  req_socket0_.close();
  req_socket1_.close();
  req_socket2_.close();

  delete req_recv0_;
  delete req_recv1_;
  delete req_recv2_;

  context_.close();
}

void ZMQ_CLASS::init()
{
  controlDone_ = false;

  /* Initialize zmq data */
  req_recv0_ = new ZmqData;
  req_recv1_ = new ZmqData;
  req_recv2_ = new ZmqData;

  /* Initialize Tcp client(Request) Socket */
  if(req_flag0_)
  {
    req_socket0_ = zmq::socket_t(context_, ZMQ_REQ);
    req_socket0_.connect(tcpreq_ip0_);
    req_socket0_.setsockopt(ZMQ_RCVTIMEO, -1);  //timeout (infinite)
    req_socket0_.setsockopt(ZMQ_LINGER, 0);
  }

  if(req_flag1_)
  {
    req_socket1_ = zmq::socket_t(context_, ZMQ_REQ);
    req_socket1_.connect(tcpreq_ip1_);
    req_socket1_.setsockopt(ZMQ_RCVTIMEO, -1);
    req_socket1_.setsockopt(ZMQ_LINGER, 0);
  }

  if(req_flag2_)
  {
    req_socket2_ = zmq::socket_t(context_, ZMQ_REQ);
    req_socket2_.connect(tcpreq_ip2_);
    req_socket2_.setsockopt(ZMQ_RCVTIMEO, -1);
    req_socket2_.setsockopt(ZMQ_LINGER, 0);
  }

}

std::string ZMQ_CLASS::getIPAddress(){
  std::string ipAddress="Unable to get IP Address";
  struct ifaddrs *interfaces = NULL;
  struct ifaddrs *temp_addr = NULL;
  int success = 0;
  success = getifaddrs(&interfaces);
  if (success == 0)
  {
    temp_addr = interfaces;
    while(temp_addr != NULL)
    {
      if(temp_addr->ifa_addr->sa_family == AF_INET)
      {
        if(strcmp(temp_addr->ifa_name, interface_name_.c_str())==0)
	{
          ipAddress = inet_ntoa(((struct sockaddr_in*)temp_addr->ifa_addr)->sin_addr);
	}
      }
      temp_addr = temp_addr->ifa_next;
    }
    freeifaddrs(interfaces);
  }
  return ipAddress;
}

bool ZMQ_CLASS::readParameters()
{
  std::string tcp_ip_server, tcp_ip_client0, tcp_ip_client1, tcp_ip_client2, tcpreq_port0, tcpreq_port1, tcpreq_port2;
  std::string udp_ip, udp_port;
  interface_name_ = std::string("ens33");

  tcp_ip_server = std::string("tcp://*");
  tcp_ip_client0 = std::string("tcp://192.168.0.10");  //LV
  tcp_ip_client1 = std::string("tcp://192.168.0.11");  //FV1
  tcp_ip_client2 = std::string("tcp://192.168.0.12");  //FV2

  tcpreq_port0 = std::string("7777");  //for LV
  tcpreq_port1 = std::string("8888");  //for FV1
  tcpreq_port2 = std::string("9999");  //for FV2

  zipcode_ = std::string("00020");
  
  req_flag0_ = true;
  req_flag1_ = true;
  req_flag2_ = true;

  //set reply socket ip
  tcpreq_ip0_ = tcp_ip_client0;
  tcpreq_ip0_.append(":");
  tcpreq_ip0_.append(tcpreq_port0);

  tcpreq_ip1_ = tcp_ip_client1;
  tcpreq_ip1_.append(":");
  tcpreq_ip1_.append(tcpreq_port1);

  tcpreq_ip2_ = tcp_ip_client2;
  tcpreq_ip2_.append(":");
  tcpreq_ip2_.append(tcpreq_port2);

  return true;
}

void* ZMQ_CLASS::requestZMQ(ZmqData* send_data)  // client: send -> recv
{ 
  if(send_data->tar_index == 0){
    if(req_socket0_.connected() && !controlDone_)
    {
      zmq::message_t recv_msg(DATASIZE), send_msg(DATASIZE);
      //send
      memcpy(send_msg.data(), send_data, DATASIZE);
      req_socket0_.send(send_msg);

      //recv
      req_socket0_.recv(&recv_msg, 0);
      memcpy(req_recv0_, recv_msg.data(), DATASIZE);
//      req_recv0_ = static_cast<ZmqData *>(rep_msg.data());
    }
  }
  if(send_data->tar_index == 1){
    if(req_socket1_.connected() && !controlDone_)
    {
      zmq::message_t recv_msg(DATASIZE), send_msg(DATASIZE);
      //send
      memcpy(send_msg.data(), send_data, DATASIZE);
      req_socket1_.send(send_msg);

      //recv
      req_socket1_.recv(&recv_msg, 0);
      memcpy(req_recv1_, recv_msg.data(), DATASIZE);
//      req_recv1_ = static_cast<ZmqData *>(rep_msg.data());
    }
  }
  if(send_data->tar_index == 2){
    if(req_socket2_.connected() && !controlDone_)
    {
      zmq::message_t recv_msg(DATASIZE), send_msg(DATASIZE);
      //send
      memcpy(send_msg.data(), send_data, DATASIZE);
      req_socket2_.send(send_msg);

      //recv
      req_socket2_.recv(&recv_msg, 0);
      memcpy(req_recv2_, recv_msg.data(), DATASIZE);
//      req_recv2_ = static_cast<ZmqData *>(rep_msg.data());
    }
  }
}
