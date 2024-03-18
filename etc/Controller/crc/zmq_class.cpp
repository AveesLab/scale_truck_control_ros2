#include "includes/zmq_class.h"

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
  req_socket_.close();
  rep_socket0_.close();
  rep_socket1_.close();
  rep_socket2_.close();
  rad_socket_.close();
  dsh_socket_.close();

  delete req_recv_;
  delete rep_recv0_;
  delete rep_recv1_;
  delete rep_recv2_;

  context_.close();
}

void ZMQ_CLASS::init()
{
  controlDone_ = false;

  /* Initialize zmq data */
  req_recv_ = new ZmqData;
  rep_recv0_ = new ZmqData;
  rep_recv1_ = new ZmqData;
  rep_recv2_ = new ZmqData;

  /* Initialize Tcp client(Request) Socket */
  if(req_flag_)
  {
    req_socket_ = zmq::socket_t(context_, ZMQ_REQ); 
    req_socket_.connect(tcpreq_ip_);
    req_socket_.setsockopt(ZMQ_RCVTIMEO, 10000);  //timeout (millisecends) 
    req_socket_.setsockopt(ZMQ_LINGER, 0); 
  }

  /* Initialize Tcp server(Reply) Socket */
  if(rep_flag0_)
  {
    rep_socket0_ = zmq::socket_t(context_, ZMQ_REP);
    rep_socket0_.bind(tcprep_ip0_);
  }

  if(rep_flag1_)
  {
    rep_socket1_ = zmq::socket_t(context_, ZMQ_REP);
    rep_socket1_.bind(tcprep_ip1_);
  }

  if(rep_flag2_)
  {
    rep_socket2_ = zmq::socket_t(context_, ZMQ_REP);
    rep_socket2_.bind(tcprep_ip2_);
  }

  /* Initialize Udp send(Radio) Socket */
  if(rad_flag_)
  {
    rad_socket_ = zmq::socket_t(context_, ZMQ_RADIO);
    rad_socket_.connect(udp_ip_);
  }

  /* Initialize Udp recv(Dish) Socket */
  if(dsh_flag_)
  {
    dsh_socket_ = zmq::socket_t(context_, ZMQ_DISH);
    dsh_socket_.bind(udp_ip_);
    dsh_socket_.join(dsh_group_.c_str());
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
    return ipAddress;
  }
}

bool ZMQ_CLASS::readParameters()
{
  std::string tcp_ip_server, tcp_ip_client, tcpreq_port, tcprep_port0, tcprep_port1, tcprep_port2;
  std::string udp_ip, udp_port;
  interface_name_ = std::string("ens33");

  tcp_ip_server = std::string("tcp://*");
//  tcp_ip_client = std::string("tcp://192.168.0.19");

  tcpreq_port = std::string("3333");
  tcprep_port0 = std::string("4444");  //for LV
  tcprep_port1 = std::string("5555");  //for FV1
  tcprep_port2 = std::string("6666");  //for FV2

  zipcode_ = std::string("00011");

  udp_ip = std::string("udp://239.255.255.250");
  udp_port = std::string("9090");
  rad_group_ = std::string("?");
  dsh_group_ = std::string("CRC");
  
  req_flag_ = false;
  rep_flag0_ = true;
  rep_flag1_ = true;
  rep_flag2_ = true;
  rad_flag_ = false;
  dsh_flag_ = false;

  //set request socket ip
  tcpreq_ip_ = tcp_ip_client;
  tcpreq_ip_.append(":");
  tcpreq_ip_.append(tcpreq_port);

  //set reply socket ip
  tcprep_ip0_ = tcp_ip_server;
  tcprep_ip0_.append(":");
  tcprep_ip0_.append(tcprep_port0);

  tcprep_ip1_ = tcp_ip_server;
  tcprep_ip1_.append(":");
  tcprep_ip1_.append(tcprep_port1);

  tcprep_ip2_ = tcp_ip_server;
  tcprep_ip2_.append(":");
  tcprep_ip2_.append(tcprep_port2);

  //set radio/dish socket ip
  udp_ip_ = udp_ip;
  udp_ip_.append(":");
  udp_ip_.append(udp_port);

  return true;
}

void* ZMQ_CLASS::requestZMQ(ZmqData* send_data)  // client: send -> recv
{ 
  if(req_socket_.connected() && !controlDone_)
  {
    zmq::message_t recv_msg(DATASIZE), send_msg(DATASIZE);

    //send
    memcpy(send_msg.data(), send_data, DATASIZE);
    req_socket_.send(send_msg);

    //recv
    req_socket_.recv(&recv_msg, 0);
    memcpy(req_recv_, recv_msg.data(), DATASIZE);
//    req_recv_ = static_cast<ZmqData *>(rep_msg.data());
  }
}

void* ZMQ_CLASS::replyZMQ(ZmqData* send_data)  //server: recv -> send
{
  zmq::message_t recv_msg(DATASIZE), send_msg(DATASIZE);

  if(send_data->tar_index == 10){  //LV LRC
    if(rep_socket0_.connected() && !controlDone_)
    {
      //recv
      rep_socket0_.recv(&recv_msg, 0);
      memcpy(rep_recv0_, recv_msg.data(), DATASIZE);
//      rep_recv0_ = static_cast<ZmqData *>(recv_msg.data()); 

      //send
      memcpy(send_msg.data(), send_data, DATASIZE);
      rep_socket0_.send(send_msg);  
    }
  }
  else if(send_data->tar_index == 11){  //FV1 LRC
    if(rep_socket1_.connected() && !controlDone_)
    {
      //recv
      rep_socket1_.recv(&recv_msg, 0);
      memcpy(rep_recv1_, recv_msg.data(), DATASIZE);
//      rep_recv1_ = static_cast<ZmqData *>(recv_msg.data()); 
  
      //send
      memcpy(send_msg.data(), send_data, DATASIZE);
      rep_socket1_.send(send_msg);  
    }
  }
  else if(send_data->tar_index == 12){  //FV2 LRC
    if(rep_socket2_.connected() && !controlDone_)
    {
      //recv
      rep_socket2_.recv(&recv_msg, 0);
      memcpy(rep_recv2_, recv_msg.data(), DATASIZE);
//      rep_recv2_ = static_cast<ZmqData *>(recv_msg.data()); 
  
      //send
      memcpy(send_msg.data(), send_data, DATASIZE);
      rep_socket2_.send(send_msg);  
    }
  }
}

void* ZMQ_CLASS::radioZMQ(ZmqData *send_data)
{
  while(rad_socket_.connected() && !controlDone_)
  {
    zmq::message_t pub_msg(DATASIZE);
    pub_msg.set_group(rad_group_.c_str());
 
    //pub
    memcpy(pub_msg.data(), send_data, DATASIZE);
    rad_socket_.send(pub_msg, 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
}

void* ZMQ_CLASS::dishZMQ()
{
  while(dsh_socket_.connected() && !controlDone_)
  {
    zmq::message_t sub_msg(DATASIZE);

    //sub
    dsh_socket_.recv(&sub_msg, 0);
    dsh_recv_ = static_cast<ZmqData *>(sub_msg.data());

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
}
