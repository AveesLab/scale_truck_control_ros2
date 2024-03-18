#include "zmq_class/zmq_class.h"

ZMQ_CLASS::ZMQ_CLASS(ros::NodeHandle nh)
  :nodeHandle_(nh), context_(1)	//zmq constructor dealing with the initialisation and termination of a zmq context
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
  if(req_flag_) {
    req_socket_.close();
    delete req_recv_;
  }
  if(rep_flag_) {
    rep_socket_.close();
    delete rep_recv_;
  }
  if(rad_flag_) {
    rad_socket_.close();
  }
  if(dsh_flag_) {
    dsh_socket_.close();
    delete dsh_recv_;
  }

  delete img_recv_;

  context_.close();
}

void ZMQ_CLASS::init()
{
  controlDone_ = false;

  /* Initialize zmq data */
  if (rep_flag_) rep_recv_ = new ZmqData;
  if (req_flag_) req_recv_ = new ZmqData;
  if (dsh_flag_) dsh_recv_ = new ZmqData;
  img_recv_ = new ImgData;

  /* Initialize Tcp client(Request) Socket */
  if(req_flag_)
  {
    req_socket_ = zmq::socket_t(context_, ZMQ_REQ); 
    req_socket_.connect(tcpreq_ip_);
    req_socket_.setsockopt(ZMQ_RCVTIMEO, -1);  //timeout (infinite) 
    req_socket_.setsockopt(ZMQ_LINGER, 0); 
  }

  /* Initialize Tcp server(Reply) Socket */
  if(rep_flag_)
  {
    rep_socket_ = zmq::socket_t(context_, ZMQ_REP);
    rep_socket_.bind(tcprep_ip_);
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

  if(req_img_flag_)
  {
    req_img_socket_ = zmq::socket_t(context_, ZMQ_REQ);
    req_img_socket_.connect(tcpreq_img_ip_);
    req_img_socket_.setsockopt(ZMQ_LINGER, 0); 
  }

  if(rep_img_flag_)
  {
    rep_img_socket_ = zmq::socket_t(context_, ZMQ_REP);
    rep_img_socket_.bind(tcprep_img_ip_);
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
  std::string tcp_ip_server, tcp_ip_client, tcp_img_ip_server, tcp_img_ip_client, tcpreq_port, tcprep_port, tcpreq_img_port, tcprep_img_port;
  std::string udp_ip, udp_port;
  nodeHandle_.param("tcp_ip/interface_name",interface_name_,std::string("ens33"));

  nodeHandle_.param("tcp_ip/ip_addr_server",tcp_ip_server,std::string("tcp://*"));
  nodeHandle_.param("tcp_ip/ip_addr_client",tcp_ip_client,std::string("tcp://192.168.0.18"));
  nodeHandle_.param("tcp_ip/req_port",tcpreq_port,std::string("4444"));
  nodeHandle_.param("tcp_ip/rep_port",tcprep_port,std::string("7777"));
  nodeHandle_.param("tcp_ip/zipcode",zipcode_,std::string("00000"));

  nodeHandle_.param("tcpimg_ip/ip_addr_server",tcp_img_ip_server,std::string("tcp://*"));
  nodeHandle_.param("tcpimg_ip/ip_addr_client",tcp_img_ip_client,std::string("tcp://192.168.0.11"));
  nodeHandle_.param("tcpimg_ip/req_port",tcpreq_img_port,std::string("4040"));
  nodeHandle_.param("tcpimg_ip/rep_port",tcprep_img_port,std::string(""));

  nodeHandle_.param("udp_ip/ip_addr",udp_ip,std::string("udp://239.255.255.250"));
  nodeHandle_.param("udp_ip/port",udp_port,std::string("9090"));
  nodeHandle_.param("udp_ip/send_group",rad_group_,std::string("FV"));
  nodeHandle_.param("udp_ip/recv_group",dsh_group_,std::string("LV"));
  
  nodeHandle_.param("socket/req_flag",req_flag_,false);
  nodeHandle_.param("socket/rep_flag",rep_flag_,false);
  nodeHandle_.param("socket/rad_flag",rad_flag_,false);
  nodeHandle_.param("socket/dsh_flag",dsh_flag_,false);
  nodeHandle_.param("socket/req_img_flag",req_img_flag_,false);
  nodeHandle_.param("socket/rep_img_flag",rep_img_flag_,false);

  //set request socket ip
  tcpreq_ip_ = tcp_ip_client;
  tcpreq_ip_.append(":");
  tcpreq_ip_.append(tcpreq_port);

  //set reply socket ip
  tcprep_ip_ = tcp_ip_server;
  tcprep_ip_.append(":");
  tcprep_ip_.append(tcprep_port);

  //set request socket ip
  tcpreq_img_ip_ = tcp_img_ip_client;
  tcpreq_img_ip_.append(":");
  tcpreq_img_ip_.append(tcpreq_img_port);

  //set reply socket ip
  tcprep_img_ip_ = tcp_img_ip_server;
  tcprep_img_ip_.append(":");
  tcprep_img_ip_.append(tcprep_img_port);

  //set radio/dish socket ip
  udp_ip_ = udp_ip;
  udp_ip_.append(":");
  udp_ip_.append(udp_port);

  return true;
}

void* ZMQ_CLASS::requestZMQ(ZmqData *send_data)  // client: send -> recv
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
//    req_recv_ = static_cast<ZmqData *>(recv_msg.data()); 
  }
}

void* ZMQ_CLASS::replyZMQ(ZmqData *send_data)  //server: recv -> send
{
  if(rep_socket_.connected() && !controlDone_)
  {
    zmq::message_t recv_msg(DATASIZE), send_msg(DATASIZE);

    //recv
    rep_socket_.recv(&recv_msg, 0);
    memcpy(rep_recv_, recv_msg.data(), DATASIZE);
//    rep_recv_ = static_cast<ZmqData *>(recv_msg.data());

    //send
    memcpy(send_msg.data(), send_data, DATASIZE);
    rep_socket_.send(send_msg);
    
  }

}

void* ZMQ_CLASS::radioZMQ(ZmqData *send_data)
{
  if(rad_socket_.connected() && !controlDone_)
  {
    zmq::message_t send_msg(DATASIZE);
    send_msg.set_group(rad_group_.c_str());
 
    //pub
    memcpy(send_msg.data(), send_data, DATASIZE);
    rad_socket_.send(send_msg, 0);
  }
}

void* ZMQ_CLASS::dishZMQ()
{
  if(dsh_socket_.connected() && !controlDone_)
  {
    zmq::message_t recv_msg(DATASIZE);

    //sub
    dsh_socket_.recv(&recv_msg, 0);
    memcpy(dsh_recv_, recv_msg.data(), DATASIZE);
//    dsh_recv_ = static_cast<ZmqData *>(sub_msg.data());
  }
}

//void* ZMQ_CLASS::requestImageZMQ(ImgData *send_data)
//{
//  if(req_img_socket_.connected() && !controlDone_)
//  {
//    size_t data_size = sizeof(ImgData);
//    zmq::message_t recv_msg, send_msg(data_size);
//
//    //send
//    memcpy(send_msg.data(), send_data, data_size);
//    req_img_socket_.send(send_msg);
//
//    //recv
//    req_img_socket_.recv(&recv_msg, 0); //trash
//  }
//}

void* ZMQ_CLASS::requestImageZMQ(ImgData *send_data, ImgData *backup_data)
{
  if(req_img_socket_.connected() && !controlDone_) {
      size_t data_size = sizeof(ImgData);
      zmq::message_t recv_msg, send_msg(data_size);

      //send
      memcpy(send_msg.data(), send_data, data_size);
      req_img_socket_.send(send_msg);

      bool expect_reply = true;
      while (expect_reply) {
          //  Poll socket for a reply, with timeout
          zmq::pollitem_t items[] = {
              { req_img_socket_, 0, ZMQ_POLLIN, 0 } };
          zmq::poll (&items[0], 1, REQUEST_TIMEOUT);

          if (items[0].revents & ZMQ_POLLIN) {
	    //recv
            req_img_socket_.recv(&recv_msg, 0); //trash
            expect_reply = false;
          }
          else {
	    img_socket_change_count_++;
            //  Old socket will be confused; close it and open a new one
	    req_img_socket_.close();
            req_img_socket_ = zmq::socket_t(context_, ZMQ_REQ);
            req_img_socket_.connect(tcpreq_img_ip_);
            req_img_socket_.setsockopt(ZMQ_LINGER, 0); 

            //  Send request again, on new socket
	    zmq::message_t resend_msg(data_size);
            memcpy(resend_msg.data(), backup_data, data_size);
            req_img_socket_.send(resend_msg);
          }
      }
  }
}
void* ZMQ_CLASS::replyImageZMQ() //if camera & lidar sensor dual failure
{
  if(rep_img_socket_.connected() && !controlDone_)
  {
    size_t data_size = sizeof(ImgData);
    zmq::message_t recv_msg(data_size), send_msg;

    //recv
    rep_img_socket_.recv(&recv_msg, 0);
    memcpy(img_recv_, recv_msg.data(), data_size);

    //send
    rep_img_socket_.send(send_msg); //trash
  }
}
