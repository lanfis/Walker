#pragma once
#ifndef _ROS_LINK_H_
#define _ROS_LINK_H_

#include <string>
#include <cstring>
#include <sstream>
#include <ros/ros.h>
#include <vector>
#include <chrono>
#include <std_msgs/String.h>

#include "config/console_format.h"
#include "config/connect_port_config.h"

using namespace std;

//#define delayTime 2
#define TRANSFER_TYPE_PUBLISHER "Publisher"
#define TRANSFER_TYPE_SUBSCRIBER "Subscriber"
#define PING "PING"
#define ACK "ACK"
#define STATUS "STATUS"
#define STATUS_SIZE 8
#define STATUS_BEGIN "Begin"
#define STATUS_NODENAME "NodeName"
#define STATUS_TOPIC "Topic"
#define STATUS_TRANSFER_TYPE "Transfer_type"
#define STATUS_CONNECT_PORT  "Connect_port"
#define STATUS_END "End"
#define REQUEST_TOPIC_CHANGE "REQUEST_TOPIC_CHANGE"
#define REQUEST_TOPIC_CHANGE_ACK "REQUEST_TOPIC_CHANGE_ACK"
#define DEFAULT_CONNECT_PORT 13

class cell_publisher
{
    public:
        string topic;
        ros::Publisher* pub = NULL;
        string transfer_type = TRANSFER_TYPE_PUBLISHER;
        int connect_port = DEFAULT_CONNECT_PORT;
        cell_publisher(){};
        ~cell_publisher(){/* if(pub != NULL) pub -> shutdown(); */};    
};

class cell_subscriber
{
    public:
        string topic;
        ros::Subscriber* sub = NULL;
        string transfer_type = TRANSFER_TYPE_SUBSCRIBER;
        int connect_port = DEFAULT_CONNECT_PORT;
        cell_subscriber(){};
        ~cell_subscriber(){/* if(sub != NULL) sub -> shutdown(); */};
};


class ROS_Link
{    
  private:
    float ver_ = 1.2;//1.2 ready for overloading but consider for the efficiency, so ...
    ros::NodeHandle n_;
    string nodeName_;
    boost::shared_ptr< std_msgs::String > msg_pub_;
    boost::shared_ptr< std_msgs::String > msg_sub_;
    ros::Publisher *pub_ = NULL;
    ros::Subscriber *sub_ = NULL;
    ros::SubscriberStatusCallback connect_cb;
    ros::SubscriberStatusCallback disconnect_cb;
    void publish(string data);
    void callBack(const std_msgs::String::ConstPtr& msg);
    void connectCb(const ros::SingleSubscriberPublisher& ssp);
    void disconnectCb(const ros::SingleSubscriberPublisher&);
    
  public:
    int queue_size_pub = 4;
    int queue_size_sub = 4;
    string topic_link_pub_ = "/link";
    string topic_link_sub_ = "/status";

  private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double elapsed;
    bool flag_pub = false;
    bool flag_sub = false;
    bool flag_link = false;
    
  private:
    vector< cell_publisher > control_pub_;
    vector< cell_subscriber > control_sub_;
    vector< cell_publisher > link_pub_;
    vector< cell_subscriber > link_sub_;
    
    void callBack_search_for_publisher(const std_msgs::String::ConstPtr& msg);

  private: 
    /*
    bool add_cell(vector< cell_publisher >& cell, string& topic, ros::Publisher* pub=NULL, int connect_port=DEFAULT_CONNECT_PORT);
    bool add_cell(vector< cell_subscriber >& cell, string& topic, ros::Subscriber* sub=NULL, int connect_port=DEFAULT_CONNECT_PORT);
    */
    bool add_cell(vector< cell_publisher >& cell, string& topic, ros::Publisher* pub, int& connect_port);
    bool add_cell(vector< cell_subscriber >& cell, string& topic, ros::Subscriber* sub, int& connect_port);
    bool add_cell(vector< cell_publisher >& cell, string& topic, ros::Publisher* pub);
    bool add_cell(vector< cell_subscriber >& cell, string& topic, ros::Subscriber* sub);
    bool add_cell(vector< cell_publisher >& cell, string& topic, int& connect_port);
    bool add_cell(vector< cell_subscriber >& cell, string& topic, int& connect_port);
    bool add_cell(vector< cell_publisher >& cell, string& topic);
    bool add_cell(vector< cell_subscriber >& cell, string& topic);
    bool search_cell_topic(vector< cell_publisher >& cell, string& topic);
    bool search_cell_topic(vector< cell_subscriber >& cell, string& topic);
    bool search_cell_topic(vector< cell_publisher >& cell, string& topic, int& idx);
    bool search_cell_topic(vector< cell_subscriber >& cell, string& topic, int& idx);
    bool erase_cell(vector< cell_publisher >& cell, string& topic);
    bool erase_cell(vector< cell_subscriber >& cell, string& topic);
    bool flag_ping = false;
    bool ping_get();
    bool ping_ack();
    bool flag_status = false;
    bool flag_status_lock = false;
    int status_size = STATUS_SIZE;
    bool status_get();
    bool status_ack();
    bool flag_request_topic_change = false;
    bool request_topic_change_get();
    bool request_topic_change_ack();
    
  public:
    ROS_Link *next = this;
    bool search_for_publisher(string& topic);
    bool is_pub() { return flag_pub; };
    bool is_sub() { return flag_sub; };
    bool is_link() { return flag_link; };
    int get_link_pub_size() { return link_pub_.size(); };
    int get_link_sub_size() { return link_sub_.size(); };
    string get_link_pub_topic(int idx) { if(idx < link_pub_.size()) return link_pub_[idx].topic; else return ""; };
    string get_link_sub_topic(int idx) { if(idx < link_sub_.size()) return link_sub_[idx].topic; else return ""; };
    int get_link_pub_connect_port(int idx) { if(idx < link_pub_.size()) return link_pub_[idx].connect_port; else return -1; };
    int get_link_sub_connect_port(int idx) { if(idx < link_sub_.size()) return link_sub_[idx].connect_port; else return -1; };
    bool add_cell(ros::Publisher& cell, string& topic, int& connect_port);
    bool add_cell(ros::Subscriber& cell, string& topic, int& connect_port);
    bool add_cell(ros::Publisher& cell, string& topic);
    bool add_cell(ros::Subscriber& cell, string& topic);
    bool add_cell(ros::Publisher& cell, string& topic, Connect_Port connect_type);
    bool add_cell(ros::Subscriber& cell, string& topic, Connect_Port connect_type);
    bool add_cell(ros::Publisher* cell, string& topic, int& connect_port);
    bool add_cell(ros::Subscriber* cell, string& topic, int& connect_port);
    bool add_cell(ros::Publisher* cell, string& topic);
    bool add_cell(ros::Subscriber* cell, string& topic);
    bool add_cell(ros::Publisher* cell, string& topic, Connect_Port connect_type);
    bool add_cell(ros::Subscriber* cell, string& topic, Connect_Port connect_type);
    bool erase_cell(string& topic);
    bool ping();
    bool status();
    bool request_topic_change(string& topic, string& new_topic);

    
  public:
    ROS_Link(ros::NodeHandle& n, string nodeName);
    ~ROS_Link();
    void run();
    void pub_init();
    void pub_shutdown();
    void sub_init();
    void sub_shutdown();
};

ROS_Link::ROS_Link(ros::NodeHandle& n, string nodeName) : n_(n), nodeName_(nodeName), msg_pub_(new std_msgs::String), msg_sub_(new std_msgs::String)
{
    topic_link_pub_ = nodeName_ + topic_link_pub_;
    topic_link_sub_ = nodeName_ + topic_link_sub_;
    
    connect_cb    = boost::bind(&ROS_Link::connectCb, this, _1);
    disconnect_cb = boost::bind(&ROS_Link::disconnectCb, this, _1);
//  OUT_INFO(nodeName_.c_str(), "ROS_Link is used for fast and automatically topics remapping !");
}

ROS_Link::~ROS_Link()
{
    pub_shutdown();
    sub_shutdown();
    if(next != this && next != NULL)
        delete next;
}

void ROS_Link::run()
{
    now = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
    if(ping_get()) return;
    if(ping_ack()) return;
    if(status_get()) return;
    if(status_ack()) return;
    if(request_topic_change_get()) return;
    if(request_topic_change_ack()) return;
}

bool ROS_Link::request_topic_change(string& topic, string& new_topic)
{
    OUT_INFO(nodeName_.c_str(), "request for topic changing ...");
    flag_request_topic_change = true;
    string header = REQUEST_TOPIC_CHANGE;
    string msg = header + "\t" + topic + "\t" + new_topic;
    publish(msg);
    start = std::chrono::high_resolution_clock::now();
    return true;
}

bool ROS_Link::request_topic_change_get()
{
    stringstream token(msg_sub_ -> data);
    string cmd;
    token >> cmd;
    if(cmd == REQUEST_TOPIC_CHANGE)
    {
        string topic;
        string new_topic;
        token >> topic >> new_topic;
        int idx;
        if(search_cell_topic(this -> control_pub_, topic, idx))
            control_pub_[idx].topic = new_topic;
        else if(search_cell_topic(this -> control_sub_, topic, idx))
            control_sub_[idx].topic = new_topic;
        else if(this -> topic_link_pub_ == topic)
        {
            this -> topic_link_pub_ = new_topic;
            publish(REQUEST_TOPIC_CHANGE_ACK);
            pub_init();
        }
        else if(this -> topic_link_sub_ == topic)
        {
            this -> topic_link_sub_ = new_topic;
            publish(REQUEST_TOPIC_CHANGE_ACK);
            sub_init();
        }
        else
            return false;
        publish(REQUEST_TOPIC_CHANGE_ACK);
        return true;
    }
    return false;
}

bool ROS_Link::request_topic_change_ack()
{
    if(flag_request_topic_change)
    {
        if(msg_sub_ -> data != REQUEST_TOPIC_CHANGE_ACK) return false;
        OUT_INFO(nodeName_.c_str(), "request topic changing ack received !");
        flag_request_topic_change = false;
        return true;
    }
    return false;
}

bool ROS_Link::status()
{
    OUT_INFO(nodeName_.c_str(), "requesting for status ...");
    flag_status = true;
    publish(STATUS);
    start = std::chrono::high_resolution_clock::now();
    return true;
}

bool ROS_Link::status_get()
{
    if(msg_sub_ -> data == STATUS)
    {
        string nodeName = STATUS_NODENAME;
        string topic = STATUS_TOPIC;
        string transfer_type = STATUS_TRANSFER_TYPE;
        string connect_port = STATUS_CONNECT_PORT;
        
        publish(STATUS_BEGIN);
        stringstream token;
        string token_str;
//      publish(nodeName + "\t" + nodeName_);
        for(int i = 0; i < control_pub_.size(); i++)
        {
            token.clear();
            token << control_pub_[i].connect_port;
            token >> token_str;
            publish(topic + "\t" + control_pub_[i].topic + "\t" + 
                    transfer_type + "\t" + control_pub_[i].transfer_type + "\t" + 
                    connect_port + "\t" + token_str);
        }
        for(int i = 0; i < control_sub_.size(); i++)
        {
            token.clear();
            token << control_sub_[i].connect_port;
            token >> token_str;
            publish(topic + "\t" + control_sub_[i].topic + "\t" +           
                    transfer_type + "\t" + control_sub_[i].transfer_type + "\t" +
                    connect_port + "\t" + token_str);
        }
        publish(STATUS_END);
        return true;
    }
    return false;
}

bool ROS_Link::status_ack()
{
    if(flag_status)
    {
        if(msg_sub_ -> data == STATUS_BEGIN)
        {
            flag_status_lock = true;
            status_size = STATUS_SIZE;
            OUT_INFO(nodeName_.c_str(), "status message receiving ...");
            return true;
        }
        else if(msg_sub_ -> data == STATUS_END)
        {
            flag_status_lock = false;
            flag_status = false;
            OUT_INFO(nodeName_.c_str(), "status message received !");
            status_size -= 1;
            return true;
        }   
        else if(status_size < 1)
        {
            flag_status_lock = false;
            flag_status = false;
            OUT_WARN(nodeName_.c_str(), "status message size out of size !");
            OUT_WARN(nodeName_.c_str(), "status message maynot be received completely !");
            return true;
        }    
        if(flag_status_lock)
        {
            stringstream token(msg_sub_ -> data);
            string str;
            string transfer_type;
            string topic;
            int connect_port;
            while(token >> str)
            {
//              if(str == STATUS_NODENAME)
//              { token >> str; }
                if(str == STATUS_TOPIC)
                {
                    token >> topic;
                }
                if(str == STATUS_TRANSFER_TYPE)
                {
                    token >> transfer_type;
                }
                if(str == STATUS_CONNECT_PORT)
                {
                    token >> connect_port;
                }
            }
            int idx;
            if(transfer_type == TRANSFER_TYPE_PUBLISHER)
            {
                if(search_cell_topic(link_pub_, topic, idx))
                {
                    link_pub_[idx].connect_port = connect_port;
                }
                else
                {
                    add_cell(link_pub_, topic, NULL, connect_port);
                }
                
            }
            else if(transfer_type == TRANSFER_TYPE_SUBSCRIBER)
            {
                if(search_cell_topic(link_sub_, topic, idx))
                {
                    link_sub_[idx].connect_port = connect_port;
                }
                else
                {
                    add_cell(link_sub_, topic, NULL, connect_port);
                }
            }
            else 
            {
                OUT_WARN(nodeName_.c_str(), "status message error format !");
            }
            status_size -= 1;
            return true;
        }
        
    }
    return false;
}

bool ROS_Link::ping()
{
    OUT_INFO(nodeName_.c_str(), "pinging ...");
    flag_ping = true;
    flag_link = false;
    string msg = PING;
    msg = msg + " " + topic_link_sub_;
    publish(msg);
    start = std::chrono::high_resolution_clock::now();
    return true;
}

bool ROS_Link::ping_get()
{
    stringstream token(msg_sub_ -> data);
    string msg;
    string topic;
    token >> msg;
    if(msg == PING)
    {
        token >> topic;
        this -> topic_link_pub_ = topic;
        pub_init();
        publish(ACK);
        flag_link = true;
        return true;
    }
    return false;
}

bool ROS_Link::ping_ack()
{
    if(flag_ping)
    {
        if(msg_sub_ -> data != ACK) return false;
        OUT_INFO(nodeName_.c_str(), "ack received !");
        flag_link = true;
        flag_ping = false;
        return true;
    }
    return false;
}

void ROS_Link::callBack_search_for_publisher(const std_msgs::String::ConstPtr& msg){};
bool ROS_Link::search_for_publisher(string& topic)
{
    ros::Subscriber sub_temp;
    sub_temp = n_.subscribe(topic.c_str(), 1, &ROS_Link::callBack_search_for_publisher, this);
    bool is_publisher = (sub_temp.getNumPublishers() > 0)? true : false;
    sub_temp.shutdown();
    return is_publisher;
}

/*
bool ROS_Link::add_cell(vector< cell_publisher >& cell, string& topic, ros::Publisher* pub, int connect_port)
{
    if(search_cell_topic(cell, topic))
        return false;
    cell_publisher p;
    p.pub = pub;
    p.topic = topic;
//    p.transfer_type = TRANSFER_TYPE_PUBLISHER;
    p.connect_port = connect_port;
    cell.emplace_back(p);
    return true;
}

bool ROS_Link::add_cell(vector< cell_subscriber >& cell, string& topic, ros::Subscriber* sub, int connect_port)
{
    if(search_cell_topic(cell, topic))
        return false;
    cell_subscriber s;
    s.sub = sub;
    s.topic = topic;
//    s.transfer_type = TRANSFER_TYPE_SUBSCRIBER;
    s.connect_port = connect_port;
    cell.emplace_back(s);
    return true;
}
*/
bool ROS_Link::add_cell(ros::Publisher& cell, string& topic, int& connect_port)
{
    if(search_cell_topic(control_pub_, topic))
        return false;
    add_cell(this -> control_pub_, topic, &cell, connect_port);
    return true;
}

bool ROS_Link::add_cell(ros::Subscriber& cell, string& topic, int& connect_port)
{
    if(search_cell_topic(control_sub_, topic))
        return false;
    add_cell(this -> control_sub_, topic, &cell, connect_port);
    return true;
}

bool ROS_Link::add_cell(ros::Publisher& cell, string& topic)
{
    if(search_cell_topic(control_pub_, topic))
        return false;
    add_cell(this -> control_pub_, topic, &cell);
    return true;
}

bool ROS_Link::add_cell(ros::Subscriber& cell, string& topic)
{
    if(search_cell_topic(control_sub_, topic))
        return false;
    add_cell(this -> control_sub_, topic, &cell);
    return true;
}

bool ROS_Link::add_cell(ros::Publisher& cell, string& topic, Connect_Port connect_type)
{
    if(search_cell_topic(control_pub_, topic))
        return false;
    int port = static_cast<int>(connect_type);
    add_cell(this -> control_pub_, topic, &cell, port);
    return true;
}

bool ROS_Link::add_cell(ros::Subscriber& cell, string& topic, Connect_Port connect_type)
{
    if(search_cell_topic(control_sub_, topic))
        return false;
    int port = static_cast<int>(connect_type);
    add_cell(this -> control_sub_, topic, &cell, port);
    return true;
}

bool ROS_Link::add_cell(ros::Publisher* cell, string& topic, int& connect_port)
{
    if(search_cell_topic(control_pub_, topic))
        return false;
    add_cell(this -> control_pub_, topic, cell, connect_port);
    return true;
}

bool ROS_Link::add_cell(ros::Subscriber* cell, string& topic, int& connect_port)
{
    if(search_cell_topic(control_sub_, topic))
        return false;
    add_cell(this -> control_sub_, topic, cell, connect_port);
    return true;
}

bool ROS_Link::add_cell(ros::Publisher* cell, string& topic)
{
    if(search_cell_topic(control_pub_, topic))
        return false;
    add_cell(this -> control_pub_, topic, cell);
    return true;
}

bool ROS_Link::add_cell(ros::Subscriber* cell, string& topic)
{
    if(search_cell_topic(control_sub_, topic))
        return false;
    add_cell(this -> control_sub_, topic, cell);
    return true;
}

bool ROS_Link::add_cell(ros::Publisher* cell, string& topic, Connect_Port connect_type)
{
    if(search_cell_topic(control_pub_, topic))
        return false;
    int port = static_cast<int>(connect_type);
    add_cell(this -> control_pub_, topic, cell, port);
    return true;
}

bool ROS_Link::add_cell(ros::Subscriber* cell, string& topic, Connect_Port connect_type)
{
    if(search_cell_topic(control_sub_, topic))
        return false;
    int port = static_cast<int>(connect_type);
    add_cell(this -> control_sub_, topic, cell, port);
    return true;
}

bool ROS_Link::erase_cell(string& topic)
{
    if(erase_cell(control_pub_, topic))
        return true;
    else
        return erase_cell(control_sub_, topic);
}

bool ROS_Link::add_cell(vector< cell_publisher >& cell, string& topic, ros::Publisher* pub, int& connect_port)
{
    if(search_cell_topic(cell, topic))
        return false;
    cell_publisher p;
    p.pub = pub;
    p.topic = topic;
//    p.transfer_type = TRANSFER_TYPE_PUBLISHER;
    p.connect_port = connect_port;
    cell.emplace_back(p);
    return true;
}

bool ROS_Link::add_cell(vector< cell_subscriber >& cell, string& topic, ros::Subscriber* sub, int& connect_port)
{
    if(search_cell_topic(cell, topic))
        return false;
    cell_subscriber s;
    s.sub = sub;
    s.topic = topic;
//    s.transfer_type = TRANSFER_TYPE_SUBSCRIBER;
    s.connect_port = connect_port;
    cell.emplace_back(s);
    return true;
}

bool ROS_Link::add_cell(vector< cell_publisher >& cell, string& topic, ros::Publisher* pub)
{
    if(search_cell_topic(cell, topic))
        return false;
    cell_publisher p;
    p.pub = pub;
    p.topic = topic;
//    p.transfer_type = TRANSFER_TYPE_PUBLISHER;
    cell.emplace_back(p);
    return true;
}

bool ROS_Link::add_cell(vector< cell_subscriber >& cell, string& topic, ros::Subscriber* sub)
{
    if(search_cell_topic(cell, topic))
        return false;
    cell_subscriber s;
    s.sub = sub;
    s.topic = topic;
//    s.transfer_type = TRANSFER_TYPE_SUBSCRIBER;
    cell.emplace_back(s);
    return true;
}

bool ROS_Link::add_cell(vector< cell_publisher >& cell, string& topic, int& connect_port)
{
    if(search_cell_topic(cell, topic))
        return false;
    cell_publisher p;
    p.topic = topic;
//    p.transfer_type = TRANSFER_TYPE_PUBLISHER;
    p.connect_port = connect_port;
    cell.emplace_back(p);
    return true;
}

bool ROS_Link::add_cell(vector< cell_subscriber >& cell, string& topic, int& connect_port)
{
    if(search_cell_topic(cell, topic))
        return false;
    cell_subscriber s;
    s.topic = topic;
//    s.transfer_type = TRANSFER_TYPE_SUBSCRIBER;
    s.connect_port =connect_port;
    cell.emplace_back(s);
    return true;
}


bool ROS_Link::add_cell(vector< cell_publisher >& cell, string& topic)
{
    if(search_cell_topic(cell, topic))
        return false;
    cell_publisher p;
    p.topic = topic;
//    p.transfer_type = TRANSFER_TYPE_PUBLISHER;
    cell.emplace_back(p);
    return true;
}

bool ROS_Link::add_cell(vector< cell_subscriber >& cell, string& topic)
{
    if(search_cell_topic(cell, topic))
        return false;
    cell_subscriber s;
    s.topic = topic;
//    s.transfer_type = TRANSFER_TYPE_SUBSCRIBER;
    cell.emplace_back(s);
    return true;
}

bool ROS_Link::search_cell_topic(vector< cell_publisher >& cell, string& topic)
{
    for(int idx = 0; idx < cell.size(); idx++)
    {
        if(topic == cell[idx].topic)
            return true;
    }
    return false;
}

bool ROS_Link::search_cell_topic(vector< cell_subscriber >& cell, string& topic)
{
    for(int idx = 0; idx < cell.size(); idx++)
    {
        if(topic == cell[idx].topic)
            return true;
    }
    return false;
}

bool ROS_Link::search_cell_topic(vector< cell_publisher >& cell, string& topic, int& idx)
{
    for(idx = 0; idx < cell.size(); idx++)
    {
        if(topic == cell[idx].topic)
            return true;
    }
    idx = -1;
    return false;
}

bool ROS_Link::search_cell_topic(vector< cell_subscriber >& cell, string& topic, int& idx)
{
    for(idx = 0; idx < cell.size(); idx++)
    {
        if(topic == cell[idx].topic)
            return true;
    }
    idx = -1;
    return false;
}

bool ROS_Link::erase_cell(vector< cell_publisher >& cell, string& topic)
{
    int idx = 0;
    for(vector<cell_publisher>::iterator i = cell.begin(); i != cell.end(); i++, idx++)
    {
        if(topic == cell[idx].topic)
        {
            cell.erase(i);
            return true;
        }
    }
    return false;
}

bool ROS_Link::erase_cell(vector< cell_subscriber >& cell, string& topic)
{
    int idx = 0;
    for(vector<cell_subscriber>::iterator i = cell.begin(); i != cell.end(); i++, idx++)
    {
        if(topic == cell[idx].topic)
        {
            cell.erase(i);
            return true;
        }
    }
    return false;
}

void ROS_Link::publish(string data)
{
    if(!flag_pub) return;
    msg_pub_ -> data = data;
    pub_ -> publish(msg_pub_);
}

void ROS_Link::callBack(const std_msgs::String::ConstPtr& msg)
{
    this -> msg_sub_ -> data = msg -> data;
    run();
}

void ROS_Link::connectCb(const ros::SingleSubscriberPublisher& ssp)
{
    if(pub_ -> getNumSubscribers() > 1)
    {
        string str = topic_link_pub_ + " : "+ "links establishing !";
        OUT_WARN(nodeName_.c_str(), str);
        return;
    }
    if(!flag_sub)
    {
      string str = topic_link_pub_ + " : "+ "establishing !";
      OUT_INFO(nodeName_.c_str(), str);
      sub_init();
      ping();
    }
}

void ROS_Link::disconnectCb(const ros::SingleSubscriberPublisher&)
{
    if(pub_ -> getNumSubscribers() > 0) return;
    if(flag_sub)
    {
      string str = topic_link_sub_ + " : " + "shutting down !";
      OUT_WARN(nodeName_.c_str(), str);
      sub_shutdown();
      flag_link = false;
    }
}

void ROS_Link::pub_init()
{
    pub_shutdown();
    pub_ = new ros::Publisher;
    *pub_ = n_.advertise< std_msgs::String >(topic_link_pub_.c_str(), queue_size_pub, connect_cb, disconnect_cb, nullptr, true);
    this -> topic_link_pub_ = pub_ -> getTopic();
    flag_pub = true;
}

void ROS_Link::pub_shutdown()
{
    if(pub_ != NULL)
    {
      pub_ -> shutdown();
      delete pub_;
      pub_ = NULL;
    }
    flag_pub = false;
}

void ROS_Link::sub_init()
{
    sub_shutdown();
    sub_ = new ros::Subscriber;
    *sub_ = n_.subscribe(topic_link_sub_.c_str(), queue_size_sub, &ROS_Link::callBack, this);
    this -> topic_link_sub_ = sub_ -> getTopic();
    flag_sub = true;
}

void ROS_Link::sub_shutdown()
{
    if(sub_ != NULL)
    {
      sub_ -> shutdown();
      delete sub_;
      sub_ = NULL;
    }
    flag_sub = false;
}

#endif
