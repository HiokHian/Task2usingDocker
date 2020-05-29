/* 
ROS Subscriber and Client service using Arduino.

This is an OOP implementation of a ROS publisher and server service with efforts made to align with 
the S.O.L.I.D principles such as Single Responsibility Principle and Object Encapsulation. The
Subscriber and the Client were put into separate classes to adhere to Single Responsibility Principle.
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

using std_srvs::SetBool;

// Create a handle to this process' node
ros::NodeHandle nh;

/*
ArdSubscriber creates a ROS Subscriber that will subscribe to the topic 'toggle_led' and if the value
that is published to 'toggle_led' is true, it will toggle the BUILTIN_LED on the Arduino board
 */
class ArdSubscriber
{
private:
// The ROS Subscriber is a private member as it does not need to be accessed directly by the user
  ros::Subscriber<std_msgs::Bool, ArdSubscriber> sub;
  
public:
// Constructor for ArdSubscriber class using initialization list
  ArdSubscriber()
  : sub("toggle_led", &ArdSubscriber::callback, this)
  {}
  
  // Initialises ArdSubscriber
  void sub_init(ros::NodeHandle& nh)
  {
    pinMode(LED_BUILTIN, OUTPUT);
    nh.subscribe(sub);
  }

  //Callback function that will be called by the ROS Subscriber when the publisher
  //publishes to the topic 'toggle_led'
  void callback(const std_msgs::Bool& toggle_msg)
  {
    if (toggle_msg.data) {
    // Toggle the LED if the message is true, otherwise do nothing
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    nh.loginfo("Subscriber has received toggle_msg");
    
    if (digitalRead(LED_BUILTIN) == HIGH) {
      nh.loginfo("Thruster is ON");
    }
    else {
      nh.loginfo("Thruster is OFF");
    }
    
    }
  }   
};


/*
ArdClient creates a ROS Client service that will send requests to the service 'receive_request'.
The client will send the Bool values true to the ROS server service written in Python at t = 0 seconds,
t = 3 seconds and t = 6 seconds. The purpose is to get the BUILTIN_LED on the Arduino board to toggle 
ON and OFF at those timings.
 */
class ArdClient
{
private:
// The following varibales are private member as they do not need to be directly accessed by the user
  short margin = 10;
  short current_time;
  short time_difference; 
  short initial_time;
  short first_request_time;
  short latency_time;
  SetBool::Request req;
  SetBool::Response res;
  int count;
  
  ros::ServiceClient<SetBool::Request, SetBool::Response> client;
    
public:
// Constructor for the ArdClient
  ArdClient()
  : client("receive_request")
  {}
  
  // Initialises the ROS Client as well as the initial_time
  void client_init(ros::NodeHandle& nh)
  {
    nh.serviceClient(client);
    count = 0;
    initial_time = millis();
  }

  // Sends the requests for toggling the LED at the timings as stated in the description of the class
  void send_request()
  {
    // Get the latency time between initialising the client and waiting for the node to connect
    // The 'count' variable is used like a flag so that we only get the latency time once at the start
    if (count == 0){
      current_time = millis();
      latency_time = current_time - initial_time;
      nh.loginfo("Latency time obtained");
      count = count + 1;
    }

    // This is where the requests are sent at t = 0 seconds, 3 seconds and 6 seconds. Since we only
    // want the requests to start sending after the node has connected, we have to take the starting
    // of zero seconds to be equal to latency_time after the time at which the client node was 
    // initialised
    current_time = millis();
    time_difference = current_time - initial_time;

    //the 50 milliseconds is added because it seems like it takes around that amount time for the 
    //variables current_time and time_difference to be evaluated, so this is additional latency 
    //due to the running of the two lines of code above
    if ((latency_time + 50 - margin <= time_difference) && (time_difference <= latency_time + 50 + margin)){
      nh.loginfo("1st turn on the Thrusters");
      req.data = true;
      client.call(req, res);
    }
    else if ((latency_time + 3000 - margin <= time_difference) && (time_difference <= latency_time + 3000 + margin)){
      nh.loginfo("2nd turn off the Thrusters");
      req.data = true;
      client.call(req, res);
    }
    else if ((latency_time + 6000 - margin <= time_difference) && (time_difference <= latency_time + 6000 + margin)){
      nh.loginfo("3rd turn on the Thrusters");
      req.data = true;
      client.call(req, res);
    }
  } 
};

// Instantiate the subscriber instance and the client instance
ArdSubscriber subscriberinstance1;
ArdClient clientinstance1;

void setup()
{ 
  //Initialise the ROS node
  nh.initNode();
  subscriberinstance1.sub_init(nh);
  clientinstance1.client_init(nh);
  // Wait for the ROS node to be connected
  while(!nh.connected()) nh.spinOnce();
  nh.loginfo("Node is Ready");
}



void loop()
{  
  // Call the send request function during each iteration of the loop and if the timing matches
  // the conditions of t = 0 seconds, 3 seconds, 6 seconds, the request will be sent
  clientinstance1.send_request();
  nh.spinOnce();
}
