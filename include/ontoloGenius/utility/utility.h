#include <string>

#include "ontologenius/REST.h"
#include "ros/ros.h"

using namespace std;

int send_request(string method, string url, string body, string* response)
{
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<ontologenius::REST>("ontoloGenius/REST");

  ontologenius::REST srv;
  srv.request.method = method;
  srv.request.URL = url;
  srv.request.body = body;

  if(!client.call(srv))
  {
    ROS_ERROR("REST client call failed");
    return -1;
  }
  else
  {
    (*response) = srv.response.text;
    return 0;
  }
}
