#ifndef ONTOLOGENIUS_UTILITY_H
#define ONTOLOGENIUS_UTILITY_H

#include <string>

#include <ros/ros.h>

#include "ontologenius/REST.h"

namespace ontologenius {

inline int send_request(const std::string& method, const std::string& url, const std::string& body, std::string& response)
{
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<ontologenius::REST>("ontologenius/rest");

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
    response = srv.response.text;
    return 0;
  }
}

} // namespace ontologenius

#endif // ONTOLOGENIUS_UTILITY_H
