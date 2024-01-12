import os

if os.environ["ROS_VERSION"] == "1":
    import rospy

    class OntoService :
        def __init__(self, srv_name, srv_type):
            self._client = rospy.ServiceProxy(srv_name, srv_type, True)
            self.srv_name = srv_name
            self.srv_type = srv_type

        def call(self, request, verbose):
            try:
                response = self._client(request)
                return response
            except (rospy.ServiceException, rospy.exceptions.TransportTerminated) as e:
                if verbose == True:
                    print("Failure to call " + self.srv_name)
                self._client = rospy.ServiceProxy(self.srv_name, self.srv_type, True)
                try:
                    response = self._client(request)
                    if verbose == True:
                        print("Restored " + self.srv_name)
                    return response
                except (rospy.ServiceException, rospy.exceptions.TransportTerminated) as e:
                    if verbose == True:
                        print("Failure of service restoration")
                    return None
                
        def wait(self, timeout = -1):
            if(timeout != -1):
                rospy.wait_for_service(self.srv_name, timeout)
            else:
                rospy.wait_for_service(self.srv_name)
    
    class OntoPublisher :
        def __init__(self, pub_name, pub_type, queue_size):
            self.pub = rospy.Publisher(pub_name, pub_type, queue_size=queue_size)

        def publish(self, msg):
            self.pub.publish(msg)

        def getNumSubscribers(self):
            self.pub.get_num_connections()
    
    class OntoSubscriber :
        def __init__(self, sub_name, sub_type, callback):
            self.sub = rospy.Subscriber(sub_name, sub_type, callback)

        def getNumPublishers(self):
            self.sub.get_num_connections()

    class Ontoros :
        def createService(srv_name, srv_type):
            return OntoService(srv_name, srv_type)
        
        def createPublisher(pub_name, pub_type, queue_size):
            return OntoPublisher(pub_name, pub_type, queue_size)
        
        def createSubscriber(sub_name, sub_type, callback):
            return OntoSubscriber(sub_name, sub_type, callback)
        
        def getRosTime():
            return rospy.get_rostime()
        
        def isShutdown():
            return rospy.is_shutdown()

    print("v1")
elif os.environ["ROS_VERSION"] == "2":
    import rclpy 

    print("v2")