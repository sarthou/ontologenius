import os

if os.environ["ROS_VERSION"] == "1":
    import rospy

    class ontoService :
        def __init__(self, client):
            self.client = client

        def call(params):
            pass
    
    class ontoPublisher :
        def __init__(self, pub):
            self.pub = pub

        def publish(self, msg):
            self.pub.publish(msg)
    
    class ontoSubscriber :
        def __init__(self, sub):
            self.sub = sub

    class ontoros :
        def createService(srv_name, srv_type, connected):
            return ontoService(rospy.ServiceProxy(srv_name, srv_type, connected))
        
        def createPublisher(pub_name, pub_type, queue_size):
            return ontoPublisher(rospy.Publisher(pub_name, pub_type, queue_size=queue_size))
        
        def createSubscriber(sub_name, sub_type, callback):
            return ontoSubscriber(rospy.Subscriber(sub_name, sub_type, callback))
        
        def getRosTime():
            return rospy.get_rostime()
        
        def isShutdown():
            return rospy.is_shutdown()

    print("v1")
elif os.environ["ROS_VERSION"] == "2":
    import rclpy 

    print("v2")