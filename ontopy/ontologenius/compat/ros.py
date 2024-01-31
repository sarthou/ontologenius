import os
import time
from ontologenius.msg import OntologeniusTimestamp

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

        def unregister(self):
            self.sub.unregister()

    class Ontoros :
        def createService(srv_name, srv_type):
            return OntoService(srv_name, srv_type)
        
        def createPublisher(pub_name, pub_type, queue_size):
            return OntoPublisher(pub_name, pub_type, queue_size)
        
        def createSubscriber(sub_name, sub_type, callback):
            return OntoSubscriber(sub_name, sub_type, callback)
        
        def getRosTime():
            t_msg = rospy.get_rostime()
            stamp_msg = OntologeniusTimestamp(seconds = t_msg.secs, nanoseconds = t_msg.nsecs)
            return stamp_msg
        
        def getStamp(time):
            return OntologeniusTimestamp(seconds = time.secs, nanoseconds = time.nsecs)
        
        def isShutdown():
            return rospy.is_shutdown()

        def spin_once():
            time.sleep(0.01)

elif os.environ["ROS_VERSION"] == "2":

    import rclpy
    from rclpy.node import Node
    from threading import Lock
    from rclpy.service import SrvTypeRequest, SrvTypeResponse
    from rclpy.client import Client
    from rclpy.publisher import Publisher
    from rclpy.subscription import Subscription
    from rclpy.time import Time

    class SingletonMeta(type):
        """
        This is a thread-safe implementation of Singleton.
        """

        _instances = {}

        _lock: Lock = Lock()
        """
        We now have a lock object that will be used to synchronize threads during
        first access to the Singleton.
        """

        def __call__(cls, *args, **kwargs):
            """
            Possible changes to the value of the `__init__` argument do not affect
            the returned instance.
            """
            # Now, imagine that the program has just been launched. Since there's no
            # Singleton instance yet, multiple threads can simultaneously pass the
            # previous conditional and reach this point almost at the same time. The
            # first of them will acquire lock and will proceed further, while the
            # rest will wait here.
            with cls._lock:
                # The first thread to acquire the lock, reaches this conditional,
                # goes inside and creates the Singleton instance. Once it leaves the
                # lock block, a thread that might have been waiting for the lock
                # release may then enter this section. But since the Singleton field
                # is already initialized, the thread won't create a new object.
                if cls not in cls._instances:
                    instance = super().__call__(*args, **kwargs)
                    cls._instances[cls] = instance
            return cls._instances[cls]


    class OntoService:
        def __init__(self, client: Client, name, node_: Node):
            self.client: Client = client
            self.node_: Node = node_
            self.srv_name = name

        def call(self, params: SrvTypeRequest, verbose) -> SrvTypeResponse:
            future = self.client.call_async(request=params)
            rclpy.spin_until_future_complete(self.node_, future)
            return future.result()
                
        def wait(self, timeout = -1):
            if(timeout != -1):
                self.client.wait_for_service(timeout)
            else:
                self.client.wait_for_service()


    class OntoPublisher:
        def __init__(self, pub):
            self.pub: Publisher = pub

        def publish(self, msg):
            self.pub.publish(msg)

        def getNumSubscribers(self):
            self.pub.get_subscription_count()


    class OntoSubscriber:
        def __init__(self, sub):
            self.sub: Subscription = sub

        def getNumPublishers(self):
            self.sub.get_num_connections()

        def unregister(self):
            pass


    class Ontoros(Node, metaclass=SingletonMeta):

        def __init__(self):
            super().__init__("OntoRos")

        @staticmethod
        def createService(srv_name, srv_type) -> OntoService:
            return OntoService(Ontoros().create_client(srv_type, srv_name), srv_name, Ontoros())

        @staticmethod
        def createPublisher(pub_name, pub_type, queue_size: int) -> OntoPublisher:
            return OntoPublisher(Ontoros().create_publisher(pub_type, pub_name, qos_profile=queue_size))

        @staticmethod
        def createSubscriber(sub_name, sub_type, callback, queue_size: int = 10) -> OntoSubscriber:
            return OntoSubscriber(Ontoros().create_subscription(sub_type, sub_name, callback, qos_profile=queue_size))

        @staticmethod
        def getRosTime() -> Time:
            t_msg = Ontoros().get_clock().now().to_msg()
            stamp_msg = OntologeniusTimestamp(seconds = t_msg._sec, nanoseconds = t_msg._nanosec)
            return stamp_msg
        
        @staticmethod
        def getStamp(time):
            return OntologeniusTimestamp(seconds = time._sec, nanoseconds = time._nanosec)

        @staticmethod
        def isShutdown() -> bool:
            return not rclpy.ok()

        @staticmethod
        def spin_once():
            rclpy.spin_once(Ontoros(), timeout_sec=0.01)
