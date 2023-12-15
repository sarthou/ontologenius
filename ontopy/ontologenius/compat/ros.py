try:
    import rospy

    # todo: ROS1 support
except ModuleNotFoundError:
    import rclpy
    from rclpy.node import Node
    from ontologenius import OntologeniusStampedString

    node: Node

    def init(name):
        rclpy.init()
        global node
        node = Node(node_name=name)


    def spin():
        global node
        rclpy.spin(node)


    class ROSPublisher:
        def __init__(self, interface_ty: type, name: str):
            global node
            self.handle_ = node.create_publisher(interface_ty, name, 10)


    class ROSSubscriber:
        def __init__(self, interface_ty: type, name: str, callback: callable):
            global node
            self.handle_ = node.create_subscription(interface_ty, name, callback, 10)


    class ROSService:
        def __init__(self, interface_ty: type, name: str, callback: callable):
            global node
            self.handle_ = node.create_service(interface_ty, name, callback)


    class ROSClient:
        def __init__(self, interface_ty: type, name: str):
            global node
            self.handle_ = node.create_client(interface_ty, name)

client = ROSClient()