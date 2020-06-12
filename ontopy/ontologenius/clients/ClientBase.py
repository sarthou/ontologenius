import rospy

from ontologenius.srv import OntologeniusService

class ClientBase:
    _verbose = False
    _cpt = 0

    def __init__(self, name):
        self._name = name
        self._client = rospy.ServiceProxy('ontologenius/' + self._name, OntologeniusService, True)

    def nb(self):
        return ClientBase._cpt

    def resetNb(self):
        ClientBase._cpt = 0

    def setVerbose(self, verbose):
        ClientBase._verbose = verbose

    def call(self, action, param):
        ClientBase._cpt += 1
        try:
            response = self._client(action, param)
            return response.values
        except rospy.ServiceException as e:
            if ClientBase._verbose == True:
                print  "Failure to call ontologenius/" + self._name
            self._client = rospy.ServiceProxy('ontologenius/' + self._name, OntologeniusService, True)
            try:
                response = self._client(action, param)
                if ClientBase._verbose == True:
                    print "Restored ontologenius/" + self._name
                return response.values
            except rospy.ServiceException as e:
                if ClientBase._verbose == True:
                    print  "Failure of service restoration"
                return None

    def callStr(self, action, param):
        ClientBase._cpt += 1
        try:
            response = self._client(action, param)
            if len(response.values) > 0:
                return response.values[0]
            else:
                return ''
        except rospy.ServiceException as e:
            if ClientBase._verbose == True:
                print  "Failure to call ontologenius/" + self._name
            self._client = rospy.ServiceProxy('ontologenius/' + self._name, OntologeniusService, True)
            try:
                response = self._client(action, param)
                if ClientBase._verbose == True:
                    print "Restored ontologenius/" + self._name
                if len(response.values) > 0:
                    return response.values[0]
                else:
                    return ''
            except rospy.ServiceException as e:
                if ClientBase._verbose == True:
                    print  "Failure of service restoration"
                return None

    def callNR(self, action, param):
        ClientBase._cpt += 1
        try:
            response = self._client(action, param)
            return True
        except rospy.ServiceException as e:
            if ClientBase._verbose == True:
                print  "Failure to call ontologenius/" + self._name
            self._client = rospy.ServiceProxy('ontologenius/' + self._name, OntologeniusService, True)
            try:
                response = self._client(action, param)
                if ClientBase._verbose == True:
                    print "Restored ontologenius/" + self._name
                return True
            except rospy.ServiceException as e:
                if ClientBase._verbose == True:
                    print  "Failure of service restoration"
                return False

    def callBool(self, action, param):
        ClientBase._cpt += 1
        try:
            response = self._client(action, param)
            return response.code == 0
        except rospy.ServiceException as e:
            if ClientBase._verbose == True:
                print  "Failure to call ontologenius/" + self._name
            self._client = rospy.ServiceProxy('ontologenius/' + self._name, OntologeniusService, True)
            try:
                response = self._client(action, param)
                if ClientBase._verbose == True:
                    print "Restored ontologenius/" + self._name
                return response.code == 0
            except rospy.ServiceException as e:
                if ClientBase._verbose == True:
                    print  "Failure of service restoration"
                return False
