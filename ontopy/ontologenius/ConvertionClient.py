import rospy

from ontologenius.srv import OntologeniusConvertion
from ontologenius.srv import OntologeniusConvertionRequest

class ConvertionClient:
    

    def __init__(self, name):
        """Constructs a ROS client linked to the service name(str)."""
        self._name = 'ontologenius/convertion'
        if name != '':
          self._name = self._name + name
        self._verbose = False
        self._client = rospy.ServiceProxy(self._name, OntologeniusConvertion, True)

    def setVerbose(self, verbose):
        """If verbose(bool) is set to True, the clients will post messages about
           the failure to call the services and about their restoration.
        """
        self._verbose = verbose

    def individualsIndexes2Ids(self, indexes):
       return self._indexes2Ids(indexes, OntologeniusConvertionRequest.INDIVIDUALS)

    def classesIndexes2Ids(self, indexes):
       return self._indexes2Ids(indexes, OntologeniusConvertionRequest.CLASSES)

    def dataPropertiesIndexes2Ids(self, indexes):
       return self._indexes2Ids(indexes, OntologeniusConvertionRequest.DATA_PROPERTIES)

    def objectPropertiesIndexes2Ids(self, indexes):
       return self._indexes2Ids(indexes, OntologeniusConvertionRequest.OBJECT_PROPERTIES)

    def literalsIndexes2Ids(self, indexes):
       return self._indexes2Ids(indexes, OntologeniusConvertionRequest.LITERAL)
    

    def individualsIndex2Id(self, index):
       return self._index2Id(index, OntologeniusConvertionRequest.INDIVIDUALS)

    def classesIndex2Id(self, index):
       return self._index2Id(index, OntologeniusConvertionRequest.CLASSES)

    def dataPropertiesIndex2Id(self, index):
       return self._index2Id(index, OntologeniusConvertionRequest.DATA_PROPERTIES)

    def objectPropertiesIndex2Id(self, index):
       return self._index2Id(index, OntologeniusConvertionRequest.OBJECT_PROPERTIES)

    def literalsIndex2Id(self, index):
       return self._index2Id(index, OntologeniusConvertionRequest.LITERAL)
    

    def individualsIds2Indexes(self, ids):
       return self._ids2Indexes(ids, OntologeniusConvertionRequest.INDIVIDUALS)

    def classesIds2Indexes(self, ids):
       return self._ids2Indexes(ids, OntologeniusConvertionRequest.CLASSES)

    def dataPropertiesIds2Indexes(self, ids):
       return self._ids2Indexes(ids, OntologeniusConvertionRequest.DATA_PROPERTIES)

    def objectPropertiesIds2Indexes(self, ids):
       return self._ids2Indexes(ids, OntologeniusConvertionRequest.OBJECT_PROPERTIES)

    def literalsIds2Indexes(self, ids):
       return self._ids2Indexes(ids, OntologeniusConvertionRequest.LITERAL)
    

    def individualsId2Index(self, id):
       return self._id2Index(id, OntologeniusConvertionRequest.INDIVIDUALS)

    def classesId2Index(self, id):
       return self._id2Index(id, OntologeniusConvertionRequest.CLASSES)

    def dataPropertiesId2Index(self, id):
       return self._id2Index(id, OntologeniusConvertionRequest.DATA_PROPERTIES)

    def objectPropertiesId2Index(self, id):
       return self._id2Index(id, OntologeniusConvertionRequest.OBJECT_PROPERTIES)

    def literalsId2Index(self, id):
       return self._id2Index(id, OntologeniusConvertionRequest.LITERAL)

    def _indexes2Ids(self, indexes, source):
        response = self._call(source, [], indexes)
        if response:
          return response.values_str
        else:
          return None

    def _index2Id(self, index, source):
        response = self._call(source, [], [index])
        if response:
          return response.values_str[0]
        else:
          return None

    def _ids2Indexes(self, ids, source):
        response = self._call(source, ids, [])
        if response:
          return response.values_int
        else:
          return None

    def _id2Index(self, id, source):
        response = self._call(source, [id], [])
        if response:
          return response.values_int[0]
        else:
          return None

    def _call(self, source, values_str, values_int):
        try:
            response = self._client(source, values_str, values_int)
            return response
        except (rospy.ServiceException, rospy.exceptions.TransportTerminated) as e:
            if self._verbose == True:
                print("Failure to call " + self._name)
            self._client = rospy.ServiceProxy(self._name, OntologeniusConvertion, True)
            try:
                response = self._client(source, values_str, values_int)
                if self._verbose == True:
                    print("Restored " + self._name)
                return response
            except (rospy.ServiceException, rospy.exceptions.TransportTerminated) as e:
                if self._verbose == True:
                    print("Failure of service restoration")
                return None