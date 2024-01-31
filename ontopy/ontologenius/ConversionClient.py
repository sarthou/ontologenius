from .compat.ros import Ontoros, OntoService
import os

from ontologenius.srv import OntologeniusConversion
if os.environ["ROS_VERSION"] == "1":
    from ontologenius.srv import OntologeniusConversionRequest
else:
    from ontologenius.srv._ontologenius_conversion import OntologeniusConversion_Request as OntologeniusConversionRequest

class ConversionClient:

    def __init__(self, name):
        """Constructs a ROS client linked to the service name(str)."""
        self._name = 'ontologenius/conversion'
        if name != '':
          self._name = self._name + "/" + name
        self._verbose = False
        self._client = Ontoros.createService(self._name, OntologeniusConversion)

    def setVerbose(self, verbose):
        """If verbose(bool) is set to True, the clients will post messages about
           the failure to call the services and about their restoration.
        """
        self._verbose = verbose

    def individualsIndexes2Ids(self, indexes):
       return self._indexes2Ids(indexes, OntologeniusConversionRequest.INDIVIDUALS)

    def classesIndexes2Ids(self, indexes):
       return self._indexes2Ids(indexes, OntologeniusConversionRequest.CLASSES)

    def dataPropertiesIndexes2Ids(self, indexes):
       return self._indexes2Ids(indexes, OntologeniusConversionRequest.DATA_PROPERTIES)

    def objectPropertiesIndexes2Ids(self, indexes):
       return self._indexes2Ids(indexes, OntologeniusConversionRequest.OBJECT_PROPERTIES)

    def literalsIndexes2Ids(self, indexes):
       return self._indexes2Ids(indexes, OntologeniusConversionRequest.LITERAL)
    

    def individualsIndex2Id(self, index):
       return self._index2Id(index, OntologeniusConversionRequest.INDIVIDUALS)

    def classesIndex2Id(self, index):
       return self._index2Id(index, OntologeniusConversionRequest.CLASSES)

    def dataPropertiesIndex2Id(self, index):
       return self._index2Id(index, OntologeniusConversionRequest.DATA_PROPERTIES)

    def objectPropertiesIndex2Id(self, index):
       return self._index2Id(index, OntologeniusConversionRequest.OBJECT_PROPERTIES)

    def literalsIndex2Id(self, index):
       return self._index2Id(index, OntologeniusConversionRequest.LITERAL)
    

    def individualsIds2Indexes(self, ids):
       return self._ids2Indexes(ids, OntologeniusConversionRequest.INDIVIDUALS)

    def classesIds2Indexes(self, ids):
       return self._ids2Indexes(ids, OntologeniusConversionRequest.CLASSES)

    def dataPropertiesIds2Indexes(self, ids):
       return self._ids2Indexes(ids, OntologeniusConversionRequest.DATA_PROPERTIES)

    def objectPropertiesIds2Indexes(self, ids):
       return self._ids2Indexes(ids, OntologeniusConversionRequest.OBJECT_PROPERTIES)

    def literalsIds2Indexes(self, ids):
       return self._ids2Indexes(ids, OntologeniusConversionRequest.LITERAL)
    

    def individualsId2Index(self, id):
       return self._id2Index(id, OntologeniusConversionRequest.INDIVIDUALS)

    def classesId2Index(self, id):
       return self._id2Index(id, OntologeniusConversionRequest.CLASSES)

    def dataPropertiesId2Index(self, id):
       return self._id2Index(id, OntologeniusConversionRequest.DATA_PROPERTIES)

    def objectPropertiesId2Index(self, id):
       return self._id2Index(id, OntologeniusConversionRequest.OBJECT_PROPERTIES)

    def literalsId2Index(self, id):
       return self._id2Index(id, OntologeniusConversionRequest.LITERAL)

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
        request = OntologeniusConversionRequest(source = source, values_str = values_str, values_int = values_int)
        return self._client.call(request, self._verbose)