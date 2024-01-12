from ..compat.ros import Ontoros, OntoService

from ontologenius.srv import OntologeniusSparqlIndexService, OntologeniusSparqlIndexServiceRequest

class SparqlIndexClient:
    """The SparqlIndexClient class provides a ROS service to explore ontologenius with SPARQL-like queries based on indexes.
       The variables start with the symbol ? (e.g. ?my_var) and each triplet is separated by a comma.
    """

    def __init__(self, name):
        """Constructs a sparql client.
           Can be used in a multi-ontology mode by specifying the name of the ontology name(str). For classic use, name should be defined as ''.
        """
        self._name = 'sparql_index'
        if name != '':
            self._name = self._name + '/' + name
        self._client = Ontoros.createService('ontologenius/' + self._name, OntologeniusSparqlIndexService, True)


    def call(self, query):
        request = OntologeniusSparqlIndexServiceRequest(query)
        response = self._client.call(request)
        if(response is None):
            return None
        else:
            return (response.names, response.results)
