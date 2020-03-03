import rospy
from std_msgs.msg import String

import time
from datetime import datetime

class FeederPublisher:

    def __init__(self, name):
        self._name = name
        pub_topic_name = 'ontologenius/insert'
        if self._name != '':
            pub_topic_name += '/' + self._name
        self._pub = rospy.Publisher(pub_topic_name, String, queue_size=1000)
        sub_topic_name = 'ontologenius/end'
        if self._name != '':
            sub_topic_name += '/' + self._name
        self._commit_sub = rospy.Subscriber(sub_topic_name, String, self.commitCallback)
        self._commited = False

    def addProperty(self, concept_from, property, concept_on):
        msg = '[add]' + concept_from + '|' + property + '|' + concept_on
        self._publish(msg)

    def addProperty(self, concept_from, concept_property, type, data):
        msg = '[add]' + concept_from + '|' + property + '|' + type + '#' + data
        self._publish(msg)

    def addInheritage(self, concept_from, concept_on):
        msg = '[add]' + concept_from + '|+|' + concept_on
        self._publish(msg)

    def addLanguage(self, concept_from, lang, name):
        msg = '[add]' + concept_from + '|@' + lang + '|' + name
        self._publish(msg)

    def addConcept(self, concept_from):
        msg = '[add]' + concept_from + '|'
        self._publish(msg)

    def removeProperty(self, concept_from, property):
        msg = '[del]' + concept_from + '|' + property + '|_'
        self._publish(msg)

    def removeProperty(self, concept_from, property, concept_on):
        msg = '[del]' + concept_from + '|' + property + '|' + concept_on
        self._publish(msg)

    def removeProperty(self, concept_from, property, type, data):
        msg = '[del]' + concept_from + '|' + property + '|' + type + '#' + data
        self._publish(msg)

    def removeInheritage(self, concept_from, concept_on):
        msg = '[del]' + concept_from + '|+|' + concept_on
        self._publish(msg)

    def removeLanguage(self, concept_from, lang, name):
        msg = '[add]' + concept_from + '|@' + lang + '|' + name
        self._publish(msg)

    def removeConcept(self, concept_from):
        msg = '[del]' + concept_from + '|'
        self._publish(msg)

    def getNumSubscribers(self):
        return self._pub.get_num_connections()

    def waitConnected(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.getNumSubscribers() == 0:
            rate.sleep()

    def commit(self, timeout = 100000000):
        self._commited = False

        start_time = datetime.now()
        self._sendNop()

        while not rospy.is_shutdown() and not self._commited and (self.millis_interval(start_time, datetime.now()) < timeout) :
            time.sleep(.001)

        if self._commited == True:
            return True
        else:
            return False

    def _sendNop(self):
        self._publish('[nop]nop|')

    def _publish(self, data):
        self._pub.publish(data)

    def commitCallback(self, data):
        if data.data == 'end':
            self._commited = True

    def millis_interval(self, start, end):
        diff = end - start
        millis = diff.days * 24 * 60 * 60 * 1000
        millis += diff.seconds * 1000
        millis += diff.microseconds / 1000
        return millis
