import rospy
from std_msgs.msg import String
from ontologenius.msg import StampedString

import time
import random
from datetime import datetime

class FeederPublisher:

    def __init__(self, name):
        self._name = name
        pub_topic_name = 'ontologenius/insert'
        if self._name != '':
            pub_topic_name += '/' + self._name
        self._pub = rospy.Publisher(pub_topic_name, String, queue_size=1000)
        pub_topic_name = 'ontologenius/insert_stamped'
        if self._name != '':
            pub_topic_name += '/' + self._name
        self._stamped_pub = rospy.Publisher(pub_topic_name, StampedString, queue_size=1000)
        sub_topic_name = 'ontologenius/end'
        if self._name != '':
            sub_topic_name += '/' + self._name
        self._commit_sub = rospy.Subscriber(sub_topic_name, String, self.commitCallback)
        self._updated = False
        random.seed()
        self._commit_nb = random.randint(1, 100000)

    def addObjectProperty(self, concept_from, property, concept_on, stamp = None):
        msg = '[add]' + concept_from + '|' + property + '|' + concept_on
        if stamp == None:
            stamp = rospy.get_rostime()
        self._publish_stamped(msg, stamp)

    def addDataProperty(self, concept_from, property, type, data, stamp = None):
        msg = '[add]' + concept_from + '|' + property + '|' + type + '#' + data
        if stamp == None:
            stamp = rospy.get_rostime()
        self._publish_stamped(msg, stamp)

    def addInheritage(self, concept_from, concept_on, stamp = None):
        msg = '[add]' + concept_from + '|+|' + concept_on
        if stamp == None:
            stamp = rospy.get_rostime()
        self._publish_stamped(msg, stamp)

    def addLanguage(self, concept_from, lang, name, stamp = None):
        msg = '[add]' + concept_from + '|@' + lang + '|' + name
        if stamp == None:
            stamp = rospy.get_rostime()
        self._publish_stamped(msg, stamp)

    def addConcept(self, concept_from, stamp = None):
        msg = '[add]' + concept_from + '|'
        if stamp == None:
            stamp = rospy.get_rostime()
        self._publish_stamped(msg, stamp)

    def removeProperty(self, concept_from, property, stamp = None):
        msg = '[del]' + concept_from + '|' + property + '|_'
        if stamp == None:
            stamp = rospy.get_rostime()
        self._publish_stamped(msg, stamp)

    def removeObjectProperty(self, concept_from, property, concept_on, stamp = None):
        msg = '[del]' + concept_from + '|' + property + '|' + concept_on
        if stamp == None:
            stamp = rospy.get_rostime()
        self._publish_stamped(msg, stamp)

    def removeDataProperty(self, concept_from, property, type, data, stamp = None):
        msg = '[del]' + concept_from + '|' + property + '|' + type + '#' + data
        if stamp == None:
            stamp = rospy.get_rostime()
        self._publish_stamped(msg, stamp)

    def removeInheritage(self, concept_from, concept_on, stamp = None):
        msg = '[del]' + concept_from + '|+|' + concept_on
        if stamp == None:
            stamp = rospy.get_rostime()
        self._publish_stamped(msg, stamp)

    def removeLanguage(self, concept_from, lang, name, stamp = None):
        msg = '[add]' + concept_from + '|@' + lang + '|' + name
        if stamp == None:
            stamp = rospy.get_rostime()
        self._publish_stamped(msg, stamp)

    def removeConcept(self, concept_from, stamp = None):
        msg = '[del]' + concept_from + '|'
        if stamp == None:
            stamp = rospy.get_rostime()
        self._publish_stamped(msg, stamp)

    def getNumSubscribers(self):
        return self._pub.get_num_connections()

    def waitConnected(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.getNumSubscribers() == 0:
            rate.sleep()

    def waitUpdate(self, timeout = 100000000):
        self._updated = False

        start_time = datetime.now()
        self._sendNop()

        while not rospy.is_shutdown() and not self._updated and (self.millis_interval(start_time, datetime.now()) < timeout) :
            time.sleep(.001)

        if self._updated == True:
            return True
        else:
            return False

    def commitAuto(self, timeout = 100000000):
        commit_name = str(self._commit_nb)
        self._commit_nb = self._commit_nb + 1

        if self.commit(commit_name, timeout):
            return commit_name
        else:
            return ''

    def commit(self, commit_name, timeout = 100000000):
        self._updated = False
        msg = '[commit]' + commit_name + '|'

        start_time = datetime.now()
        self._publish_stamped(msg, rospy.get_rostime())

        while not rospy.is_shutdown() and not self._updated and (self.millis_interval(start_time, datetime.now()) < timeout) :
            time.sleep(.001)

        if self._updated == True:
            return True
        else:
            return False

    def checkout(self, commit_name, timeout = 100000000):
        self._updated = False

        start_time = datetime.now()
        self._publish_stamped('[checkout]' + commit_name + '|', rospy.get_rostime())

        while not rospy.is_shutdown() and not self._updated and (self.millis_interval(start_time, datetime.now()) < timeout) :
            time.sleep(.001)

        if self._updated == True:
            return True
        else:
            return False

    def _sendNop(self):
        self._publish_stamped('[nop]nop|', rospy.get_rostime())

    def _publish(self, data):
        self._pub.publish(data)

    def _publish_stamped(self, data, stamp):
        self._stamped_pub.publish(data, stamp)

    def commitCallback(self, data):
        if data.data == 'end':
            self._updated = True

    def millis_interval(self, start, end):
        diff = end - start
        millis = diff.days * 24 * 60 * 60 * 1000
        millis += diff.seconds * 1000
        millis += diff.microseconds / 1000
        return millis
