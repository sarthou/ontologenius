import rospy

from .ClientBase import ClientBase

class ActionClient(ClientBase):

    def __init__(self, name):
        if name == '':
            ClientBase.__init__(self, "actions")
        else:
            ClientBase.__init__(self, "actions/" + name)

    def close(self):
        return self.callNR("close", "")

    def save(self, path):
        return self.callNR("save", path)

    def setLang(self, lang):
        return self.callNR("setLang", lang)

    def getLang(self):
        return self.callStr("getLang", "")

    def add(self, uri):
        return self.callNR("add", uri)

    def fadd(self, file):
        return self.callNR("fadd", file)

    def reset(self):
        return self.callNR("reset", "")
