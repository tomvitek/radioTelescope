import rospy
from mount_ctrl.msg import status, cmd
class Position:
    def __init__(self, hr = 0.0, dec = 0.0):
        self.hr = hr
        self.dec = dec

class RatelBridge:
    __instance = None
    @staticmethod
    def getInstance():
        if RatelBridge.__instance == None:
            RatelBridge()
        return RatelBridge.__instance
    
    def __init__(self):
        if RatelBridge.__instance != None:
            raise Exception("Constructor called on a singleton class (RatelBridge)")
        else:
            RatelBridge.__instance = self
            self.pos = Position()
            self.status = 0 # Disconnected
            self.scanProgress = 0
            self.sigStrength = 0
            rospy.init_node("ratel_server")
            rospy.loginfo("ratel_server node initialized")
            rospy.Subscriber("mountStatus", status, callback=self.statusCallback)
            self.cmdPublisher = rospy.Publisher("mountCmd", cmd, queue_size=10)

    def statusCallback(self, data):
        self.pos.hr = data.hr
        self.pos.dec = data.dec
        self.status = data.status
        self.scanProgress = data.scanProgress
        self.sigStrength = data.signal

    def getStatus(self):
        return self.status

    def getPosition(self):
        return self.pos

    def getScanProgress(self):
        return self.scanProgress
    
    def getSigStrength(self):
        return self.sigStrength

    def sendGoto(self, pos):
        gotoCmd = cmd()
        gotoCmd.text = "goto " + pos.hr.__str__() + " " + pos.dec.__str__()
        self.cmdPublisher.publish(gotoCmd)
        rospy.loginfo("RatelBridge: send goto command")

    def sendStop(self):
        stopCmd = cmd()
        stopCmd.text = "stop"
        self.cmdPublisher.publish(stopCmd)
        rospy.loginfo("RatelBridge: send stop command")

    def sendHome(self):
        homeCmd = cmd()
        homeCmd.text = "home"
        self.cmdPublisher.publish(homeCmd)
        rospy.loginfo("RatelBridge: send home command")