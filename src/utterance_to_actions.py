#!/usr/bin/env python3

import rospy
import actionlib
from enum import Enum

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from simple_tts.srv import SpeechRequest
from bio_asr.msg import Utterance

ADMINS = ['chris']
CHRIS_DESK_POSE = Pose(Point(1.98,-2.75,0),Quaternion(0,0,0.993,0.114))
#Position(1.980, -2.751, 0.000), Orientation(0.000, 0.000, 0.993, 0.114)
class Actions(Enum):
    UNK=-1
    HELLO=0
    GOTO=1
    CHECK_GOTO=2
    LOCATION=3

class UtteranceToAction:
    def __init__(self):
        self.last_said = {}

        rospy.Subscriber('parsed_utterances', Utterance, self.handle_utterances)

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.tts_client = rospy.ServiceProxy('call_tts', SpeechRequest)

        rospy.loginfo("Attempting to connect to move_base...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Attempting to connect to simple_tts...")
        self.tts_client.wait_for_service()

        rospy.loginfo("Ready to go!")

    def handle_utterances(self, msg:Utterance):
        # simple state machine to translate utterances to actions
        utt = msg.utt
        agent = msg.agent
        action = self.fuzzy_action_search(utt)

        if action == Actions.HELLO:
            rospy.loginfo("GOT ACTION: GREETING")
            self.say_hello(msg.agent, msg.known_agent)

        if action == Actions.GOTO:
            rospy.loginfo("GOT ACTION: GOTO")
            self.goto(utt, agent)

        if action == Actions.CHECK_GOTO:
            rospy.loginfo("GOT ACTION: CHECK GOTO")
            self.check_goto(agent)

        if action == Actions.LOCATION:
            rospy.loginfo("GOT ACTION: LOCATION")
            self.context_goto(utt, agent)

        if action == Actions.UNK:
            rospy.loginfo("NO ACTION: IGNORING")
            return

    def fuzzy_action_search(self, utt:str):
        if any(words in utt.lower() for words in ['hello', 'hi', 'good', 'morning', 'mourning']):
            return Actions.HELLO
        if all(words in utt.lower() for words in ['can', 'go']):
            return Actions.CHECK_GOTO
        if all(words in utt.lower() for words in ['go', 'desk']):
            return Actions.GOTO
        if all(words in utt.lower() for words in ['my', 'desk']):
            return Actions.LOCATION
        return Actions.UNK

    def say_text(self, say):
        self.tts_client(say)

    def goto(self, utt:str, agent:str):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        if agent not in ADMINS:
            self.say_text("Sorry "+str(agent)+", but you are not allowed to give me movement commands.")
            return

        utt = utt.lower().replace('my', agent)
        if all(words in utt for words in ['chris', 'desk']):
            goal.target_pose.pose = CHRIS_DESK_POSE
            rospy.loginfo("Went to location Chris Desk")
            self.say_text("Sure, I will go to location Chris Desk")
            self.move_base_client.send_goal_and_wait(goal)
        else:
            self.say_text("Sorry "+str(agent)+", I'm not sure what you mean by that.")

    def check_goto(self, agent):
        self.last_said[agent] = True
        self.say_text("Sure, just tell me where to go and I'll go there right away.")

    def context_goto(self, utt, agent):
        if self.last_said[agent]:
            self.last_said[agent] = False
            self.goto(utt, agent)
        else:
            self.say_text("Sorry "+str(agent)+", I'm not sure what you mean by that.")

    def say_hello(self, agent, known):
        if known:
            self.say_text("Hello, "+str(agent)+".")
        else:
            self.say_text("Hello there.")

def main():
    rospy.init_node('utterance_to_action')
    uta = UtteranceToAction()
    rospy.spin()

if __name__ == '__main__':
    main()
