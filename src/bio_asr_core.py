#!/usr/bin/env python3
import re
import rospy
from multiprocessing import Process
from typing import List
from bio_asr.msg import AudioFileNotification, Utterance
from bio_asr.srv import (
    SpeakerRecognitionOnFileRequest,
    SpeakerRecognitionOnFileResponse,
    SpeakerRecognitionOnFile,
)
from bio_asr.srv import AsrOnFileRequest, AsrOnFileResponse, AsrOnFile
from bio_asr.srv import VadOnFileRequest, VadOnFileResponse, VadOnFile


class BioAsrCore:
    def __init__(self):
        self.run_vad_on_file = rospy.ServiceProxy("run_vad_on_file", VadOnFile)
        self.run_asr_on_file = rospy.ServiceProxy("run_asr_on_file", AsrOnFile)
        self.run_recog_on_file = rospy.ServiceProxy(
            "run_recog_on_file", SpeakerRecognitionOnFile
        )

        self.sub_audio_file_notifications = rospy.Subscriber(
            "audio_file_notification",
            AudioFileNotification,
            self.handle_audio_file_notifications,
            queue_size=10,
        )

        self.pub_parsed_utterances = rospy.Publisher(
                "parsed_utterances",
                Utterance,
                queue_size=10
        )

        self.audio_management_queue:List[AudioFileNotification] = []

    def handle_audio_file_notifications(self, msg:AudioFileNotification):
        if msg.action == msg.ACTION_AVAILABLE:
            self.audio_management_queue.append(msg)

    def run_audio_management_queue(self, event):
        del event
        current_file = self.audio_management_queue[0]
        rospy.logdebug('Calling VAD on file')
        vad_resp:VadOnFileResponse = self.run_vad_on_file(VadOnFileRequest(current_file.metadata, current_file.path))
        if vad_resp.marks.SPEECH in vad_resp.marks.types:
            # if there's speech in here: let's deal with it
            rospy.logdebug('got speech, now processing')

            # threading util setup
            asr_resp = AsrOnFileResponse()
            recog_resp = SpeakerRecognitionOnFileResponse()
            def call_asr(current_file, resp):
                resp = self.run_asr_on_file(AsrOnFileRequest(current_file.metadata, current_file.path))
            def call_recog(current_file, resp):
                resp = self.run_recog_on_file(SpeakerRecognitionOnFileRequest(current_file.metadata, current_file.path))

            # threading calls to get data
            p_asr = Process(target=call_asr, args=(current_file,asr_resp,))
            p_recog = Process(target=call_recog, args=(current_file,recog_resp,))
            p_asr.start()
            p_recog.start()
            p_asr.join()
            p_recog.join()

            # given responses, create and publish utterance
            utt = Utterance()
            utt.metadata = current_file.metadata
            utt.voice_markers.metadata = current_file.metadata
            utt.voice_markers = vad_resp.marks

            utt.known_agent = recog_resp.found_match
            if utt.known_agent:
                utt.agent = recog_resp.agent

            utt.utt = asr_resp.utt

            self.pub_parsed_utterances.publish(utt)
        else:
            rospy.logdebug('got no speech, not publishing any utterance')

def main():
    rospy.init_node("BioAsrCore", anonymous=False)
    bac = BioAsrCore()
    rospy.spin()


if __name__ == "__main__":
    main()
