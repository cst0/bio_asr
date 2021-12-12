#!/usr/bin/env python3
import rospy
from typing import List
from bio_asr.msg import AudioFileNotification, Utterance
from bio_asr.srv import (
    SpeakerRecognitionOnFileRequest,
    SpeakerRecognitionOnFileResponse,
    SpeakerRecognitionOnFile,
)
from bio_asr.srv import AppendAudioFileRequest, AppendAudioFileResponse, AppendAudioFile
from bio_asr.srv import AsrOnFileRequest, AsrOnFileResponse, AsrOnFile
from bio_asr.srv import VadOnFileRequest, VadOnFileResponse, VadOnFile
from rospy.rostime import Duration


class BioAsrCore:
    def __init__(self):
        rospy.loginfo("Starting bio_asr core system.")

        self.current_appended_audio:AppendAudioFileResponse = AppendAudioFileResponse()
        self.last_processed_audio:AppendAudioFileResponse = self.current_appended_audio
        self.file_to_process = False

        self.run_vad_on_file = rospy.ServiceProxy("run_vad_on_file", VadOnFile)
        self.run_asr_on_file = rospy.ServiceProxy("run_asr_on_file", AsrOnFile)
        self.run_audio_appender = rospy.ServiceProxy("audio_appender", AppendAudioFile)
        self.run_recog_on_file = rospy.ServiceProxy(
            "run_recog_on_file", SpeakerRecognitionOnFile
        )

        rospy.loginfo("Waiting for VAD service to connect...")
        self.run_vad_on_file.wait_for_service()
        rospy.loginfo("Got it! Waiting for ASR service to connect...")
        self.run_asr_on_file.wait_for_service()
        rospy.loginfo("Got it! Waiting for recognition service to connect...")
        self.run_recog_on_file.wait_for_service()
        rospy.loginfo("Got it! Waiting for audio appending service to connect...")
        self.run_audio_appender.wait_for_service()
        rospy.loginfo("All bio_asr services connected.")

        audio_file_use_topic = "audio_file_updates"
        self.pub_parsed_utterances = rospy.Publisher(
            "parsed_utterances", Utterance, queue_size=10
        )
        self.pub_file_use = rospy.Publisher(
            audio_file_use_topic, AudioFileNotification, queue_size=1
        )
        self.sub_file_use = rospy.Subscriber(
            audio_file_use_topic,
            AudioFileNotification,
            self.handle_file_usage,
            queue_size=10,
        )

        rospy.loginfo("All bio_asr publishers/subscribers ready to go.")
        self.audio_management_queue: List[AudioFileNotification] = []
        self.timer = rospy.timer.Timer(Duration(1), self.run_audio_management_queue)

    def handle_file_usage(self, msg: AudioFileNotification):
        if msg.action == msg.ACTION_AVAILABLE:
            self.audio_management_queue.append(msg)

    def run_audio_management_queue(self, event):
        del event
        if len(self.audio_management_queue) == 0:
            # nice, the queue's been cleaned out!
            rospy.logdebug("[run_audio_management_queue]: queue empty")
            return

        current_file = self.audio_management_queue.pop(0)
        rospy.logdebug(
            "[run_audio_management_queue]: operating on " + str(current_file.path)
        )
        try:
            vad_resp: VadOnFileResponse = self.run_vad_on_file(
                VadOnFileRequest(current_file.metadata, current_file.path)
            )
        except rospy.ServiceException:
            rospy.logerr_throttle(15, "tried calling VAD but it wasn't available")
            return

        if vad_resp.marks.SPEECH in vad_resp.marks.results:
            # if there's speech in here: let's deal with it
            rospy.loginfo("[run_audio_management_queue]: speech detected.")
            req = AppendAudioFileRequest()
            req.append_this = current_file.path
            if not self.file_to_process:
                req.new_file = True

            self.file_to_process = True
            self.current_appended_audio:AppendAudioFileResponse = self.run_audio_appender(req)
        else:
            # we were getting speech but now we're not!
            if not self.file_to_process:
                rospy.logdebug('no new audio to address')
                # we've already dealt with this, skip
                return

            # just making sure we're getting the tail end of the current utterance
            req = AppendAudioFileRequest()
            req.append_this = current_file.path
            self.current_appended_audio:AppendAudioFileResponse = self.run_audio_appender(req)

            self.file_to_process = False
            rospy.logdebug('processing on appended file')
            self.last_processed_audio = self.current_appended_audio

            asr_req = AsrOnFileRequest()
            asr_req.metadata = current_file.metadata
            asr_req.file_path = self.current_appended_audio.appended_file
            asr_resp: AsrOnFileResponse = self.run_asr_on_file(asr_req)
            rospy.loginfo('got an utterance! "' + str(asr_resp.utt.utt) + '"\n(in file '+str(self.current_appended_audio.appended_file)+')')
            found_utt = str(asr_resp.utt.utt)

            recog_req = SpeakerRecognitionOnFileRequest()
            recog_req.metadata = current_file.metadata
            recog_req.file = self.current_appended_audio.appended_file
            recog_resp: SpeakerRecognitionOnFileResponse = self.run_recog_on_file(
                recog_req
            )

            # given responses, create and publish utterance
            utt = Utterance()
            utt.metadata = current_file.metadata
            utt.voice_markers.metadata = current_file.metadata
            utt.voice_markers = vad_resp.marks

            utt.known_agent = recog_resp.found_match
            if utt.known_agent:
                utt.agent = recog_resp.agent

            utt.utt = found_utt

            self.pub_parsed_utterances.publish(utt)


def main():
    rospy.init_node("BioAsrCore", anonymous=False)
    bac = BioAsrCore()
    rospy.spin()


if __name__ == "__main__":
    main()
