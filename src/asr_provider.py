#!/usr/bin/env python3

import rospy
import rospkg
import os
from bio_asr.srv import AsrOnFile, AsrOnFileRequest, AsrOnFileResponse
from bio_asr.msg import AudioFileNotification, Utterance
from speechbrain.pretrained import EncoderDecoderASR


class AsrProvider:
    def __init__(self):
        rospy.loginfo('prepping')
        rp = rospkg.RosPack()
        path = rp.get_path("bio_asr")
        data_dir = os.path.join(path, "data")
        model_dir = os.path.join(path, "pretrained_models")

        # transcription clutters symlinks everywhere if we aren't in this dir

        self.asr_model = EncoderDecoderASR.from_hparams(
            source="speechbrain/asr-crdnn-rnnlm-librispeech",
            savedir=os.path.join(model_dir, "asr-crdnn-rnnlm-librispeech"),
        )

        audio_file_use_topic = "audio_file_updates"
        self.pub_file_use = rospy.Publisher(
            audio_file_use_topic, AudioFileNotification, queue_size=1
        )

        os.chdir('/tmp')
        rospy.Service("run_asr_on_file", AsrOnFile, self.provide_asr_on_file)
        rospy.loginfo('ready')

    def provide_asr_on_file(self, req: AsrOnFileRequest):
        assert type(req.file_path) is str
        transcription = self.asr_model.transcribe_file(req.file_path)
        return AsrOnFileResponse(Utterance(metadata=req.metadata, utt=transcription))


def main():
    rospy.init_node("asr_provider", anonymous=False)
    ap = AsrProvider()
    rospy.spin()


if __name__ == "__main__":
    main()
