#!/usr/bin/env python3

import rospy
from audio_common_msgs.msg import AudioData, AudioInfo
from bio_asr.msg import AudioBatch


class AudioBatcher(object):
    def __init__(self, hz=4, batchtime=3.0, overlap=1):
        if not overlap < batchtime:
            rospy.logerr("overlap must be less than batch time! Setting overlap to 0.")
        if batchtime < 0:
            rospy.signal_shutdown("batch time must be greater than 0!")
        if overlap < 0:
            rospy.signal_shutdown("overlap time must be greater than 0!")
        self.batchtime = rospy.Duration(batchtime)
        self.overlap = rospy.Duration(overlap)

        self.sub_audio = rospy.Subscriber(
            "audio/audio", AudioData, callback=self.handle_audio_msg, queue_size=200
        )
        self.sub_audio_info = rospy.Subscriber(
            "audio/audio_info",
            AudioInfo,
            callback=self.handle_audio_info_msg,
            queue_size=1,
        )

        self.audio_info = None
        self.batch_data = AudioBatch()
        self.batch_start = rospy.Time.now()

        self.pub_audio_batch = rospy.Publisher(
            "audio/audio_batched", AudioBatch, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(1 / hz), self.run)

    def run(self, event):
        del event
        now = rospy.Time.now()
        if now - self.batch_start > self.batchtime:
            assert self.batch_data.data is not None

            # We've collected enough data here. So, set up for publishing and send that info.
            self.batch_data.overlap = self.overlap
            self.batch_data.length = now - self.batch_start
            self.batch_data.header.stamp = self.batch_start

            self.pub_audio_batch.publish(self.batch_data)

            # Strip the appropriate amount of data to make sure that we're
            # keeping some overlap in the next message.
            data_length = len(self.batch_data.data)
            strip_index = int(
                (self.overlap.to_sec() / self.batchtime.to_sec()) * data_length
            )
            self.batch_data.data = self.batch_data.data[-strip_index:]
            self.batch_start = now

    def handle_audio_msg(self, msg: AudioData):
        assert self.batch_data.data is not None
        self.batch_data.data.append(msg)

    def handle_audio_info_msg(self, msg: AudioInfo):
        if self.audio_info is not None:
            rospy.loginfo(
                "Got new AudioInfo message: this might be fine, but it might"
                "indicate a new audio source, which might break things!"
            )
        self.audio_info = msg


def main():
    rospy.init_node("audio_batcher")
    ab = AudioBatcher()
    rospy.spin()


if __name__ == "__main__":
    main()
