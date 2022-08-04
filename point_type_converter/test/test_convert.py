# Copyright 2022 Takeshi Ishita
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Takeshi Ishita nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import unittest

from point_type_converter.convert import (
    create_point_format, create_points_format, filter_point_data,
    pack_point_data, PointTypeConverter, split, unpack_point_bytes)

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField


def test_create_point_format():
    point_step = 48
    fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1),
        PointField(name='intensity', offset=16, datatype=7, count=1),
        PointField(name='t', offset=20, datatype=6, count=1),
        PointField(name='reflectivity', offset=24, datatype=4, count=1),
        PointField(name='ring', offset=26, datatype=2, count=1),
        PointField(name='noise', offset=28, datatype=4, count=1),
        PointField(name='range', offset=32, datatype=6, count=1)
    ]

    format_ = create_point_format(fields, point_step)

    assert format_ == 'fffxxxxfIHBxHxxIxxxxxxxxxxxx'


def test_create_points_format():
    fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1),
        PointField(name='intensity', offset=16, datatype=7, count=1)
    ]

    expected = '>fffxxxxfxxxxfffxxxxfxxxx'
    assert create_points_format(fields, 2, 24, True) == expected

    fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1),
        PointField(name='intensity', offset=16, datatype=7, count=1),
        PointField(name='ring', offset=20, datatype=2, count=1)
    ]
    expected = (
        '<'
        'fffxxxxfBxxxxxxxxxxx'
        'fffxxxxfBxxxxxxxxxxx'
    )
    assert create_points_format(fields, 2, 32, False) == expected


def test_unpack_point_bytes():
    data = (
        b'\x00\x00\x00@\x00\x00\x80@\x00\x00\xc0@\x00\x00\x00\x00\x00\x00 A\x16'
        b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
        b'\x00\x00\x80?\x00\x00@@\x00\x00\xa0@\x00\x00\x00\x00\x00\x00\xa0A\x0b'
        b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
    )

    fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1),
        PointField(name='intensity', offset=16, datatype=7, count=1),
        PointField(name='ring', offset=20, datatype=2, count=1)
    ]

    unpacked = unpack_point_bytes(data, fields, 32, False)
    assert unpacked == ((2., 4., 6., 10., 22), (1., 3., 5., 20., 11))

    fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1),
        PointField(name='padding', offset=12, datatype=7, count=1),
        PointField(name='intensity', offset=16, datatype=7, count=1),
        PointField(name='ring', offset=20, datatype=2, count=1)
    ]

    unpacked = unpack_point_bytes(data, fields, 32, False)
    assert unpacked == ((2., 4., 6., 0., 10., 22), (1., 3., 5., 0., 20., 11))


def test_pack_point_data():
    point_data = ((2., 4., 6., 0., 10., 22), (1., 3., 5., 0., 20., 11))
    fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1),
        PointField(name='padding', offset=12, datatype=7, count=1),
        PointField(name='intensity', offset=16, datatype=7, count=1),
        PointField(name='ring', offset=20, datatype=2, count=1)
    ]

    packed = pack_point_data(point_data, fields, 32, False)
    assert unpack_point_bytes(packed, fields, 32, False) == point_data


def test_split():
    collection = (2., 4., 6., 10., 22, 1., 3., 5., 20., 11)
    chunk_size = 5
    expected = ((2., 4., 6., 10., 22), (1., 3., 5., 20.,  11))
    assert split(collection, chunk_size) == expected


def test_filter_point_data():
    input_fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1),
        PointField(name='padding', offset=12, datatype=7, count=1),
        PointField(name='intensity', offset=16, datatype=7, count=1),
        PointField(name='t', offset=20, datatype=6, count=1),
        PointField(name='reflectivity', offset=24, datatype=4, count=1),
        PointField(name='ring', offset=26, datatype=2, count=1),
    ]

    output_fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1),
        PointField(name='padding', offset=12, datatype=7, count=1),
        PointField(name='intensity', offset=16, datatype=7, count=1),
        PointField(name='ring', offset=20, datatype=2, count=1),
    ]

    point_data = (
        (1., 2., 3., 0., 40., 60, 100, 6),
        (4., 6., 8., 0., 30., 80, 200, 7)
    )

    result = filter_point_data(point_data, input_fields, output_fields)

    expected = (
        (1., 2., 3., 0., 40., 6),
        (4., 6., 8., 0., 30., 7)
    )

    assert result == expected


def make_input_cloud():
    input_cloud = PointCloud2()
    input_cloud.fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1),
        PointField(name='intensity', offset=16, datatype=7, count=1),
        PointField(name='t', offset=20, datatype=6, count=1),
        PointField(name='reflectivity', offset=24, datatype=4, count=1),
        PointField(name='ring', offset=26, datatype=2, count=1),
    ]

    point_data = (
        (1., 2., 3., 10., 50, 100, 2),
        (4., 6., 8., 20., 40, 200, 8)
    )
    data = pack_point_data(point_data, input_cloud.fields, 32, False)
    input_cloud.is_bigendian = False
    input_cloud.is_dense = True
    input_cloud.height = 1
    input_cloud.width = 2
    input_cloud.point_step = 32
    input_cloud.row_step = 64
    input_cloud.data = data
    return input_cloud


class TestPointTypeConverter(unittest.TestCase):

    def setUp(self):
        self.has_called = False

    def check_output_cloud(self, output_cloud):
        result = unpack_point_bytes(
            output_cloud.data, output_cloud.fields,
            output_cloud.point_step, output_cloud.is_bigendian)

        self.assertEqual(
            result,
            ((1., 2., 3., 0., 10., 2), (4., 6., 8., 0., 20., 8)))

        self.has_called = True

        self.assertFalse(output_cloud is None)
        self.assertFalse(output_cloud.is_bigendian)
        self.assertTrue(output_cloud.is_dense)
        self.assertEqual(output_cloud.height, 1)
        self.assertEqual(output_cloud.width, 2)
        self.assertEqual(output_cloud.point_step, 32)
        self.assertEqual(output_cloud.row_step, 64)

    def test_point_type_converter(self):
        context = rclpy.context.Context()

        rclpy.init(context=context)

        pub_node = rclpy.create_node('pub_node', context=context)
        sub_node = rclpy.create_node('sub_node', context=context)

        input_topic = '/points_raw'

        converter = PointTypeConverter(context=context)
        converter

        publisher_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL
        )
        subscription_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_ALL
        )

        pub = pub_node.create_publisher(PointCloud2, input_topic, publisher_qos)
        sub = sub_node.create_subscription(
            PointCloud2, 'points_converted',
            self.check_output_cloud, subscription_qos)
        sub

        executor = MultiThreadedExecutor(context=context)
        executor.add_node(converter)
        executor.add_node(sub_node)

        for _ in range(5):
            pub.publish(make_input_cloud())
            executor.spin_once(timeout_sec=1)

        self.assertTrue(self.has_called)

        pub_node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown(context=context)
