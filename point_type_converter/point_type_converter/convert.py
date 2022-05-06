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

import struct

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField

# PointCloud2 message format.

datatypes = [
    np.int8, np.uint8,
    np.int16, np.uint16,
    np.int32, np.uint32,
    np.float32, np.float64
]

format_characters = [
    'b', 'B',
    'h', 'H',
    'i', 'I',
    'f', 'd'
]


def get_datatype_id(datatype):
    return datatypes.index(datatype) + 1


def get_format_character(datatype_int):
    return format_characters[datatype_int - 1]


def get_datatype(datatype_int):
    return datatypes[datatype_int - 1]


def data_size(datatype):
    return np.dtype(datatype).itemsize


def create_point_format(fields, point_step):
    format_ = ''
    index = 0
    for f in fields:
        if index < f.offset:
            format_ += 'x' * (f.offset - index)
            index = f.offset
        format_ += get_format_character(f.datatype)
        index += data_size(get_datatype(f.datatype))

    if index < point_step:
        format_ += 'x' * (point_step - index)
    return format_


def create_points_format(fields, n_points, point_step, is_bigendian):
    endian = '>' if is_bigendian else '<'
    point_format = create_point_format(fields, point_step)
    return endian + point_format * n_points


def unpack_point_bytes(bytes_data, fields, point_step, is_bigendian):
    assert len(bytes_data) % point_step == 0, \
           'Data size must be mutiple of point step'
    n_points = len(bytes_data) // point_step
    format_ = create_points_format(fields, n_points, point_step, is_bigendian)
    unpacked = struct.unpack(format_, bytes_data)
    return split(unpacked, len(fields))


def pack_point_data(point_data, fields, point_step, is_bigendian):
    n_points = len(point_data)
    format_ = create_points_format(fields, n_points, point_step, is_bigendian)
    return struct.pack(format_, *flatten(point_data))


class BytesToPoint(object):

    def __init__(self, fields):
        self.names = [f.name for f in fields]
        self.offsets = [f.offset for f in fields]
        self.types = [get_datatype(f.datatype) for f in fields]
        self.sizes = [data_size(datatype) for datatype in self.types]

    def __call__(self, point_byte):
        def extract(offset, datatype, size):
            return point_byte[offset:offset+size].view(datatype)[0]

        d = zip(self.names, self.offsets, self.types, self.sizes)
        return {n: extract(o, t, s) for n, o, t, s in d}


def flatten(chunks):
    return [e for chunk in chunks for e in chunk]


def make_point_field(name, offset, datatype, count):
    return PointField(name=name, offset=offset,
                      datatype=get_datatype_id(datatype), count=count)


def split(collection, chunk_size):
    n = len(collection)
    m = chunk_size
    return tuple(collection[i:i+m] for i in range(n) if i % m == 0)


def find_indices(fields, retained_names):
    return [i for i in range(len(fields)) if fields[i].name in retained_names]


def filter_point_data(point_tuples, input_fields, output_fields):
    output_names = {f.name for f in output_fields}
    indices = find_indices(input_fields, output_names)
    return tuple(tuple(chunk[i] for i in indices) for chunk in point_tuples)


output_point_step = 32


def make_fields():
    return [
        make_point_field('x', 0, np.float32, 1),
        make_point_field('y', 4, np.float32, 1),
        make_point_field('z', 8, np.float32, 1),
        make_point_field('padding', 12, np.float32, 1),
        make_point_field('intensity', 16, np.float32, 1),
        make_point_field('ring', 20, np.uint16, 1),
    ]


class PointTypeConverter(Node):

    def __init__(self, *args, **kwargs):
        super(PointTypeConverter, self).__init__(
            'point_type_converter', *args, **kwargs)
        self.subscription = self.create_subscription(
            PointCloud2, '/os1_cloud_node/points', self.callback, 10)
        self.subscription  # prevent unused variable warning

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL
        )
        self.publisher = self.create_publisher(PointCloud2, '/points_raw', qos)

    def callback(self, input_cloud):
        input_fields = input_cloud.fields
        input_fields.append(make_point_field('padding', 12, np.float32, 1))
        input_fields = sorted(input_fields, key=lambda f: f.offset)
        unpacked = unpack_point_bytes(
            input_cloud.data, input_fields,
            input_cloud.point_step, input_cloud.is_bigendian)

        output_fields = make_fields()
        unpacked = tuple(c for c in unpacked if not (c[0] == 0. and c[1] == 0. and c[2] == 0.))
        filtered = filter_point_data(unpacked, input_fields, output_fields)

        self.get_logger().info('len(filtered) = {}'.format(len(filtered)))

        data = pack_point_data(
            filtered, output_fields, output_point_step, False)

        output_cloud = PointCloud2()
        output_cloud.header = input_cloud.header

        output_cloud.height = 1
        output_cloud.width = len(filtered)

        output_cloud.fields = output_fields

        output_cloud.is_bigendian = False
        output_cloud.point_step = output_point_step
        output_cloud.row_step = output_point_step * output_cloud.width
        output_cloud.data = data
        output_cloud.is_dense = True

        self.publisher.publish(output_cloud)


def main(args=None):
    rclpy.init(args=args)

    converter = PointTypeConverter()

    rclpy.spin(converter)

    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
