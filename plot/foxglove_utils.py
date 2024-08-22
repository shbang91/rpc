import asyncio
import json
import time
from foxglove_websocket.server import FoxgloveServerListener
from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate


def SubtopicGen(names):
    subs = {}
    for i in names:
        subs[i] = {"type": "number"}
    return json.dumps({"type": "object", "properties": subs})


class ShapeScene:
    def __init__(self):
        self.shapes = {}

    def add_shape(self, name, shape, color, size):
        shape_update = SceneUpdate()
        shape_entity = shape_update.entities.add()
        shape_entity.id = name
        shape_entity.frame_id = name
        framework = getattr(shape_entity, shape)
        shape_model = framework.add()
        shape_model.color.r = color[0]
        shape_model.color.g = color[1]
        shape_model.color.b = color[2]
        shape_model.color.a = color[3]
        if shape == "arrows":
            shape_model.shaft_diameter = size[0]
            shape_model.head_length = size[1]
            shape_model.head_diameter = size[2]
        else:
            shape_model.size.x = size[0]
            shape_model.size.y = size[1]
            shape_model.size.z = size[2]
        # update dictionary of shapes to visualize
        self.shapes[name] = shape_update

    def update(self, name, timestamp):
        self.shapes[name].entities[0].timestamp.FromNanoseconds(timestamp)

    def scale(self, name, quat_force, force_magnitude, timestamp):
        # update timestamp
        self.shapes[name].entities[0].timestamp.FromNanoseconds(timestamp)
        # update force direction
        self.shapes[name].entities[0].arrows[0].pose.orientation.x = quat_force[0]
        self.shapes[name].entities[0].arrows[0].pose.orientation.y = quat_force[1]
        self.shapes[name].entities[0].arrows[0].pose.orientation.z = quat_force[2]
        self.shapes[name].entities[0].arrows[0].pose.orientation.w = quat_force[3]
        # update force scale
        self.shapes[name].entities[0].arrows[
            0
        ].shaft_length = force_magnitude  # scale down

    def serialized_msg(self, name):
        return self.shapes[name].SerializeToString()


class SceneChannel:
    def __init__(self, scdata, topic, encoding, schemaName, schema):
        self.data = scdata
        self.topic = topic
        self.encoding = encoding
        self.schemaName = schemaName
        self.schema = schema

    def add_chan(self, server):
        if self.data is True:
            return server.add_channel(
                {
                    "topic": self.topic,
                    "encoding": self.encoding,
                    "schemaName": self.schemaName,
                    "schema": SubtopicGen(self.schema),
                    "schemaEncoding": "jsonschema",
                }
            )
        else:
            return server.add_channel(
                {
                    "topic": self.topic,
                    "encoding": self.encoding,
                    "schemaName": self.schemaName,
                    "schema": self.schema,
                }
            )

