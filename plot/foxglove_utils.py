import asyncio
import json
import time
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate

def SubtopicGen(names):
    subs = {}
    for i in names:
        subs[i] = {"type": "number"}
    return json.dumps({"type": "object","properties": subs})


class ScalableArrowsScene:
    def __init__(self):
        self.arrows = {}

    def add_arrow(self, name, color):
        arrow_update = SceneUpdate()
        arrow_entity = arrow_update.entities.add()
        arrow_entity.id = name
        arrow_entity.frame_id = name
        arrow_model = arrow_entity.arrows.add()
        arrow_model.color.r = color[0]
        arrow_model.color.g = color[1]
        arrow_model.color.b = color[2]
        arrow_model.color.a = color[3]
        arrow_model.shaft_diameter = .03
        arrow_model.head_length = .1
        arrow_model.head_diameter = .08
        # update dictionary of arrows to visualize
        self.arrows[name] = arrow_update

    def update(self, name, quat_force, force_magnitude, timestamp):
        # update timestamp
        self.arrows[name].entities[0].timestamp.FromNanoseconds(timestamp)

        # update force direction
        self.arrows[name].entities[0].arrows[0].pose.orientation.x = quat_force[0]
        self.arrows[name].entities[0].arrows[0].pose.orientation.y = quat_force[1]
        self.arrows[name].entities[0].arrows[0].pose.orientation.z = quat_force[2]
        self.arrows[name].entities[0].arrows[0].pose.orientation.w = quat_force[3]

        # update force scale
        self.arrows[name].entities[0].arrows[0].shaft_length = force_magnitude       # scale down

    def serialized_msg(self, name):
        return self.arrows[name].SerializeToString()

class SceneChannel():
    def __init__(self, scdata, topic, encoding, schemaName, schema):
        self.data = scdata
        self.topic = topic
        self.encoding = encoding
        self.schemaName = schemaName
        self.schema = schema

    def add_chan(self, server):
        if self.data is True:
            return server.add_channel({
                "topic": self.topic,
                "encoding": self.encoding,
                "schemaName": self.schemaName,
                "schema": SubtopicGen(self.schema),
                "schemaEncoding": "jsonschema"})
        else:
            return server.add_channel({
                "topic": self.topic,
                "encoding": self.encoding,
                "schemaName": self.schemaName,
                "schema": self.schema,})

def FormHandler(entity, shape, size):
    shape_get = getattr(entity, shape)
    nodel = shape_get.add()
    if(shape == "spheres"):
        nodel.size.x = size[0]
        nodel.size.y = size[1]
        nodel.size.z = size[2]
    # Currently, the arrows change length so are visualized differently
    # if(shape == "arrows"):
        # nodel.shaft_length = size[0]
        # nodel.shaft_diameter = size[1]
        # nodel.head_length = size[2]
        # nodel.head_diameter = size[3]
    return nodel

class FoxgloveShapeListener(FoxgloveServerListener):
    def __init__(self, scene_chanel_id, shape, msgs_list, size_dict, color_dict):
        self.scene_chan_id = scene_chanel_id
        self.shape = shape
        self.msgs_list = msgs_list
        self.size_dict = size_dict
        self.color_dict = color_dict

    def on_subscribe(self, server, channel_id):
        msgs_list = self.msgs_list
        size_dict = self.size_dict
        color_dict = self.color_dict
        scene_chan_id = self.scene_chan_id
        now = time.time_ns()
        if channel_id == scene_chan_id:
            scene_update = SceneUpdate()
            for msg in msgs_list:
                rgb = color_dict[msg]
                entity = scene_update.entities.add()
                entity.timestamp.FromNanoseconds(now)
                entity.id = msg
                entity.frame_id = msg
                entity.frame_locked = True
                #shape_get = getattr(entity, self.shape)
                #nodel = shape_get.add()
                nodel = FormHandler(entity, self.shape, size_dict[msg])
                nodel.color.r = rgb[0]
                nodel.color.g = rgb[1]
                nodel.color.b = rgb[2]
                nodel.color.a = rgb[3]

            asyncio.create_task(
                server.send_message(
                    self.scene_chan_id, now, scene_update.SerializeToString()
                )
            )

    def on_unsubscribe(self, server, channel_id):
        pass
