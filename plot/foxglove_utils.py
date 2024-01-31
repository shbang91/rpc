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

class FoxgloveShapeListener(FoxgloveServerListener):
    def __init__(self, scene_chanel_id, shape, name, size, color):
        self.scene_chan_id = scene_chanel_id
        self.shape = shape
        self.name = name
        self.size = size
        self.color = color

    def on_subscribe(self, server, channel_id):
        name = self.name
        size = self.size
        color = self.color
        scene_chan_id = self.scene_chan_id
        now = time.time_ns()
        if channel_id == scene_chan_id:
            scene_update = SceneUpdate()
            for x in name:
                rgb = color[x]
                entity = scene_update.entities.add()
                entity.timestamp.FromNanoseconds(now)
                entity.id = x
                entity.frame_id = x
                entity.frame_locked = True
                shape_get = getattr(entity, self.shape)
                nodel = shape_get.add()
                nodel.size.x = size[0]
                nodel.size.y = size[1]
                nodel.size.z = size[2]
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
