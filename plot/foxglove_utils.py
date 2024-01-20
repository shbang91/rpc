import asyncio
import time
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate


class FoxgloveShapeListener(FoxgloveServerListener):
    def __init__(self, scene_chanel_id, shape, name, color=None):
        if color is None:  # default to solid red
            color = [1, 0, 0, 1]

        self.scene_chan_id = scene_chanel_id
        self.shape = shape
        self.name = name
        self.color = color

    def on_subscribe(self, server, channel_id):
        name = self.name
        color = self.color
        scene_chan_id = self.scene_chan_id

        now = time.time_ns()
        if channel_id == scene_chan_id:
            scene_update = SceneUpdate()
            entity1 = scene_update.entities.add()
            entity1.timestamp.FromNanoseconds(now)
            entity1.id = name
            entity1.frame_id = name
            entity1.frame_locked = True
            shape_get = getattr(entity1, self.shape)
            nodel = shape_get.add()
            nodel.size.x = 2
            nodel.size.y = 2
            nodel.size.z = 2
            nodel.color.r = color[0]
            nodel.color.g = color[1]
            nodel.color.b = color[2]
            nodel.color.a = color[3]
            asyncio.create_task(
                server.send_message(
                    self.scene_chan_id, now, scene_update.SerializeToString()
                )
            )

    def on_unsubscribe(self, server, channel_id):
        pass
