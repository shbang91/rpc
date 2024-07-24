import asyncio
from typing import Any, Dict, List, Optional
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from foxglove_websocket.types import Parameter
import time
import json

from UI.foxglove.foxglove_sceneorganizer import SceneChannel


def load_params_store() -> Dict[str, Any]:
    param_store: Dict[str, Any] = {
        "n_steps": 3,
        "t_ss": 0.8,
        "t_ds": 1.2,
        "torso_ori_task_weight": [100., 100., 10.],
        "torso_ori_task_kp": [200., 200., 100.],
        "torso_ori_task_kd": [14., 14., 10.],
    }
    return param_store

class Listener(FoxgloveServerListener):
    def __init__(self, param_store: Dict[str, Any]) -> None:
        self._param_store = param_store
        self.modified_param = ""
        self.b_modified = False

    async def on_get_parameters(
            self,
            server: FoxgloveServer,
            param_names: List[str],
            request_id: Optional[str],
    ) -> List[Parameter]:
        return [
            Parameter(name=k, value=v, type="float64")
            for k, v in self._param_store.items()
            if k in param_names or len(param_names) == 0
        ]

    async def on_set_parameters(
            self,
            server: FoxgloveServer,
            params: List[Parameter],
            request_id: Optional[str],
    ):
        for param in params:
            if not param["name"].startswith("read_only"):

                # save the parameter that changed
                if self._param_store[param["name"]] != param["value"]:
                    self.modified_param = param["name"]
                self._param_store[param["name"]] = param["value"]
        param_names = [param["name"] for param in params]

        self.b_modified = True
        return [
            Parameter(name=k, value=v, type="float64")
            for k, v in self._param_store.items()
            if k in param_names
        ]

    def has_been_modified(self):
        return self.b_modified

    def reset(self):
        self.b_modified = False

    def is_cmd_triggered(self, cmd: str) -> bool:
        if cmd == self.modified_param:
            return True
        return False

    def get_param_modified(self):
        return self.modified_param

    def get_val(self, key):
        return self._param_store[key]


async def run(step_listener: Listener):
    async with FoxgloveServer(
            "0.0.0.0",
            8766,
            "Control param server",
            capabilities=["parameters", "parametersSubscribe"]) as server:
        param_store = step_listener._param_store
        server.set_listener(step_listener)
        param_chan_id = await SceneChannel(True, "n_steps", "json", "n_steps",
                                           ["n_steps"]).add_chan(server)

        while True:
            # check every two seconds if the parameters have been modified
            await asyncio.sleep(2.0)
            await server.update_parameters(
                [Parameter(name="n_steps", value=param_store["n_steps"], type="float64")]
            )

            if step_listener.has_been_modified():
                print("Parameters have been modified")
                print(f"Modified parameter: {step_listener.get_param_modified()}")
                print(f"New value: {step_listener.get_val(step_listener.get_param_modified())}")
                now = time.time_ns()
                await server.send_message(param_chan_id, now, json.dumps(
                    {"n_steps": step_listener.get_val("n_steps")}).encode("utf8"))
                step_listener.reset()


#TODO remove?
def paramtest(server,step_listener, param_store):
    server.set_listener(step_listener)

    server.update_parameters(
        [Parameter(name="n_steps", value=param_store["n_steps"], type=None)]
    )