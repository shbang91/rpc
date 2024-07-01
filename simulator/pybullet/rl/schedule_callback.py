from stable_baselines3.common.callbacks import BaseCallback
import torch 


class CallbackHyperparamsSchedulePPO_std(BaseCallback):
    """Learning rate = Initial learning rate * training/std"""
    
    def __init__(self):
        super().__init__()
        self._learning_rate_start = None
    

    def _on_rollout_end(self) -> None:
        if self._learning_rate_start is None:
            self._learning_rate_start = self.model.learning_rate
        std = torch.exp(self.model.policy.log_std).mean().item()
        self.model.learning_rate = self._learning_rate_start * std
        self.model._lr_schedule = self._learning_rate_start * std
        self.model._setup_lr_schedule()
        print("hey")
        print(self.model._lr_schedule)

    def _on_step(self) -> bool:
        return True
    def _on_training_start(self) -> None:
        pass

    def _on_rollout_start(self) -> None:
        pass

    def _on_training_end(self) -> None:
        pass

class CallbackHyperparamsSchedulePOO_mean_len(BaseCallback):
    """Learning rate = Initial learning rate * training/std"""
    
    def __init__(self):
        super().__init__()
        self._learning_rate_start = 3e-4
        self._learning_rate_end = 5e-6
        self._num_step_start = 18
        self._max_num_step = 30
        self._count = 0
        self._av_len = 0
        self._len_list = 0
        self._num_windows = 3
        self._window_1 = 0
        self._window_2 = 0

    def _on_rollout_start(self) -> None:
        self._count = 0
        self._av_len = 0
        self._len_list = 0

    def _on_step(self) -> bool:
        if len(self.training_env.get_attr('episode_lengths')[0]) != self._len_list:
            l = self.training_env.get_attr('episode_lengths')[0][-1]
            self._len_list += 1
            self._av_len = (self._av_len*self._count + l)/(self._count+1)

        return True


    def _on_rollout_end(self) -> None:
        print("rollout")

        x = self._av_len  + self._window_1 + self._window_2
        x /= 3
        if x > 18:
            lr =(self._learning_rate_end-self._learning_rate_start)/(self._max_num_step-self._num_step_start)*(x-self._num_step_start)
            lr += self._learning_rate_start

            self.model.learning_rate = lr
            self.model._lr_schedule = lr
            self.model._setup_lr_schedule()

        self._len_list = 0
        self._window1 = self._window_2
        self._window_2 = self._av_len
    


    def _on_training_start(self) -> None:
        pass

    def _on_training_end(self) -> None:
        pass
