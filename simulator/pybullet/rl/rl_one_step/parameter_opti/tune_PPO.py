from typing import Any
from typing import Dict

import gymnasium
import optuna
from optuna.pruners import MedianPruner
from optuna.samplers import TPESampler
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
import torch
import torch.nn as nn

import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")
from simulator.pybullet.rl.rl_one_step.envs.new_reward import DracoEnvOneStepMpc


N_TRIALS = 50
N_STARTUP_TRIALS = 10
N_EVALUATIONS = 4
N_TIMESTEPS = int(25000)
EVAL_FREQ = int(N_TIMESTEPS / N_EVALUATIONS)
N_EVAL_EPISODES = 20

#ENV_ID = "CartPole-v1"

DEFAULT_HYPERPARAMS = {
    "policy": "MlpPolicy",
    "env": DracoEnvOneStepMpc(Lx_offset_des = 0., Ly_des = 0., yaw_des=20, mpc_freq=0., sim_dt=0.00175, randomized_command=False, reduced_obs_size=True, render=False)
}


def sample_ppo_params(trial: optuna.Trial) -> Dict[str, Any]:
    """
    Sampler for PPO hyperparams.

    :param trial:
    :return:
    """
    #batch_size = trial.suggest_categorical("batch_size", [8, 16, 32, 64, 128, 256])
    batch_size = trial.suggest_categorical("batch_size", [16, 32, 64, 128, 256])
    #n_steps = trial.suggest_categorical("n_steps", [32, 64, 128, 256, 512])
    n_steps = trial.suggest_categorical("n_steps", [64, 128, 256, 512])
    gamma = trial.suggest_categorical("gamma", [0.98, 0.99, 0.995, 0.999, 0.9999])
    learning_rate = trial.suggest_float("learning_rate", 1e-5, 1e-2, log=True)
    ent_coef = trial.suggest_float("ent_coef", 0.00000001, 0.1, log=True)
    clip_range = trial.suggest_categorical("clip_range", [0.1, 0.2, 0.3])
    #n_epochs = trial.suggest_categorical("n_epochs", [5, 10, 20])
    n_epochs = trial.suggest_categorical("n_epochs", [5, 10, 20])
    gae_lambda = trial.suggest_categorical("gae_lambda", [0.8, 0.9, 0.92, 0.95, 0.98, 0.99, 1.0])
    max_grad_norm = trial.suggest_categorical("max_grad_norm", [0.3, 0.5, 0.7, 0.9, 1, 2, 5])
    vf_coef = trial.suggest_float("vf_coef", 0, 1)
    #net_arch_type = trial.suggest_categorical("net_arch", ["tiny", "small"])
    net_arch_type = trial.suggest_categorical("net_arch", ["small"])

    # Uncomment for gSDE (continuous actions)
    # log_std_init = trial.suggest_float("log_std_init", -4, 1)
    # Uncomment for gSDE (continuous action)
    # sde_sample_freq = trial.suggest_categorical("sde_sample_freq", [-1, 8, 16, 32, 64, 128, 256])
    # Orthogonal initialization
    ortho_init = False
    # ortho_init = trial.suggest_categorical('ortho_init', [False, True])
    # activation_fn = trial.suggest_categorical('activation_fn', ['tanh', 'relu', 'elu', 'leaky_relu'])
    activation_fn_name = trial.suggest_categorical("activation_fn", ["tanh", "relu"])
    # lr_schedule = "constant"
    # Uncomment to enable learning rate schedule
    # lr_schedule = trial.suggest_categorical('lr_schedule', ['linear', 'constant'])
    # if lr_schedule == "linear":
    #     learning_rate = linear_schedule(learning_rate)

    # TODO: account when using multiple envs
    if batch_size > n_steps:
        batch_size = n_steps

    # Independent networks usually work best
    # when not working with images
    net_arch = {
        "tiny": dict(pi=[64], vf=[64]),
        "small": dict(pi=[64, 64], vf=[64, 64]),
        "medium": dict(pi=[256, 256], vf=[256, 256]),
    }[net_arch_type]

    activation_fn = {"tanh": nn.Tanh, "relu": nn.ReLU, "elu": nn.ELU, "leaky_relu": nn.LeakyReLU}[activation_fn_name]

    return {
        "n_steps": n_steps,
        "batch_size": batch_size,
        "gamma": gamma,
        "learning_rate": learning_rate,
        "ent_coef": ent_coef,
        "clip_range": clip_range,
        "n_epochs": n_epochs,
        "gae_lambda": gae_lambda,
        "max_grad_norm": max_grad_norm,
        "vf_coef": vf_coef,
        # "sde_sample_freq": sde_sample_freq,
        "policy_kwargs": dict(
            # log_std_init=log_std_init,
            net_arch=net_arch,
            activation_fn=activation_fn,
            ortho_init=ortho_init,
        ),
    }

class TrialEvalCallback(EvalCallback):
    """Callback used for evaluating and reporting a trial."""

    def __init__(
        self,
        eval_env: gymnasium.Env,
        trial: optuna.Trial,
        n_eval_episodes: int = 5,
        eval_freq: int = 10000,
        deterministic: bool = True,
        verbose: int = 0,
    ):
        super().__init__(
            eval_env=eval_env,
            n_eval_episodes=n_eval_episodes,
            eval_freq=eval_freq,
            deterministic=deterministic,
            verbose=verbose,
        )
        self.trial = trial
        self.eval_idx = 0
        self.is_pruned = False

    def _on_step(self) -> bool:
        if self.eval_freq > 0 and self.n_calls % self.eval_freq == 0:
            super()._on_step()
            self.eval_idx += 1
            self.trial.report(self.last_mean_reward, self.eval_idx)
            # Prune trial if need.
            if self.trial.should_prune():
                print("pruned")
                self.is_pruned = True
                return False
        return True

def objective(trial: optuna.Trial) -> float:
    kwargs = DEFAULT_HYPERPARAMS.copy()
    # Sample hyperparameters.
    kwargs.update(sample_ppo_params(trial))
    # Create the RL model.
    model = PPO(**kwargs)
    # Create env used for evaluation.
    eval_env = Monitor(DracoEnvOneStepMpc(Lx_offset_des = 0., Ly_des = 0., yaw_des=20, mpc_freq=0., sim_dt=0.00175, randomized_command=False, reduced_obs_size=True, render=False))
    # Create the callback that will periodically evaluate and report the performance.
    eval_callback = TrialEvalCallback(
        eval_env, trial, n_eval_episodes=N_EVAL_EPISODES, eval_freq=EVAL_FREQ, deterministic=True
    )

    nan_encountered = False
    try:
        print("hey")
        model.learn(N_TIMESTEPS, callback=eval_callback, progress_bar=True)
        print("HOHO")
    except AssertionError as e:
        # Sometimes, random hyperparams can generate NaN.
        print(e)
        nan_encountered = True
    finally:
        # Free memory.
        model.env.close()
        eval_env.close()

    # Tell the optimizer that the trial failed.
    if nan_encountered:
        return float("nan")

    if eval_callback.is_pruned:
        raise optuna.exceptions.TrialPruned()

    return eval_callback.last_mean_reward



if __name__ == "__main__":
    #---> input new stduy from terminal: bool 
    torch.set_num_threads(1)

    sampler = TPESampler(n_startup_trials=N_STARTUP_TRIALS)
    # Do not prune before 1/3 of the max budget is used.
    pruner = MedianPruner(n_startup_trials=N_STARTUP_TRIALS, n_warmup_steps=N_EVALUATIONS // 3)

    """
    study = optuna.create_study(sampler=sampler, 
                                pruner=pruner, 
                                direction="maximize", 
                                study_name="my_first_distributed", 
                                storage = "mysql+pymysql://root:SenTis2001?@localhost/test_delete",
                                load_if_exists=True)
    """
    study = optuna.load_study(sampler=sampler,
                              pruner=pruner,
                              study_name='seven_try',
                              storage = "mysql://carlitos:Sentis2001?@localhost/PPO_yaw_20")

 
    try:
        study.optimize(objective, n_trials=N_TRIALS, timeout=600, show_progress_bar=True)
    except KeyboardInterrupt:
        print("exept")
        pass

    print("Number of finished trials: ", len(study.trials))

    print("Best trial:")
    trial = study.best_trial

    print("  Value: ", trial.value)

    print("  Params: ")
    for key, value in trial.params.items():
        print("    {}: {}".format(key, value))

    print("  User attrs:")
    for key, value in trial.user_attrs.items():
        print("    {}: {}".format(key, value))

