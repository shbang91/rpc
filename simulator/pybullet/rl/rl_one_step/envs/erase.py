        

from stable_baselines3.ppo import PPO
from burn_in import DracoEnvOneStepMpc
import torch
env = DracoEnvOneStepMpc(0,0,0,0,0)
model = PPO('MlpPolicy', env)
a = model.get_parameters()

print(a['policy']['action_net.weight'])
print(a['policy']['action_net.bias'])

a['policy']['action_net.weight'] = torch.zeros_like(a['policy']['action_net.weight'])
a['policy']['action_net.bias'] = torch.zeros_like(a['policy']['action_net.bias'])

print(a['policy'])
model.set_parameters(a)
