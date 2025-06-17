from gym.envs.registration import register

register(
    id='AirshipLanding-v0',
    entry_point='gym_airship.envs:AirshipLandingEnv',
    max_episode_steps=500, # Ajuste conforme necessário
    reward_threshold=200.0, # Ajuste conforme necessário
)