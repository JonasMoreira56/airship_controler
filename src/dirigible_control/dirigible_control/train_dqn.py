import gymnasium as gym
from stable_baselines3 import DQN
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy
import os
import sys

# Adiciona o diretório raiz do gym-airship ao PYTHONPATH para que ele possa ser encontrado
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import gym_airship # Isso irá registrar o ambiente

def train_dqn():
    env_id = "AirshipLanding-v0"
    log_dir = "./dqn_airship_logs/"
    os.makedirs(log_dir, exist_ok=True)

    # Cria o ambiente
    # make_vec_env pode ser útil para treinamento paralelo, mas para começar, um único env
    env = gym.make(env_id)

    # Configura o modelo DQN
    model = DQN("MlpPolicy", env, verbose=1,
                learning_rate=0.0001,
                buffer_size=100000,
                learning_starts=1000,
                batch_size=32,
                tau=1.0,
                gamma=0.99,
                train_freq=4,
                gradient_steps=1,
                replay_buffer_class=None,
                replay_buffer_kwargs=None,
                optimize_memory_usage=False,
                target_update_interval=1000,
                exploration_fraction=0.1,
                exploration_initial_eps=1.0,
                exploration_final_eps=0.05,
                max_grad_norm=10,
                tensorboard_log=log_dir,
                device="auto") # "cpu", "cuda", "auto"

    # Treinamento
    print("Iniciando o treinamento do DQN...")
    model.learn(total_timesteps=100000, log_interval=4) # Ajuste total_timesteps conforme a necessidade
    print("Treinamento concluído.")

    # Salva o modelo treinado
    model_path = os.path.join(log_dir, "dqn_airship_landing_model")
    model.save(model_path)
    print(f"Modelo salvo em {model_path}")

    # Avalia o modelo treinado
    print("Avaliando o modelo...")
    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
    print(f"Média de recompensa após o treinamento: {mean_reward:.2f} +/- {std_reward:.2f}")

    env.close()

if __name__ == "__main__":
    train_dqn()