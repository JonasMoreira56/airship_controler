# airship_controler
### Feature: Integração com Gym para Aprendizado por Reforço

Esta feature adiciona um ambiente compatível com o Gym, permitindo o treinamento de modelos de aprendizado por reforço utilizando o simulador do dirigível no ROS2/Gazebo. O ambiente segue a interface padrão do OpenAI Gym, facilitando a integração com algoritmos de RL populares.

**Principais funcionalidades:**
- Interface Gym customizada para controle do dirigível.
- Observações e recompensas configuráveis.
- Suporte a diferentes algoritmos de aprendizado por reforço.
- Scripts de exemplo para treinamento e avaliação.

**Como usar:**
1. Instale as dependências necessárias (`gym`, `stable-baselines3`, etc.).
2. Execute o ambiente com seu agente de RL preferido.
3. Ajuste os parâmetros de observação e recompensa conforme necessário.

Consulte a documentação na pasta `gym_env/` para mais detalhes e exemplos de uso.
