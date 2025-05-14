# Dirigible Control
![Texto Alternativo](./src/dirigible_control/img/defult_airship.jpg)

Controlador autônomo para um dirigível simulado no Gazebo com ROS 2.

## Visão Geral

Este pacote ROS 2 permite controlar um dirigível em um ambiente simulado no Gazebo. Inclui modelos de mundo, modelos de dirigível com câmera, marca de pouso e scripts para inicialização automática do ambiente.
- O controlador está em fase de Desenvolvimento

## Estrutura do Projeto

- `dirigible_control/`: Código fonte do pacote ROS 2.
  - `dirigible_controller.py`: Nó principal de controle do dirigível.
  - `launch/dirigible.launch.py`: Script de lançamento que inicia o Gazebo com o mundo e o nó de controle.
  - `models/`: Modelos SDF do dirigível e marca de pouso.
  - `worlds/world_airship.world`: Mundo do Gazebo com os modelos posicionados.
  - `test/`: Testes de lint e copyright.
- `package.xml`, `setup.py`, `setup.cfg`: Arquivos de configuração do pacote.

## Como Usar

1. **Instale as dependências** do ROS 2 e do Gazebo.
2. **Construa o workspace**:
   ```sh
   colcon build
   ```
3. **Fonteie o ambiente**:
   ```sh
   source install/setup.bash
   ```
4. **Execute o launch**:
   ```sh
   ros2 launch dirigible_control dirigible.launch.py
   ```

## Funcionalidades

- Inicialização automática do Gazebo com o mundo customizado.
- Controle do dirigível via tópico `/cmd_vel` [Em desenvolvimento].
- Simulação de câmera embarcada.
- Marca de pouso para navegação.

## Autor

- Jonas Moreira (jonas.barbosa@ufam.edu.br)

## Licença

Veja o arquivo [package.xml](dirigible_control/package.xml) para detalhes de licença.
