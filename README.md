# Turtlebot-Teleop

Um simples teleop do turtlebot3. Desenvolvido como atividade ponderada no inteli

Foi adicionada uma funcionalidade de ver a câmera no front-end

Para rodar, vá para `src`.

Em `src/teleop_ws` basta rodar

```sh
source build.sh
ros2 run turtlebot_teleop teleop_node
```

Para rodar o node da câmera, na mesma pasta rode
```sh 
source build.sh
ros2 run teleop_camera camera
```

Rode o rosbridge para websockets

```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

E então, abra o arquivo `src/frontend/index.html` no seu navegador.

# Demonstração

https://github.com/Eduardo-Barreto/Turtlebot-Teleop/assets/34964398/9b7f6062-e4e1-4845-b3cf-915184aad3cd

> Nessa demonstração a câmera usada foi a webcam do notebook, pois o ambiente de simulação não possui uma câmera.

