# Instalando o Turtlesim

Nessa primeira tarefe nos assumimos que você já possui o ROS2 instalado no seu computador. Para começar vamos instalar o Turtlesim.
O Turtlesim pode ser instalado com os seguintes comandos:

```
sudo apt update
sudo apt install ros-humble-turtlesim
```

Para confirmar que tudo está correto execute esse comando:

```
ros2 pkg executables turtlesim
```

A saída deve ser algo assim:
```
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

# Objetivo Final

...

# Tarefa 1: Introdução Turtlesim

#### Objetivo: Introduzir e familiarizar os vocês com o ambiente Turtlesim.

### Parte 1 : Rodar o turtlesim

O turtlesim é um simulador simples criado para aprender ROS2 e para se familiarizar com ele você pode começar executando esse comando:

```
ros2 run turtlesim turtlesim_node
```

Você deverá vem uma janela como essa:

![Alt text](assets/imgs/turtlesim.png)

Como vocês podem ver é uma simples janela com fundo azul e uma tartaruga no centro, sem muito o que fazer por enquanto.

## Parte 2 : Usar o comando do teleop_key para movimentar a tartaruga

Agora que você já tem o turtlesim rodando, você pode usar o comando do teleop_key para movimentar a tartaruga. Para isso abra um novo terminal e execute o seguinte comando:

```
ros2 run turtlesim turtle_teleop_key
```

Você deverá ver uma saída como essa:

![Alt text](assets/imgs/teleop_key_turtlesim.png)

Experimente usar as teclas para movimentar a tartaruga e veja o que acontece.

#### Conclusão

Tá bom, muito legal uma tartaruga andando em uma simples janela, mas o que isso tem a ver com robôtica? Você ainda pode não saber, mas a mesma estrutura que você acaba de usar para controlar essa tartaruga foi usada de diversas maneira para controlar esse robozinho aqui:

![Alt text](assets/imgs/robonaut.jpg)

Esse robô se chama **Robonaut**, e foi construido pela **NASA** utilizando **ROS2**. Nessa foto ele está na Estação Espacial Internacional (ISS) e é controlado por astronautas da NASA. Ele é capaz de realizar diversas tarefas na ISS, como por exemplo, trocar baterias e realizar experimentos científicos. Se você quiser ler mais sobre o Robonaut ou sobre ROS2 no espaço, você pode acessar esse link: [ROS…in…space!](https://www.openrobotics.org/blog/2022/2/2/rosinspace)



### Tarefas:
- [ ] Rodar o turtlesim 
- [ ] Usar o comando do teleop_key para movimentar a tartaruga

# Tarefa 2: Nós e Tópicos no ROS2

#### Objetivo: Ensinar os conceitos básicos de nós e tópicos no ROS2. Para isso vamos criar nosso próprio nó para controlar a tartaruga.

#### Parte 1: Criando um novo pacote

Nesta tarefa, iremos desenvolver nosso próprio nó para controlar uma tartaruga. No entanto, antes de começarmos, precisamos configurar um ambiente chamado 'workspace'. Mas afinal, o que é um 'workspace'? Em termos simples, um 'workspace' é um diretório que contém todos os pacotes ROS2 necessários para um projeto específico. É como um espaço de trabalho dedicado para sua aplicação ROS2.

Para criar um novo pacote dentro do nosso 'workspace', utilizaremos o comando ros2 pkg create. Este comando nos permite criar um novo pacote ROS2, que é uma unidade organizacional fundamental no ROS2. Os pacotes são responsáveis por conter código, bibliotecas, recursos e outros elementos necessários para realizar tarefas específicas dentro de um sistema ROS2. Com a criação deste pacote, estaremos preparados para começar a desenvolver nosso próprio nó e, assim, iniciar o controle da tartaruga.

Antes de criarmos o pacote, vamos primeiro ver a estrutura básica de uma "workspace" do ROS2. A estrutura é a seguinte:

```
tarefa1_ws/
    build/
    install/
    src/
```

Onde "tarefa1_ws" é o nome do nosso workspace. O diretório "src" contém todos os pacotes ROS2 necessários para o nosso projeto. O diretório "build" contém todos os arquivos de compilação gerados durante o processo de compilação. O diretório "install" contém todos os arquivos de instalação gerados durante o processo de instalação. Veremos o que é esse processo de compilação e instalação mais tarde. O que você precisar criar agora é o diretório "tarefa1_ws" e o diretório "src" dentro dele. 

Com isso pronto entre dentro do diretório "src" e execute o seguinte comando:

```
ros2 pkg create --build-type ament_python --node-name hello_word turtlesim_project
```

Agorar vamos ver o que cada parte desse comando significa:

- ros2 pkg create: comando para criar um novo pacote
- --build-type ament_python: define o tipo de pacote que estamos criando, neste caso um pacote em python
- --node-name hello_word: define o nome do nó que estamos criando, neste caso o nome do nó é hello_word
- turtlesim_project: define o nome do pacote que estamos criando, neste caso o nome do pacote é turtlesim_project

Se tudo deu certo você terá uma estrutura de diretorios como essa:

![Alt text](assets/imgs/package_init_vs_code.png)

Objetivo: Ensinar os conceitos de nós e tópicos no ROS2.
Passos:
Criar um novo nó que publique comandos de movimento.
Criar um nó que assine os comandos e mova a tartaruga em resposta.
# Tarefa 3: Mensagens Personalizadas no ROS2

Objetivo: Introduzir mensagens personalizadas no ROS2.
Passos:
Definir uma mensagem personalizada para enviar informações de posição da tartaruga.
Modificar os nós para publicar e assinar essas mensagens personalizadas.
#Tarefa 4: Serviços no ROS2

Objetivo: Ensinar o uso de serviços para tarefas síncronas no ROS2.
Passos:
Criar um serviço que permita ao usuário definir a posição desejada da tartaruga.
Modificar um nó para usar esse serviço para mover a tartaruga para uma posição específica.

# Tarefa 5: Ações no ROS2

Objetivo: Introduzir ações para tarefas assíncronas no ROS2.
Passos:
Criar uma ação que permita ao usuário definir um objetivo de movimento para a tartaruga.
Criar um nó que execute ações para mover a tartaruga de acordo com o objetivo.

# Tarefa Final: Combinação de Tarefas

Objetivo: Criar uma aplicação mais complexa que utiliza todos os conceitos aprendidos.
Passos:
Crie um cenário onde a tartaruga deve seguir um caminho predefinido (usando serviços).
Ao longo do caminho, a tartaruga deve evitar obstáculos (usando ações).
Lembre-se de fornecer explicações detalhadas, exemplos de código e exercícios práticos para os alunos em cada tarefa. Certifique-se também de incluir desafios opcionais para alunos mais avançados e de incentivar a experimentação e a resolução de problemas. Ao final do curso, os alunos devem ter um entendimento sólido dos conceitos básicos do ROS2 e como aplicá-los usando o Turtlesim.