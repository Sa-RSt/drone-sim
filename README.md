# drone-sim

Esse é um simples projeto pessoal que simula um drone com controlador PID, que mostra o drone em tempo real usando 3D falso.

## Como executar
`python3 -m venv .venv` - Criar um virtualenv para instalar dependências (opcional)  

`source .venv/bin/activate` (Linux) ou `.\.venv\Scripts\activate` (Windows) - Ativar o virtualenv (se tiver criado)  

`pip install -r requirements.txt` - Instalar as dependências  

`python -m dronesim` - Executar o módulo  

## Como usar
Enquanto o programa estiver aberto, o terminal exibirá informações pontuais sobre o estado da simulação. Além disso, pode-se acessar as funcionalidades do programa apertando certas teclas. Elas são:

* **ESPAÇO**: Iniciar/parar gravação. Quando parada a gravação, a evolução do estado da simulação é gravada em uma série de gráficos dentro da pasta `out/`.
* **ESC**: Fechar o programa.
* **P**: Controlar o drone usando o controlador PID.
* **Setas**: Ajustar a velocidade horizontal desejada enquanto controlando o drone com PID. Segure **Shift** para fazer ajustes menos finos.
* **Q e E**: Enquanto usado o PID, ajusta a velocidade vertical desejada, aumentando-a para cima com **Q** e para baixo com **E**. Segure **Shift** para fazer ajustes menos finos.
* **M**: Controlar o drone manualmente.
* **R**: Reinicializar a posição do drone, sem alterar nenhum outro estado.
* **BACKSPACE**: Reiniciar todo o estado da simulação, incluindo o relógio e os estados dos controladores PID.
* **X**: Pausar a simulação.
* **H**: Habilitar/desabilitar o modo de alta precisão. Nesse modo, em vez de a simulação ser executada em tempo real, são usados passos com tempo de 1 milissegundo.

A interface também conta com sliders para ajustar a propulsão dos motores individuais e para adicionar aleatoriedade à densidade do ar (ADA) ou aleatoriedade ao desempenho dos motores (ADM), para ver como os controladores se ajustam à presença de imperfeições.
