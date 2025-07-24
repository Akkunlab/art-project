# DMX-Controller

DMX-Controller は、**TouchDesignerを使用してuDMX信号を制御する**アプリケーションです。

## Required Libraries

このプログラムを使用するには、以下のPythonライブラリが必要です：

- [udmx-pyusb](https://github.com/dhocker/udmx-pyusb) （uDMX信号の送受信）

## How to Use

### 1. Clone the Repository

最初に、このリポジトリをローカル環境にクローンしてください。

```bash
git clone https://github.com/akkunlab/art-project.git
cd art-project
```

### 2. Install Required Libraries

TouchDesignerのPython環境に`udmx-pyusb`をインストールする必要があります。

```bash
"C:\Program Files\Derivative\TouchDesigner\bin\python.exe" -m pip install udmx-pyusb
```

### 3. Run the Application

1. uDMXインターフェイスを接続する。
2. TouchDesignerで`dmx_controller.toe`を開く。
3. `Out`ボタンを押してDMX信号の出力を有効にする。
