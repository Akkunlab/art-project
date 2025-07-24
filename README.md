# Art Project

Art Projectは、**インタラクティブアートやメディアアートを制作するプロジェクト**です。
このリポジトリでは、アートを制作する際に役立つツールをまとめています。

## Tools

本プロジェクトには、以下のツールが含まれています。

### 1. YDLIDAR-OSC
YDLIDAR-OSC は、**YDLIDAR センサから取得したデータ**を、OSC（Open Sound Control）メッセージとして送信するアプリケーションです。

詳しくは [こちら](./ydlidar-osc/README.md) をご覧ください。

### 2. YDLIDAR-Touch
YDLIDAR-Touch は、**YDLIDAR センサの範囲内で検出されたタッチされた座標を OSC メッセージで送信**するアプリケーションです。YDLIDAR を使用して距離データを取得し、指定された範囲内のタッチイベントをキャプチャし、oscpack ライブラリを使ってネットワーク経由で OSC メッセージを送信します。

詳しくは [こちら](./ydlidar-touch/README.md) をご覧ください。

### 3. DMX-Controller
DMX-Controller は、**TouchDesignerを使用してuDMX信号を制御する**アプリケーションです。

詳しくは [こちら](./dmx-controller/README.md) をご覧ください。

## Required Libraries

プロジェクト全体をビルド・実行するためには、以下のライブラリが必要です：

- [YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK) （LIDARデータの処理）
- [oscpack](http://www.audiomulch.com/~rossb/code/oscpack/) （OSCメッセージの送受信）
- [udmx-pyusb](https://github.com/dhocker/udmx-pyusb) （uDMX信号の送受信）

## How to Use

### 1. Clone the Repository

最初に、このリポジトリをローカル環境にクローンしてください。

```bash
git clone https://github.com/akkunlab/art-project.git
cd art-project
```

### 2. Install Required Libraries

`lib/` ディレクトリに `YDLIDAR SDK` および `oscpack` ライブラリを配置する必要があります。`YDLIDAR` および `oscpack` のバイナリファイル（`.lib`）を`lib/`に配置してください。

また、その他のライブラリも必要に応じてインストールしてください。

```bash
mkdir lib
```

### 3. Build the Project

次に、プロジェクトをビルドします。以下のコマンドでビルドディレクトリを作成し、CMakeを実行してビルドを行います。

```bash
mkdir build
cd build
cmake ..
make
```

Windows で Visual Studio を使用している場合は、生成されたプロジェクトを開き、ビルドします。

### 4. Run the Application

各コンポーネントを実行するには、それぞれのディレクトリ内で指定された方法に従ってください。  

詳細な実行手順は各コンポーネントの README をご確認ください。
