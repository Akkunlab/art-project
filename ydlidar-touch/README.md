# YDLIDAR-Touch

これは、**`YDLIDAR` センサーの範囲内で検出されたタッチされた座標を `OSC` メッセージで送信**するアプリケーションです。`YDLIDAR` を使用して距離データを取得し、指定された範囲内のタッチイベントをキャプチャし、`oscpack` ライブラリを使ってネットワーク経由で OSC メッセージを送信します。

## Required Libraries

このプロジェクトをビルドするためには、以下のライブラリが必要です：

- [YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK) （LIDARデータの処理）
- [oscpack](http://www.audiomulch.com/~rossb/code/oscpack/) （OSCメッセージの送受信）

## How to Use

### 1. Clone the Repository

最初に、このリポジトリをローカル環境にクローンしてください。

```bash
git clone https://github.com/akkunlab/art-project.git
cd art-project
```

### 2. Install Required Libraries

`lib/` ディレクトリに `YDLIDAR SDK` および `oscpack` ライブラリを配置する必要があります。`YDLIDAR` および `oscpack` のバイナリファイル（`.lib`）を`lib/`に配置してください。

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

ビルドが完了したら、以下のコマンドで実行できます。

```bash
./ydlidar-touch
```
