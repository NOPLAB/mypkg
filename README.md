# b64

ROS2パッケージ: 任意のトピックをbase64エンコード/デコードする

## ノード

### b64_encode

任意のトピックをsubscribeし、base64エンコードしてStringとしてpublish

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `input_topic` | `/input` | 入力トピック名 |
| `input_type` | `std_msgs/msg/String` | 入力メッセージ型 |

| トピック | 型 | 説明 |
|---------|-----|------|
| `~/output` | `std_msgs/msg/String` | base64エンコード済み文字列 |

### b64_decode

base64文字列をsubscribeし、デコードして任意の型としてpublish

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `output_topic` | `/output` | 出力トピック名 |
| `output_type` | `std_msgs/msg/String` | 出力メッセージ型 |

| トピック | 型 | 説明 |
|---------|-----|------|
| `~/input` | `std_msgs/msg/String` | base64エンコード済み文字列 |

## 使用例

### 文字列のエンコード

```bash
ros2 run b64 b64_encode --ros-args -p input_topic:=/chatter
```

### 画像のエンコード

```bash
ros2 run b64 b64_encode --ros-args \
  -p input_topic:=/camera/image_raw \
  -p input_type:=sensor_msgs/msg/Image
```

### デコードして復元

```bash
ros2 run b64 b64_decode --ros-args \
  -p output_topic:=/camera/image_restored \
  -p output_type:=sensor_msgs/msg/Image
```

## テスト

```bash
./test/test_b64.sh
```

### テスト環境

- ROS2 Humble (`ros:humble`)
- ROS2 Jazzy (`ros:jazzy`)

## ライセンスおよびコピーライト

© 2025 nop

このプロジェクトはMITライセンスの下で公開されています。詳細は[LICENSE](LICENSE)ファイルをご覧ください。
