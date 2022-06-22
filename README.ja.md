[English](README.md) | [日本語](README.ja.md)

# kondo_b3m_ros2

近藤科学のB3MシリーズのモータをROS2で扱うためのノードです．
パソコンとモータは[RS485USB/シリアル変換アダプター](https://kondo-robot.com/product/02133)を介して接続します．

# 使い方

このリポジトリに加えて[kondo_b3m_interfaces](https://github.com/Actat/kondo_b3m_interfaces)が必要です．

```
mkdir -p ~/test_ws/src
cd ~/test_ws/src
git clone git@github.com:Actat/kondo_b3m_ros2.git
git clone git@github.com:Actat/kondo_b3m_interfaces.git
```

[RS485USB/シリアル変換アダプター](https://kondo-robot.com/product/02133)を`/dev/ttyUSB0`として認識させます．

```
sudo cp ~/test_ws/src/kondo_b3m_ros2/config/99-kondo-rs485.rules /etc/udev/rules.d/
```

コンパイルして実行します．

```
cd ~/test_ws
colcon build
source install/setup.bash
ros2 run kondo_b3m_ros2 kondo_b3m
```

# topic

TODO

# service

TODO

# parameter

パラメータはすべてオプションです．

|パラメータ名|型|デフォルト値|内容|
|:-:|:--|:--|:--|
|port_name|String value|`"/dev/ttyUSB0"`|RS485USB/シリアル変換アダプターのデバイスファイルの場所です．|
|baudrate|Integer value|`1500000`|ボーレートです．モータの設定値に合わせてください．|
|joint_name_list|String values|`std::vector<std::string>{}`|このリストのID番目の要素を`/b3m_joint_states`へ出力する`sensor_msgs/JointState Message`の`name`にします．
|joint_direction_list|Boolean values|`std::vector<bool>{}`|軸の回転方向を切り替えられます．`true`の場合と設定しない場合は正回転，`false`の場合は逆回転になります．|

# 使用上の注意

接続するモータのIDを0, 1, 2, ..., nのようなゼロ始まりの連番にしてください．
`joint_name_list`や`joint_direction_list`に関係する処理で不具合を起こす可能性があります．
モータのIDを変更するために`util_id_changer`を用意しています．
`ros2 run kondo_b3m_ros2 util_id_changer`で使用できます．
